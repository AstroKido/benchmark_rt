// go2_eskf_logger — Go2 Pro multi-stream data logger for ESKF offline tuning
//
// Subscribes to rt/lowstate at ~500 Hz and logs IMU, joint, and contact data
// into three CSV files + a JSON metadata file. Read-only: does not publish any
// command, will not disturb motion control.
//
// RT kernel design:
//   - CLOCK_MONOTONIC timestamp captured as the first action in the DDS callback,
//     giving dt_jitter σ ~5–30 µs (vs ~100–200 µs in Python due to GIL/interpreter).
//   - DDS callback thread is raised to SCHED_FIFO priority 80 on first invocation.
//   - Priority-inheriting mutex (PI mutex) prevents priority inversion between the
//     callback thread and the main/status threads.
//   - mlockall + stack prefault eliminate page-fault latency spikes.
//   - All data is buffered in memory; CSV files are written only after collection
//     ends, so disk I/O never contaminates timestamps.
//
// Usage:
//   go2_eskf_logger <iface> [output_prefix]
//
//   iface          network interface connected to Go2 (e.g. eth0)
//   output_prefix  filename prefix   (default: go2_YYYYMMDD_HHMMSS)
//
// Outputs:
//   <prefix>_imu.csv      — quaternion, gyro, acc, rpy (one row per LowState)
//   <prefix>_joints.csv   — q, dq, tau_est for 12 joints
//   <prefix>_contact.csv  — foot_force (raw int16) and foot_force_est for 4 feet
//   <prefix>_meta.json    — recording metadata and column descriptions
//
// Capabilities required:
//   sudo setcap cap_sys_nice,cap_ipc_lock+ep go2_eskf_logger
//   (or run as root)

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include "rt_utils.hpp"

using namespace unitree::robot;

// ── Globals ──────────────────────────────────────────────────────────────────

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }

// ── Joint / foot name tables ─────────────────────────────────────────────────

static constexpr const char* JOINT_NAMES[12] = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
};

static constexpr const char* FOOT_NAMES[4] = { "FR", "FL", "RR", "RL" };

// ── Per-message sample ────────────────────────────────────────────────────────
// All fields from a single LowState message, stored at ~236 bytes each.
// At 500 Hz, 1 hour costs ~425 MB — well within RAM budget.

struct Sample {
    double  t_sys;              // CLOCK_MONOTONIC seconds
    float   quat[4];            // [w, x, y, z]
    float   gyro[3];            // rad/s
    float   acc[3];             // m/s²
    float   rpy[3];             // [roll, pitch, yaw] rad
    float   q[12];              // joint position rad
    float   dq[12];             // joint velocity rad/s
    float   tau_est[12];        // joint torque estimate Nm
    int16_t foot_force[4];      // raw ADC integer (Go2 MCU)
    int16_t foot_force_est[4];  // estimated contact force (raw int16 from IDL)
};

// ── ESKFLogger ────────────────────────────────────────────────────────────────

class ESKFLogger {
public:
    explicit ESKFLogger(std::string prefix)
        : prefix_(std::move(prefix))
    {
        buf_.reserve(500 * 3600); // 500 Hz × 1 h headroom
    }

    // Initialise DDS and block until first LowState arrives (5 s timeout).
    void init(const std::string& iface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel([this](const void* m){ on_lowstate(m); }, 10);

        std::cout << "[init] Waiting for first LowState... " << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(5), [this]{ return ready_; });
        if (!ready_) {
            std::cerr << "TIMEOUT\n[init] No data in 5 s — check interface and robot state.\n";
            std::exit(1);
        }
        std::cout << "OK\n";
        std::cout << "[init] Recording. Press Ctrl-C to stop.\n\n";
    }

    // Block main thread until SIGINT/SIGTERM.
    void run() {
        while (!g_stop.load(std::memory_order_relaxed))
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Called by status thread every 2 s — lock-free read of buffer size.
    void print_status() const {
        std::lock_guard<rt::PIMutex> lk(const_cast<rt::PIMutex&>(mtx_));
        const size_t n = buf_.size();
        if (n < 2) return;
        const double dur = buf_.back().t_sys - buf_.front().t_sys;
        const double hz  = (dur > 0.0) ? (n - 1) / dur : 0.0;
        std::cout << "[status] " << std::setw(7) << n << " msgs | "
                  << std::fixed << std::setprecision(1) << dur << " s | "
                  << std::setprecision(0) << hz << " Hz\n";
    }

    // Flush buffers to disk. Called from main after g_stop is set.
    void save() {
        std::vector<Sample> local;
        {
            std::lock_guard<rt::PIMutex> lk(mtx_);
            local = std::move(buf_);
        }

        if (local.size() < 10) {
            std::cerr << "[logger] Too few samples (" << local.size() << ") — nothing saved.\n";
            return;
        }

        const double duration = local.back().t_sys - local.front().t_sys;
        const double hz       = (local.size() > 1 && duration > 0.0)
                                ? (local.size() - 1) / duration : 0.0;

        std::cout << "\n[logger] Stopping.\n"
                  << "[logger] Total messages : " << local.size() << "\n"
                  << std::fixed << std::setprecision(2)
                  << "[logger] Duration       : " << duration << " s\n"
                  << std::setprecision(1)
                  << "[logger] Average rate   : " << hz << " Hz\n\n";

        write_imu(local);
        write_joints(local);
        write_contact(local);
        write_meta(local.size(), duration, hz);
    }

private:
    // ── DDS callback ──────────────────────────────────────────────────────────

    void on_lowstate(const void* raw) {
        // Raise this thread to SCHED_FIFO once; errors are non-fatal.
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{
            try { rt::set_thread_fifo(80); } catch (...) {}
        });

        // Timestamp is the very first action — before any data copy.
        const double t = rt::mono_now();
        const auto& msg = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw);
        const auto& imu = msg.imu_state();

        Sample s;
        s.t_sys = t;

        for (int i = 0; i < 4; ++i) s.quat[i] = imu.quaternion()[i];
        for (int i = 0; i < 3; ++i) {
            s.gyro[i] = imu.gyroscope()[i];
            s.acc[i]  = imu.accelerometer()[i];
            s.rpy[i]  = imu.rpy()[i];
        }

        const auto& ms = msg.motor_state();
        for (int i = 0; i < 12; ++i) {
            s.q[i]       = ms[i].q();
            s.dq[i]      = ms[i].dq();
            s.tau_est[i] = ms[i].tau_est();
        }

        for (int i = 0; i < 4; ++i) {
            s.foot_force[i]     = msg.foot_force()[i];
            s.foot_force_est[i] = msg.foot_force_est()[i];
        }

        std::lock_guard<rt::PIMutex> lk(mtx_);
        buf_.push_back(s);
        if (!ready_) { ready_ = true; cv_.notify_all(); }
    }

    // ── CSV / JSON writers ────────────────────────────────────────────────────

    void write_imu(const std::vector<Sample>& data) const {
        const std::string path = prefix_ + "_imu.csv";
        std::ofstream f(path);
        if (!f) { std::cerr << "[logger] Cannot open " << path << "\n"; return; }

        f << "t_sys,quat_w,quat_x,quat_y,quat_z,"
             "gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,rpy_r,rpy_p,rpy_y\n";
        f << std::fixed;
        for (const auto& s : data) {
            f << std::setprecision(6) << s.t_sys << std::setprecision(8)
              << ',' << s.quat[0] << ',' << s.quat[1]
              << ',' << s.quat[2] << ',' << s.quat[3]
              << ',' << s.gyro[0] << ',' << s.gyro[1] << ',' << s.gyro[2]
              << ',' << s.acc[0]  << ',' << s.acc[1]  << ',' << s.acc[2]
              << ',' << s.rpy[0]  << ',' << s.rpy[1]  << ',' << s.rpy[2]
              << '\n';
        }
        std::cout << "[logger] IMU     -> " << path
                  << "  (" << data.size() << " rows)\n";
    }

    void write_joints(const std::vector<Sample>& data) const {
        const std::string path = prefix_ + "_joints.csv";
        std::ofstream f(path);
        if (!f) { std::cerr << "[logger] Cannot open " << path << "\n"; return; }

        f << "t_sys";
        for (int i = 0; i < 12; ++i)
            f << ",q_"   << JOINT_NAMES[i]
              << ",dq_"  << JOINT_NAMES[i]
              << ",tau_" << JOINT_NAMES[i];
        f << '\n';

        f << std::fixed << std::setprecision(8);
        for (const auto& s : data) {
            f << std::setprecision(6) << s.t_sys;
            for (int i = 0; i < 12; ++i)
                f << std::setprecision(8)
                  << ',' << s.q[i]
                  << ',' << s.dq[i]
                  << ',' << s.tau_est[i];
            f << '\n';
        }
        std::cout << "[logger] Joints  -> " << path
                  << "  (" << data.size() << " rows)\n";
    }

    void write_contact(const std::vector<Sample>& data) const {
        const std::string path = prefix_ + "_contact.csv";
        std::ofstream f(path);
        if (!f) { std::cerr << "[logger] Cannot open " << path << "\n"; return; }

        f << "t_sys";
        for (int i = 0; i < 4; ++i)
            f << ",force_" << FOOT_NAMES[i] << ",force_est_" << FOOT_NAMES[i];
        f << '\n';

        f << std::fixed << std::setprecision(6);
        for (const auto& s : data) {
            f << s.t_sys;
            for (int i = 0; i < 4; ++i)
                f << ',' << s.foot_force[i]
                  << ',' << s.foot_force_est[i];
            f << '\n';
        }
        std::cout << "[logger] Contact -> " << path
                  << "  (" << data.size() << " rows)\n";
    }

    void write_meta(size_t n, double duration, double hz) const {
        const std::string path = prefix_ + "_meta.json";
        std::ofstream f(path);
        if (!f) { std::cerr << "[logger] Cannot open " << path << "\n"; return; }

        char ts_buf[32];
        std::time_t now = std::time(nullptr);
        std::strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%S", std::gmtime(&now));

        // Build joint_order JSON array inline
        auto json_str_arr = [](const char* const* arr, int len) {
            std::string out = "[";
            for (int i = 0; i < len; ++i) {
                if (i) out += ", ";
                out += '"'; out += arr[i]; out += '"';
            }
            out += ']';
            return out;
        };

        f << "{\n"
          << "  \"prefix\": \""       << prefix_   << "\",\n"
          << "  \"recorded_at\": \""  << ts_buf    << "\",\n"
          << "  \"total_messages\": " << n          << ",\n"
          << std::fixed << std::setprecision(3)
          << "  \"duration_s\": "     << duration   << ",\n"
          << std::setprecision(1)
          << "  \"avg_rate_hz\": "    << hz          << ",\n"
          << "  \"timestamp_clock\": \"CLOCK_MONOTONIC\",\n"
          << "  \"joint_order\": "    << json_str_arr(JOINT_NAMES, 12) << ",\n"
          << "  \"foot_order\": "     << json_str_arr(FOOT_NAMES,  4)  << ",\n"
          << "  \"imu_columns\": ["
             "\"t_sys\","
             "\"quat_w\",\"quat_x\",\"quat_y\",\"quat_z\","
             "\"gyro_x[rad/s]\",\"gyro_y[rad/s]\",\"gyro_z[rad/s]\","
             "\"acc_x[m/s2]\",\"acc_y[m/s2]\",\"acc_z[m/s2]\","
             "\"rpy_r[rad]\",\"rpy_p[rad]\",\"rpy_y[rad]\"],\n"
          << "  \"joint_columns_per_joint\": [\"q[rad]\",\"dq[rad/s]\",\"tau_est[Nm]\"],\n"
          << "  \"contact_columns_per_foot\": [\"foot_force[raw_int16]\",\"foot_force_est[raw_int16]\"],\n"
          << "  \"notes\": \"foot_force and foot_force_est are raw int16 from the Go2 MCU. "
             "Quaternion convention: [w, x, y, z]. "
             "t_sys is CLOCK_MONOTONIC (monotonic, not wall clock) — use for dt, not absolute time.\"\n"
          << "}\n";

        std::cout << "[logger] Meta    -> " << path << "\n";
    }

    // ── Members ───────────────────────────────────────────────────────────────

    std::string prefix_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    mutable rt::PIMutex          mtx_;
    std::condition_variable_any  cv_;
    bool                         ready_ = false;

    std::vector<Sample> buf_;
};

// ── Status printer thread ─────────────────────────────────────────────────────

static void status_thread_fn(const ESKFLogger* logger) {
    while (!g_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        logger->print_status();
    }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <iface> [output_prefix]\n"
                  << "  iface          network interface (e.g. eth0)\n"
                  << "  output_prefix  filename prefix (default: go2_YYYYMMDD_HHMMSS)\n"
                  << "  Ctrl-C stops recording and flushes all data to disk.\n";
        return 1;
    }

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    const std::string iface = argv[1];

    std::string prefix;
    if (argc >= 3) {
        prefix = argv[2];
    } else {
        std::time_t now = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "go2_%Y%m%d_%H%M%S", std::localtime(&now));
        prefix = buf;
    }

    std::cout << "==============================\n"
              << "  Go2 ESKF Data Logger\n"
              << "  Interface : " << iface << "\n"
              << "  Prefix    : " << prefix << "\n"
              << "==============================\n";

    try {
        rt::lock_memory();
        std::cout << "[rt] Memory locked (MCL_CURRENT | MCL_FUTURE)\n";
    } catch (const std::exception& e) {
        std::cerr << "[rt] Warning: " << e.what()
                  << "\n      Run: sudo setcap cap_sys_nice,cap_ipc_lock+ep " << argv[0] << "\n";
    }
    rt::prefault_stack();

    ESKFLogger logger(prefix);
    logger.init(iface);

    std::thread status_t(status_thread_fn, &logger);
    logger.run();
    g_stop.store(true);
    if (status_t.joinable()) status_t.join();

    logger.save();
    return 0;
}
