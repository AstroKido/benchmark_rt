// latency_v3_fix — Go2 Pro callback-phase-aligned latency benchmark
// ─────────────────────────────────────────────────────────────────
// Measures the round-trip latency between a LowCmd torque pulse and its
// reflection in LowState.motor_state[0].tau_est.  The pulse is sent on a
// callback frame boundary so every reported latency is an exact multiple of
// the 2 ms DDS period (frame-aligned, no sub-frame interpolation).
//
// Phases
//   warmup    — publishes damp cmd, waits for ≥430 Hz stable arrival rate
//   idle      — waits PULSE_INTERVAL (250) frames between pulses
//   armed     — writes pulse cmd; records t_send on the next callback tick
//   detecting — watches tau_est for a rise ≥ threshold (tau × 0.2); on hit
//               records latency = (t_recv − t_send) / 1e6 ms
//
// Usage
//   latency_v3_fix <iface> B [--tau <Nm>] [--n <samples>]
//
//   <iface>        network interface connected to Go2 (e.g. eth0)
//   B              condition flag (reserved, must be "B")
//   --tau  <float> torque pulse amplitude in Nm  (default 0.5)
//   --n    <int>   number of valid latency samples to collect (default 300)
//
// Examples
//   latency_v3_fix eth0 B
//   latency_v3_fix eth0 B --tau 0.3 --n 500
//
// Output
//   latency_v3_fixed_results.json   — mean/median/std/p95/p99/min/max/timeouts
//   latency_v3_fixed_samples.csv    — per-sample latency_ms and frame_count
//
// Notes
//   • Robot must be in low-level mode; joint 0 (FR_hip) is used.
//   • Keep the robot secured or lying flat — the torque pulse is small (0.5 Nm
//     default) but repeated 300+ times.
//   • Timeout counter increments when no response is detected within 25 frames.
// ─────────────────────────────────────────────────────────────────
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include "crc32.hpp"
#include "rt_utils.hpp"
#include "stats.hpp"

using namespace unitree::robot;

static constexpr int    JOINT_IDX        = 0;
static constexpr float  KD_DAMP          = 0.5f;
static constexpr float  POS_STOP_F       = 2.146e9f;
static constexpr float  VEL_STOP_F       = 16000.0f;

static constexpr float  DEFAULT_TAU_PULSE = 0.5f;
static constexpr float  DETECT_FRAC       = 0.2f;
static constexpr int    DEFAULT_N_SAMPLES = 300;

static constexpr int    PULSE_INTERVAL   = 250;
static constexpr int    TIMEOUT_FRAMES   = 25;

static constexpr int    WARMUP_WIN       = 50;
static constexpr double WARMUP_MIN_HZ    = 430.0;
static constexpr int    WARMUP_STABLE_WINS = 1;

static volatile bool g_running = true;
void sigint_handler(int) { g_running = false; }

enum class Phase { WARMUP, IDLE, ARMED, DETECTING };

class LatencyV3Fixed {
public:
    LatencyV3Fixed(float tau_pulse, int n_samples)
        : tau_pulse_(tau_pulse), n_samples_(n_samples),
          threshold_(tau_pulse * DETECT_FRAC) {
        samples_.reserve(n_samples + 64);
        frame_counts_.reserve(n_samples + 64);
    }

    void build_cmd(unitree_go::msg::dds_::LowCmd_& cmd, float tau) {
        cmd.head()[0]   = 0xFE;
        cmd.head()[1]   = 0xEF;
        cmd.level_flag() = 0xFF;
        cmd.gpio()       = 0;
        for (int i = 0; i < 20; i++) {
            cmd.motor_cmd()[i].mode() = 0x01;
            cmd.motor_cmd()[i].q()    = POS_STOP_F;
            cmd.motor_cmd()[i].kp()   = 0.0f;
            cmd.motor_cmd()[i].dq()   = VEL_STOP_F;
            cmd.motor_cmd()[i].kd()   = KD_DAMP;
            cmd.motor_cmd()[i].tau()  = 0.0f;
        }
        cmd.motor_cmd()[JOINT_IDX].tau() = tau;
        cmd.crc() = crc32_core(
            reinterpret_cast<uint32_t*>(&cmd),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    }

    void init(const std::string& iface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);

        build_cmd(cmd_damp_,  0.0f);
        build_cmd(cmd_pulse_, tau_pulse_);

        pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
        pub_->InitChannel();

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel(
            [this](const void* msg) { on_lowstate(msg); }, 10);

        std::cout << "Waiting for LowState..." << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(8), [this]{ return first_msg_; });
        if (!first_msg_) { std::cerr << "\nNo LowState received.\n"; std::exit(1); }
        std::cout << " OK\n";
    }

    void run() {
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(300), [this]{ return done_; });
        if (!done_) std::cout << "\nTimeout.\n";

        if (samples_.empty()) { std::cout << "No valid samples.\n"; return; }
        report();
    }

private:
    using Clock = std::chrono::steady_clock;
    using NS    = std::chrono::nanoseconds;

    void on_lowstate(const void* raw) {
        // Elevate DDS callback thread to SCHED_FIFO once on first invocation.
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...) {} });

        auto t_ns = Clock::now().time_since_epoch().count(); // nanoseconds
        const auto& msg = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw);
        float tau = msg.motor_state()[JOINT_IDX].tau_est();

        std::unique_lock<rt::PIMutex> lk(mtx_);
        frame_++;
        int frame   = frame_;
        Phase phase = phase_;
        double t_s  = t_ns * 1e-9;
        ts_win_.push_back(t_s);
        if ((int)ts_win_.size() > WARMUP_WIN) ts_win_.pop_front();

        if (!first_msg_) { first_msg_ = true; cv_.notify_all(); }
        lk.unlock();

        if (phase == Phase::WARMUP) {
            pub_->Write(cmd_damp_);
            if ((int)ts_win_.size() == WARMUP_WIN) {
                double dt = ts_win_.back() - ts_win_.front();
                double hz = (WARMUP_WIN - 1) / dt;
                std::cout << "\rWarmup Hz=" << std::fixed << std::setprecision(2) << hz << std::flush;
                lk.lock();
                if (hz >= WARMUP_MIN_HZ) stable_++;
                else stable_ = 0;
                if (stable_ >= WARMUP_STABLE_WINS) {
                    std::cout << "\nWarmup OK (" << hz << " Hz)\n";
                    phase_ = Phase::IDLE;
                    pulse_frame_ = frame;
                }
            }
            return;
        }

        if (phase == Phase::IDLE) {
            pub_->Write(cmd_damp_);
            lk.lock();
            if (frame - pulse_frame_ >= PULSE_INTERVAL) {
                tau_before_ = tau;
                phase_ = Phase::ARMED;
            }
            return;
        }

        if (phase == Phase::ARMED) {
            pub_->Write(cmd_pulse_);
            lk.lock();
            t_send_ns_   = t_ns;
            pulse_frame_ = frame;
            phase_       = Phase::DETECTING;
            return;
        }

        if (phase == Phase::DETECTING) {
            pub_->Write(cmd_pulse_);
            int k = frame - pulse_frame_;
            float delta = std::fabs(tau - tau_before_);

            if (delta >= threshold_) {
                double latency_ms = (t_ns - t_send_ns_) / 1e6;
                lk.lock();
                samples_.push_back(latency_ms);
                frame_counts_.push_back(k);
                int n = (int)samples_.size();
                phase_       = Phase::IDLE;
                pulse_frame_ = frame;
                lk.unlock();

                if (n % 20 == 0)
                    std::cout << "\n" << n << "/" << n_samples_
                              << " last=" << std::fixed << std::setprecision(3) << latency_ms
                              << "ms k=" << k << std::flush;

                if (n >= n_samples_) {
                    lk.lock(); done_ = true; cv_.notify_all();
                }
            } else if (k >= TIMEOUT_FRAMES) {
                timeouts_++;
                lk.lock();
                phase_       = Phase::IDLE;
                pulse_frame_ = frame;
            }
        }
    }

    void report() {
        auto arr = samples_;
        double mn   = stats::mean(arr);
        double med  = stats::median(arr);
        double sd   = stats::stddev(arr);
        double p95  = stats::percentile(arr, 95.0);
        double p99  = stats::percentile(arr, 99.0);
        double mn_v = stats::vmin(arr);
        double mx_v = stats::vmax(arr);

        std::cout << "\n=== Results ===\n"
                  << "n:       " << arr.size() << "\n"
                  << "mean:    " << mn   << " ms\n"
                  << "median:  " << med  << " ms\n"
                  << "std:     " << sd   << " ms\n"
                  << "p95:     " << p95  << " ms\n"
                  << "p99:     " << p99  << " ms\n"
                  << "min:     " << mn_v << " ms\n"
                  << "max:     " << mx_v << " ms\n"
                  << "timeouts:" << timeouts_ << "\n";

        std::ofstream jf("latency_v3_fixed_results.json");
        jf << std::fixed << std::setprecision(6)
           << "{\n"
           << "  \"n\":        " << arr.size()   << ",\n"
           << "  \"mean\":     " << mn            << ",\n"
           << "  \"median\":   " << med           << ",\n"
           << "  \"std\":      " << sd            << ",\n"
           << "  \"p95\":      " << p95           << ",\n"
           << "  \"p99\":      " << p99           << ",\n"
           << "  \"min\":      " << mn_v          << ",\n"
           << "  \"max\":      " << mx_v          << ",\n"
           << "  \"timeouts\": " << timeouts_     << "\n"
           << "}\n";

        // Write samples CSV for external plotting
        std::ofstream cf("latency_v3_fixed_samples.csv");
        cf << "latency_ms,frame_count\n";
        for (size_t i = 0; i < samples_.size(); i++)
            cf << samples_[i] << "," << frame_counts_[i] << "\n";

        std::cout << "Saved latency_v3_fixed_results.json\n"
                  << "Saved latency_v3_fixed_samples.csv\n";
    }

    float tau_pulse_;
    int   n_samples_;
    float threshold_;

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>    pub_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    unitree_go::msg::dds_::LowCmd_ cmd_damp_{};
    unitree_go::msg::dds_::LowCmd_ cmd_pulse_{};

    rt::PIMutex                  mtx_;
    std::condition_variable_any  cv_;
    bool first_msg_ = false;
    bool done_      = false;

    Phase phase_       = Phase::WARMUP;
    int   frame_       = 0;
    int   pulse_frame_ = 0;
    float tau_before_  = 0.0f;
    int64_t t_send_ns_ = 0;

    int stable_ = 0;
    std::deque<double> ts_win_;

    std::vector<double> samples_;
    std::vector<int>    frame_counts_;
    int timeouts_ = 0;
};

int main(int argc, const char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <iface> B [--tau <val>] [--n <val>]\n";
        return 1;
    }
    std::signal(SIGINT, sigint_handler);

    float tau_pulse  = DEFAULT_TAU_PULSE;
    int   n_samples  = DEFAULT_N_SAMPLES;

    for (int i = 3; i < argc - 1; i++) {
        if (std::string(argv[i]) == "--tau") tau_pulse = std::stof(argv[i+1]);
        if (std::string(argv[i]) == "--n")   n_samples = std::stoi(argv[i+1]);
    }

    try {
        rt::lock_memory();
        std::cout << "Memory locked (MCL_CURRENT | MCL_FUTURE)\n";
    } catch (const std::exception& e) {
        std::cerr << "Warning: " << e.what()
                  << "\n  Run with sudo or: sudo setcap cap_ipc_lock+ep " << argv[0] << "\n";
    }
    rt::prefault_stack();

    LatencyV3Fixed bench(tau_pulse, n_samples);
    bench.init(argv[1]);
    bench.run();
    return 0;
}
