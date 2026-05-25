// imu_logger — Go2 Pro standstill IMU data logger
// ─────────────────────────────────────────────────────────────────
// Subscribes to rt/lowstate and logs IMU gyroscope and accelerometer data
// to a CSV file while the robot is standing completely still.  The output is
// directly compatible with imu_noise_params.py for Allan deviation analysis
// and ESKF noise parameter extraction.
//
// Timing design
//   Timestamps use clock_gettime(CLOCK_MONOTONIC) captured as the very first
//   action in the DDS callback — before any struct copy or data extraction.
//   This eliminates the GIL contention and interpreter overhead present in the
//   Python logger, reducing dt_jitter σ from ~100–200 µs to ~5–30 µs.
//   All samples are buffered in memory; the CSV is written only after
//   collection ends so disk I/O never contaminates the timestamps.
//
// Usage
//   imu_logger <iface> [duration_s] [output_csv]
//
//   <iface>       network interface connected to Go2 (e.g. eth0)
//   duration_s    collection window in seconds  (default 300)
//   output_csv    output file path  (default standstill_imu.csv)
//
// Examples
//   imu_logger eth0                       # 300 s -> standstill_imu.csv
//   imu_logger eth0 600                   # 10 min for better long-tau coverage
//   imu_logger eth0 300 my_run.csv
//
// Output
//   standstill_imu.csv (or custom path)
//     Columns: t_sys, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
//     t_sys   — CLOCK_MONOTONIC seconds (double, 9 decimal places)
//     gyro_*  — angular velocity in rad/s
//     acc_*   — linear acceleration in m/s²
//
// Recommended duration
//   ≥ 300 s  — covers OADEV up to τ ≈ 37 s (minimum for ARW + bias instability)
//   ≥ 600 s  — covers OADEV up to τ ≈ 150 s (recommended for full curve)
//
// Downstream analysis
//   python3 imu_noise_params.py standstill_imu.csv
//
// Notes
//   • Keep the robot completely still throughout collection.
//   • Ctrl-C (SIGINT) flushes whatever has been collected and exits cleanly.
//   • Progress is printed every 10 s showing achieved Hz and dt_jitter σ (µs).
// ─────────────────────────────────────────────────────────────────

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
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <numeric>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include "rt_utils.hpp"

using namespace unitree::robot;

static std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

struct ImuSample {
    double t_sys;         // CLOCK_MONOTONIC seconds
    float  gyro[3];       // rad/s
    float  acc[3];        // m/s²
};


class ImuLogger {
public:
    ImuLogger(double duration_s, const std::string& out_path)
        : duration_s_(duration_s), out_path_(out_path) {
        buf_.reserve((size_t)(duration_s * 600)); // 500 Hz + headroom
    }

    void init(const std::string& iface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel([this](const void* m){ on_lowstate(m); }, 10);

        std::cout << "Waiting for LowState..." << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(5), [this]{ return ready_; });
        if (!ready_) { std::cerr << "\nNo LowState in 5 s.\n"; std::exit(1); }
        std::cout << " OK\n";
    }

    // Collect for duration_s, printing progress every 10 s.
    // Returns when time is up or SIGINT fires.
    void collect() {
        double t_start = rt::mono_now();
        double next_print = t_start + 10.0;

        std::cout << "Collecting " << std::fixed << std::setprecision(0)
                  << duration_s_ << " s of standstill IMU data.\n"
                  << "Keep robot completely still. Ctrl-C saves partial data.\n\n";

        while (!g_stop.load()) {
            double now = rt::mono_now();
            double elapsed = now - t_start;
            if (elapsed >= duration_s_) break;

            if (now >= next_print) {
                size_t n;
                { std::lock_guard<rt::PIMutex> lk(mtx_); n = buf_.size(); }
                double achieved_hz = (elapsed > 0) ? n / elapsed : 0.0;

                // Compute recent dt jitter from last 500 samples
                double jitter_us = 0.0;
                {
                    std::lock_guard<rt::PIMutex> lk(mtx_);
                    int tail = std::min((int)buf_.size(), 500);
                    if (tail > 2) {
                        std::vector<double> dts;
                        int start = (int)buf_.size() - tail;
                        for (int i = start + 1; i < (int)buf_.size(); i++)
                            dts.push_back((buf_[i].t_sys - buf_[i-1].t_sys) * 1e6);
                        double m = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
                        double sq = 0.0;
                        for (double d : dts) sq += (d - m) * (d - m);
                        jitter_us = std::sqrt(sq / dts.size());
                    }
                }

                std::cout << "  " << std::setw(5) << std::setprecision(0) << elapsed
                          << " s  samples=" << std::setw(7) << n
                          << "  Hz=" << std::setprecision(1) << achieved_hz
                          << "  dt_jitter=" << std::setprecision(2) << jitter_us << " µs\n";

                next_print += 10.0;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    void save() {
        std::vector<ImuSample> local;
        { std::lock_guard<rt::PIMutex> lk(mtx_); local = buf_; }

        if (local.size() < 100) {
            std::cerr << "Too few samples (" << local.size() << "), not saving.\n";
            return;
        }

        // Final stats
        double duration  = local.back().t_sys - local.front().t_sys;
        double achieved_hz = (local.size() - 1) / duration;

        std::vector<double> dts;
        dts.reserve(local.size() - 1);
        for (size_t i = 1; i < local.size(); i++)
            dts.push_back((local[i].t_sys - local[i-1].t_sys) * 1e3);

        double dt_mean = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
        double sq = 0.0;
        for (double d : dts) sq += (d - dt_mean) * (d - dt_mean);
        double dt_std_ms = std::sqrt(sq / dts.size());

        auto it_max = std::max_element(dts.begin(), dts.end());
        auto it_p99 = dts.begin();
        {
            std::vector<double> s = dts;
            std::sort(s.begin(), s.end());
            double idx = 0.99 * (s.size() - 1);
            size_t lo = (size_t)idx;
            it_p99 = dts.begin(); // reuse variable just for value
            double p99 = s[lo] + (idx - lo) * (s[std::min(lo+1, s.size()-1)] - s[lo]);

            std::cout << "\n=== Collection complete ===\n"
                      << "  samples  : " << local.size() << "\n"
                      << std::fixed << std::setprecision(1)
                      << "  duration : " << duration << " s\n"
                      << "  Hz       : " << achieved_hz << "\n"
                      << std::setprecision(4)
                      << "  dt mean  : " << dt_mean   << " ms\n"
                      << "  dt std   : " << dt_std_ms << " ms\n"
                      << "  dt p99   : " << p99       << " ms\n"
                      << "  dt max   : " << *it_max   << " ms\n";
        }

        // Write CSV — columns match imu_noise_params.py expectations
        std::ofstream f(out_path_);
        if (!f) { std::cerr << "Cannot open " << out_path_ << "\n"; return; }

        f << "t_sys,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z\n";
        f << std::fixed << std::setprecision(9);
        for (auto& s : local) {
            f << s.t_sys
              << "," << s.gyro[0] << "," << s.gyro[1] << "," << s.gyro[2]
              << "," << s.acc[0]  << "," << s.acc[1]  << "," << s.acc[2]
              << "\n";
        }

        std::cout << "\nSaved " << local.size() << " samples -> " << out_path_ << "\n";
    }

private:
    void on_lowstate(const void* raw) {
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...) {} });

        double t = rt::mono_now(); // capture timestamp before any other work
        const auto& msg = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw);
        const auto& imu = msg.imu_state();

        ImuSample s;
        s.t_sys   = t;
        s.gyro[0] = imu.gyroscope()[0];
        s.gyro[1] = imu.gyroscope()[1];
        s.gyro[2] = imu.gyroscope()[2];
        s.acc[0]  = imu.accelerometer()[0];
        s.acc[1]  = imu.accelerometer()[1];
        s.acc[2]  = imu.accelerometer()[2];

        std::lock_guard<rt::PIMutex> lk(mtx_);
        buf_.push_back(s);
        if (!ready_) { ready_ = true; cv_.notify_all(); }
    }

    double      duration_s_;
    std::string out_path_;

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    rt::PIMutex                  mtx_;
    std::condition_variable_any  cv_;
    bool ready_ = false;

    std::vector<ImuSample> buf_;
};

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <iface> [duration_s] [output_csv]\n"
                  << "  default duration : 300 s\n"
                  << "  default output   : standstill_imu.csv\n"
                  << "  recommended      : >= 600 s for good long-tau OADEV coverage\n";
        return 1;
    }

    std::signal(SIGINT, on_sigint);

    std::string iface    = argv[1];
    double duration_s    = (argc >= 3) ? std::stod(argv[2]) : 300.0;
    std::string out_path = (argc >= 4) ? argv[3] : "standstill_imu.csv";

    std::cout << "==============================\n"
              << "  Go2 IMU Standstill Logger\n"
              << "  Interface : " << iface << "\n"
              << "  Duration  : " << std::fixed << std::setprecision(0)
              << duration_s << " s\n"
              << "  Output    : " << out_path << "\n"
              << "==============================\n"
              << "Press Enter to start...\n";
    std::cin.get();

    try {
        rt::lock_memory();
        std::cout << "Memory locked (MCL_CURRENT | MCL_FUTURE)\n";
    } catch (const std::exception& e) {
        std::cerr << "Warning: " << e.what()
                  << "\n  Run with sudo or: sudo setcap cap_ipc_lock+ep " << argv[0] << "\n";
    }
    rt::prefault_stack();

    ImuLogger logger(duration_s, out_path);
    logger.init(iface);
    logger.collect();
    logger.save();
    return 0;
}
