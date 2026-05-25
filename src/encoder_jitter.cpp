// encoder_jitter — Go2 Pro joint encoder jitter benchmark
// ─────────────────────────────────────────────────────────────────
// Subscribes to rt/lowstate and records joint position (q) and velocity (dq)
// for all 12 joints while the robot stands still.  Jitter is quantified as the
// consecutive-sample difference Δq per joint: a perfect encoder would show
// exactly zero; any spread reveals quantisation noise, update-rate irregularity,
// or DDS transport jitter.
//
// Also records per-message wall-clock arrival times and reports the inter-arrival
// interval distribution, which characterises the health of the 500 Hz DDS link.
//
// Usage
//   encoder_jitter <iface> [duration_s]
//
//   <iface>       network interface connected to Go2 (e.g. eth0)
//   duration_s    collection window in seconds  (default 10)
//
// Examples
//   encoder_jitter eth0          # 10 s quick test
//   encoder_jitter eth0 30       # 30 s for tighter statistics
//
// Output
//   encoder_jitter_results.json       — per-joint Δq std/p99/max + arrival intervals
//   encoder_jitter_samples.csv        — raw t_sys, q[0..11], dq[0..11] per frame
//   encoder_arrival_intervals.csv     — inter-arrival interval (ms) per gap
//
// Notes
//   • Keep the robot completely still (standing or secured) during collection.
//   • Joint order: FR_hip/thigh/calf, FL_hip/thigh/calf,
//                  RR_hip/thigh/calf, RL_hip/thigh/calf  (indices 0–11).
//   • Δq values near encoder LSB (~0.001 rad for typical 12-bit encoders) are normal.
// ─────────────────────────────────────────────────────────────────
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <numeric>

#include "rt_utils.hpp"

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include "stats.hpp"

using namespace unitree::robot;

static constexpr int N_JOINTS = 12;
static const char* JOINT_NAMES[N_JOINTS] = {
    "FR_hip","FR_thigh","FR_calf",
    "FL_hip","FL_thigh","FL_calf",
    "RR_hip","RR_thigh","RR_calf",
    "RL_hip","RL_thigh","RL_calf",
};

struct Sample {
    double t;
    std::array<float, N_JOINTS> q;
    std::array<float, N_JOINTS> dq;
};

class EncoderJitterBench {
public:
    explicit EncoderJitterBench(double duration_s) : duration_s_(duration_s) {
        log_.reserve(static_cast<std::size_t>(duration_s * 600)); // 500 Hz + headroom
    }

    void init(const std::string& iface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel([this](const void* m){ on_lowstate(m); }, 10);

        std::cout << "Waiting for LowState..." << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(5), [this]{ return ready_; });
        if (!ready_) throw std::runtime_error("No LowState in 5 s");
        std::cout << " OK\n";
    }

    void run() {
        std::cout << "Collecting for " << (int)duration_s_
                  << " s — keep robot still...\n";
        std::this_thread::sleep_for(std::chrono::duration<double>(duration_s_));
        std::lock_guard<rt::PIMutex> lk(mtx_);
        std::cout << "Collected " << log_.size() << " samples.\n";
    }

    void report() {
        std::lock_guard<rt::PIMutex> lk(mtx_);
        size_t n = log_.size();
        if (n < 10) { std::cout << "Too few samples.\n"; return; }

        // Inter-arrival intervals
        std::vector<double> intervals_ms;
        for (size_t i = 1; i < n; i++)
            intervals_ms.push_back((log_[i].t - log_[i-1].t) * 1e3);

        // Per-joint consecutive differences (Δq)
        std::array<std::vector<double>, N_JOINTS> dq_consec;
        for (size_t i = 1; i < n; i++) {
            for (int j = 0; j < N_JOINTS; j++)
                dq_consec[j].push_back(log_[i].q[j] - log_[i-1].q[j]);
        }

        std::cout << "\n" << std::string(50, '-') << "\n"
                  << "  Encoder Jitter Report   n=" << n << " samples\n"
                  << std::string(50, '-') << "\n";

        std::cout << "\n  Inter-arrival interval (ms):\n"
                  << "    median=" << std::fixed << std::setprecision(3)
                  << stats::median(intervals_ms)
                  << "  mean="   << stats::mean(intervals_ms)
                  << "+-"        << stats::stddev(intervals_ms)
                  << "  p99="    << stats::percentile(intervals_ms, 99)
                  << "  max="    << stats::vmax(intervals_ms) << "\n";

        std::cout << "\n  Dq jitter (rad, consecutive differences):\n"
                  << "  " << std::left << std::setw(14) << "Joint"
                  << std::right << std::setw(12) << "std"
                  << std::setw(12) << "p99"
                  << std::setw(12) << "max" << "\n";

        // Build JSON
        std::ofstream jf("encoder_jitter_results.json");
        jf << std::fixed << std::setprecision(6);
        jf << "{\n"
           << "  \"n_samples\": " << n << ",\n"
           << "  \"duration_s\": " << (log_.back().t - log_.front().t) << ",\n"
           << "  \"arrival_interval_ms\": {\n"
           << "    \"min\":    " << stats::vmin(intervals_ms) << ",\n"
           << "    \"median\": " << stats::median(intervals_ms) << ",\n"
           << "    \"mean\":   " << stats::mean(intervals_ms) << ",\n"
           << "    \"std\":    " << stats::stddev(intervals_ms) << ",\n"
           << "    \"p99\":    " << stats::percentile(intervals_ms, 99) << ",\n"
           << "    \"max\":    " << stats::vmax(intervals_ms) << "\n"
           << "  },\n"
           << "  \"joints\": {\n";

        for (int j = 0; j < N_JOINTS; j++) {
            auto& diffs = dq_consec[j];
            std::vector<double> abs_diffs;
            for (double d : diffs) abs_diffs.push_back(std::fabs(d));

            double sd  = stats::stddev(diffs);
            double p99 = stats::percentile(abs_diffs, 99);
            double mx  = stats::vmax(abs_diffs);

            std::cout << "  " << std::left << std::setw(14) << JOINT_NAMES[j]
                      << std::right << std::fixed << std::setprecision(6)
                      << std::setw(12) << sd
                      << std::setw(12) << p99
                      << std::setw(12) << mx << "\n";

            // Per-joint q mean/std for JSON
            std::vector<double> qv;
            for (auto& s : log_) qv.push_back(s.q[j]);

            jf << "    \"" << JOINT_NAMES[j] << "\": {\n"
               << "      \"q_mean_rad\":          " << stats::mean(qv)  << ",\n"
               << "      \"q_std_rad\":           " << stats::stddev(qv) << ",\n"
               << "      \"dq_jitter_std_rad\":   " << sd  << ",\n"
               << "      \"dq_jitter_p99_rad\":   " << p99 << ",\n"
               << "      \"dq_jitter_max_rad\":   " << mx  << "\n"
               << "    }";
            if (j < N_JOINTS - 1) jf << ",";
            jf << "\n";
        }
        jf << "  }\n}\n";
        std::cout << std::string(50, '-') << "\n"
                  << "Results saved -> encoder_jitter_results.json\n";

        // CSV with raw arrival intervals for external plotting
        std::ofstream cf("encoder_arrival_intervals.csv");
        cf << "interval_ms\n";
        for (double v : intervals_ms) cf << v << "\n";
        std::cout << "Interval data saved -> encoder_arrival_intervals.csv\n";

        // Raw q/dq samples
        std::ofstream sf("encoder_jitter_samples.csv");
        sf << "t";
        for (int j = 0; j < N_JOINTS; j++) sf << ",q_" << JOINT_NAMES[j];
        for (int j = 0; j < N_JOINTS; j++) sf << ",dq_" << JOINT_NAMES[j];
        sf << "\n";
        for (auto& s : log_) {
            sf << std::fixed << std::setprecision(9) << s.t;
            for (int j = 0; j < N_JOINTS; j++) sf << "," << s.q[j];
            for (int j = 0; j < N_JOINTS; j++) sf << "," << s.dq[j];
            sf << "\n";
        }
        std::cout << "Raw samples saved -> encoder_jitter_samples.csv\n";
    }

private:
    void on_lowstate(const void* raw) {
        // Elevate DDS callback thread to SCHED_FIFO once on first invocation.
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...) {} });

        double t = rt::mono_now();
        const auto& msg = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw);

        Sample s;
        s.t = t;
        for (int j = 0; j < N_JOINTS; j++) {
            s.q[j]  = msg.motor_state()[j].q();
            s.dq[j] = msg.motor_state()[j].dq();
        }
        std::lock_guard<rt::PIMutex> lk(mtx_);
        log_.push_back(s);
        if (!ready_) { ready_ = true; cv_.notify_all(); }
    }

    double duration_s_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    rt::PIMutex                  mtx_;
    std::condition_variable_any  cv_;
    bool ready_ = false;

    std::vector<Sample> log_;
};

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <iface> [duration_s]\n";
        return 1;
    }
    double duration_s = (argc >= 3) ? std::stod(argv[2]) : 10.0;

    std::cout << std::string(50, '=') << "\n"
              << "  Go2 Encoder Jitter Benchmark\n"
              << "  Interface: " << argv[1] << "  Duration: " << duration_s << " s\n"
              << "  Keep the robot standing still during collection.\n"
              << std::string(50, '=') << "\n"
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

    EncoderJitterBench bench(duration_s);
    bench.init(argv[1]);
    bench.run();
    bench.report();
    return 0;
}
