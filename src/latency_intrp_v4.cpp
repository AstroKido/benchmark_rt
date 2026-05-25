// latency_intrp_v4 — Go2 Pro sub-frame interpolated latency benchmark
// ─────────────────────────────────────────────────────────────────
// Measures round-trip latency with sub-frame (< 2 ms) resolution by linearly
// interpolating the exact crossing time of tau_est through a detection threshold
// between two consecutive LowState frames.
//
// Method (per measurement)
//   1. Publish zero-torque cmd, wait 50 ms to settle.
//   2. Record baseline tau_est (mean of last 20 frames).
//   3. Clear frame buffer; publish step cmd; record t_sent.
//   4. Wait 40 ms for the response to arrive.
//   5. Find the two consecutive frames where tau_est crosses
//      (baseline + tau_step × 0.2); interpolate the exact crossing time.
//   6. latency_ms = (t_cross − t_sent) × 1000
//
// Usage
//   latency_intrp_v4 <iface> [--tau <Nm>] [--n <samples>]
//
//   <iface>        network interface connected to Go2 (e.g. eth0)
//   --tau  <float> torque step amplitude in Nm  (default 0.5)
//   --n    <int>   number of valid samples to collect  (default 200)
//
// Examples
//   latency_intrp_v4 eth0
//   latency_intrp_v4 eth0 --tau 0.4 --n 300
//
// Output
//   latency_v4_results.json   — mean/median/std/p95/p99/min/max
//   latency_v4_samples.csv    — per-sample latency_ms, fraction, frame_interval_ms
//
// Notes
//   • Only samples with 0 < latency_ms < 20 are kept; others are silently dropped.
//   • Robot must be in low-level mode; joint 0 (FR_hip) is used.
//   • frame_interval_ms in the CSV shows the DDS frame width at detection time;
//     values near 2 ms confirm a healthy 500 Hz link.
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
#include <thread>
#include <optional>
#include <limits>
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

static constexpr int   JOINT_IDX      = 0;
static constexpr float KD_SAFE        = 0.5f;
static constexpr float POS_STOP_F     = 2.146e9f;
static constexpr float VEL_STOP_F     = 16000.0f;
static constexpr float THRESHOLD_PCT  = 0.2f;
static constexpr float DEFAULT_TAU    = 0.5f;
static constexpr int   DEFAULT_N      = 200;
static constexpr double SETTLE_TIME   = 0.15;
static constexpr double WAIT_RESPONSE = 0.040;

using Clock = std::chrono::steady_clock;
using FP    = std::chrono::duration<double>;

struct Frame { double t; float tau; };

struct Result {
    double latency_ms;
    double fraction;
    double frame_interval_ms;
};

class InterpLatency {
public:
    InterpLatency(float tau_step, int n_samples)
        : tau_step_(tau_step), n_samples_(n_samples) {
        results_.reserve(n_samples + 32);
    }

    void build_cmd(unitree_go::msg::dds_::LowCmd_& cmd, float tau) {
        cmd.head()[0]    = 0xFE;
        cmd.head()[1]    = 0xEF;
        cmd.level_flag() = 0xFF;
        cmd.gpio()       = 0;
        for (int i = 0; i < 20; i++) {
            cmd.motor_cmd()[i].mode() = 0x01;
            cmd.motor_cmd()[i].q()    = POS_STOP_F;
            cmd.motor_cmd()[i].kp()   = 0.0f;
            cmd.motor_cmd()[i].dq()   = VEL_STOP_F;
            cmd.motor_cmd()[i].kd()   = KD_SAFE;
            cmd.motor_cmd()[i].tau()  = 0.0f;
        }
        cmd.motor_cmd()[JOINT_IDX].tau() = tau;
        cmd.crc() = crc32_core(
            reinterpret_cast<uint32_t*>(&cmd),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    }

    void init(const std::string& iface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);

        build_cmd(cmd_zero_, 0.0f);
        build_cmd(cmd_step_, tau_step_);

        pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
        pub_->InitChannel();

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel([this](const void* m){ on_lowstate(m); }, 10);

        std::cout << "Waiting LowState..." << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(8), [this]{ return first_msg_; });
        if (!first_msg_) { std::cerr << "\nNo LowState.\n"; std::exit(1); }
        std::cout << " OK\n";
    }

    void run() {
        for (int i = 0; i < n_samples_; i++) {
            auto r = measure_once();
            if (r && r->latency_ms > 0.0 && r->latency_ms < 20.0) {
                results_.push_back(*r);
                std::cout << "[" << std::setw(3) << i << "] "
                          << std::fixed << std::setprecision(3) << r->latency_ms << " ms"
                          << "  frac=" << std::setprecision(3) << r->fraction << "\n";
            }
        }
        report();
    }

private:
    void on_lowstate(const void* raw) {
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...) {} });

        double t = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        float tau = reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw)
                        ->motor_state()[JOINT_IDX].tau_est();
        std::lock_guard<rt::PIMutex> lk(mtx_);
        if ((int)buf_.size() >= 200) buf_.pop_front();
        buf_.push_back({t, tau});
        if (!first_msg_) { first_msg_ = true; cv_.notify_all(); }
    }

    double get_baseline() {
        std::lock_guard<rt::PIMutex> lk(mtx_);
        if ((int)buf_.size() < 20) return std::numeric_limits<double>::quiet_NaN();
        double sum = 0.0;
        int n = std::min(20, (int)buf_.size());
        for (int i = (int)buf_.size() - n; i < (int)buf_.size(); i++)
            sum += buf_[i].tau;
        return sum / n;
    }

    std::optional<Result> interpolate_crossing(double t_sent, double baseline) {
        double threshold = baseline + tau_step_ * THRESHOLD_PCT;

        std::vector<Frame> frames;
        {
            std::lock_guard<rt::PIMutex> lk(mtx_);
            for (auto& f : buf_)
                if (f.t > t_sent) frames.push_back(f);
        }

        if ((int)frames.size() < 2) return std::nullopt;

        for (size_t i = 1; i < frames.size(); i++) {
            double t0 = frames[i-1].t, tau0 = frames[i-1].tau;
            double t1 = frames[i  ].t, tau1 = frames[i  ].tau;
            if (tau0 < threshold && tau1 >= threshold) {
                double dtau = tau1 - tau0;
                double frac = (std::fabs(dtau) < 1e-8) ? 0.0 : (threshold - tau0) / dtau;
                double tcross = t0 + frac * (t1 - t0);
                return Result{(tcross - t_sent) * 1000.0, frac, (t1 - t0) * 1000.0};
            }
        }
        return std::nullopt;
    }

    std::optional<Result> measure_once() {
        pub_->Write(cmd_zero_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        double baseline = get_baseline();
        if (std::isnan(baseline)) return std::nullopt;

        { std::lock_guard<rt::PIMutex> lk(mtx_); buf_.clear(); }

        pub_->Write(cmd_step_);
        double t_sent = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();

        std::this_thread::sleep_for(
            std::chrono::duration<double>(WAIT_RESPONSE));

        auto result = interpolate_crossing(t_sent, baseline);

        pub_->Write(cmd_zero_);
        std::this_thread::sleep_for(
            std::chrono::duration<double>(SETTLE_TIME));

        return result;
    }

    void report() {
        std::vector<double> arr;
        for (auto& r : results_) arr.push_back(r.latency_ms);
        if (arr.empty()) { std::cout << "No valid samples.\n"; return; }

        std::cout << "\n=== Interpolated Latency ===\n"
                  << "n:      " << arr.size() << "\n"
                  << std::fixed << std::setprecision(4)
                  << "mean:   " << stats::mean(arr)           << " ms\n"
                  << "median: " << stats::median(arr)         << " ms\n"
                  << "std:    " << stats::stddev(arr)         << " ms\n"
                  << "p95:    " << stats::percentile(arr, 95) << " ms\n"
                  << "p99:    " << stats::percentile(arr, 99) << " ms\n"
                  << "min:    " << stats::vmin(arr)           << " ms\n"
                  << "max:    " << stats::vmax(arr)           << " ms\n";

        std::ofstream jf("latency_v4_results.json");
        jf << std::fixed << std::setprecision(6)
           << "{\n"
           << "  \"n\":      " << arr.size()                  << ",\n"
           << "  \"mean\":   " << stats::mean(arr)            << ",\n"
           << "  \"median\": " << stats::median(arr)          << ",\n"
           << "  \"std\":    " << stats::stddev(arr)          << ",\n"
           << "  \"p95\":    " << stats::percentile(arr, 95)  << ",\n"
           << "  \"p99\":    " << stats::percentile(arr, 99)  << ",\n"
           << "  \"min\":    " << stats::vmin(arr)            << ",\n"
           << "  \"max\":    " << stats::vmax(arr)            << "\n"
           << "}\n";

        std::ofstream cf("latency_v4_samples.csv");
        cf << "latency_ms,fraction,frame_interval_ms\n";
        for (auto& r : results_)
            cf << r.latency_ms << "," << r.fraction << "," << r.frame_interval_ms << "\n";

        std::cout << "Saved: latency_v4_results.json\n"
                  << "Saved: latency_v4_samples.csv\n";
    }

    float tau_step_;
    int   n_samples_;

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>    pub_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    unitree_go::msg::dds_::LowCmd_ cmd_zero_{};
    unitree_go::msg::dds_::LowCmd_ cmd_step_{};

    rt::PIMutex                  mtx_;
    std::condition_variable_any  cv_;
    bool first_msg_ = false;

    std::deque<Frame> buf_;
    std::vector<Result> results_;
};

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <iface> [--tau <val>] [--n <val>]\n";
        return 1;
    }
    float tau   = DEFAULT_TAU;
    int   n_smp = DEFAULT_N;
    for (int i = 2; i < argc - 1; i++) {
        if (std::string(argv[i]) == "--tau") tau   = std::stof(argv[i+1]);
        if (std::string(argv[i]) == "--n")   n_smp = std::stoi(argv[i+1]);
    }

    try {
        rt::lock_memory();
        std::cout << "Memory locked (MCL_CURRENT | MCL_FUTURE)\n";
    } catch (const std::exception& e) {
        std::cerr << "Warning: " << e.what()
                  << "\n  Run with sudo or: sudo setcap cap_ipc_lock+ep " << argv[0] << "\n";
    }
    rt::prefault_stack();

    InterpLatency bench(tau, n_smp);
    bench.init(argv[1]);
    bench.run();
    return 0;
}
