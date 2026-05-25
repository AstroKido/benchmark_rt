// pulse_sweep — Go2 Pro torque pulse width sweep benchmark
// ─────────────────────────────────────────────────────────────────
// Sends torque pulses of varying width to joint 0 (FR_hip) and measures
// the detection latency and peak tau_est response for each width.
// Repeating each width N_REPEAT (20) times gives hit-rate and median latency
// as a function of pulse duration, characterising the minimum detectable
// pulse width and the transport's step-response fidelity.
//
// Pulse widths tested (ms): 2, 5, 10, 20, 40
//
// Method (per trial)
//   1. Publish zero-torque cmd, wait 80 ms to settle.
//   2. Record baseline (mean of last 30 frames).
//   3. Clear frame buffer; publish step cmd for width_ms; then publish zero.
//   4. Wait 60 ms for tail response.
//   5. Scan captured frames: first frame where tau_est ≥ (baseline + tau × 0.1)
//      is the detection event; dt from t_sent is the latency.
//
// Usage
//   pulse_sweep <iface>
//
//   <iface>   network interface connected to Go2 (e.g. eth0)
//
// Example
//   pulse_sweep eth0
//
// Output
//   pulse_summary.json   — per-width hit_rate, median_latency_ms, median_peak
//   pulse_traces.csv     — raw (width_ms, rep, t_ms, tau) trace points
//
// Notes
//   • Robot must be in low-level mode; joint 0 (FR_hip) is used.
//   • Keep the robot secured — 20 repetitions × 5 widths = 100 pulses total.
//   • A hit_rate < 1.0 for widths ≥ 10 ms suggests transport instability.
// ─────────────────────────────────────────────────────────────────
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <cmath>
#include <optional>
#include <iomanip>
#include <algorithm>
#include <numeric>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#include "crc32.hpp"
#include "rt_utils.hpp"
#include "stats.hpp"

using namespace unitree::robot;
using Clock = std::chrono::steady_clock;

static constexpr int   JOINT_IDX      = 0;
static constexpr float KD_SAFE        = 0.5f;
static constexpr float POS_STOP_F     = 2.146e9f;
static constexpr float VEL_STOP_F     = 16000.0f;

static constexpr float TAU            = 0.5f;
static constexpr int   N_REPEAT       = 20;
static constexpr float THRESHOLD_FRAC = 0.10f;

static const std::vector<int> PULSE_WIDTHS_MS = {2, 5, 10, 20, 40};

struct TracePoint { double dt_ms; float tau; };

struct TraceRecord {
    int   width_ms;
    int   rep;
    std::vector<TracePoint> points;
};

struct RunResult {
    bool   detected;
    double latency_ms;
    float  peak;
};

struct SweepSummary {
    double hit_rate;
    std::optional<double> median_latency_ms;
    std::optional<double> median_peak;
};

class PulseSweep {
public:
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
        build_cmd(cmd_step_, TAU);

        pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
        pub_->InitChannel();

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel([this](const void* m){ on_lowstate(m); }, 10);

        std::cout << "Waiting for LowState..." << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(8), [this]{ return first_; });
        if (!first_) { std::cerr << "\nNo LowState received.\n"; std::exit(1); }
        std::cout << " OK\n";
    }

    void sweep() {
        std::cout << "\nRunning sweep...\n\n";
        for (int width : PULSE_WIDTHS_MS) {
            int hits = 0;
            std::vector<double> lats, peaks;
            std::cout << width << " ms ..." << std::flush;

            for (int rep = 0; rep < N_REPEAT; rep++) {
                auto r = run_once(width, rep);
                if (r && r->detected) {
                    hits++;
                    lats.push_back(r->latency_ms);
                    peaks.push_back(r->peak);
                }
            }

            SweepSummary s;
            s.hit_rate = (double)hits / N_REPEAT;
            if (!lats.empty()) {
                s.median_latency_ms = stats::median(lats);
                s.median_peak       = stats::median(peaks);
            }
            summary_[width] = s;

            std::cout << " hit=" << std::fixed << std::setprecision(2) << s.hit_rate;
            if (s.median_latency_ms)
                std::cout << "  lat=" << *s.median_latency_ms << " ms";
            std::cout << "\n";
        }
        save();
    }

private:
    void on_lowstate(const void* raw) {
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...) {} });

        double t = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        float tau = reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw)
                        ->motor_state()[JOINT_IDX].tau_est();
        std::lock_guard<rt::PIMutex> lk(mtx_);
        if ((int)frames_.size() >= 1000) frames_.pop_front();
        frames_.push_back({t, tau});
        if (!first_) { first_ = true; cv_.notify_all(); }
    }

    float baseline() {
        std::lock_guard<rt::PIMutex> lk(mtx_);
        int n = std::min(30, (int)frames_.size());
        if (n == 0) return 0.0f;
        double sum = 0.0;
        for (int i = (int)frames_.size() - n; i < (int)frames_.size(); i++)
            sum += frames_[i].tau;
        return (float)(sum / n);
    }

    struct TFrame { double t; float tau; };

    std::optional<RunResult> run_once(int width_ms, int rep) {
        pub_->Write(cmd_zero_);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        float base = baseline();
        float threshold = base + TAU * THRESHOLD_FRAC;

        { std::lock_guard<rt::PIMutex> lk(mtx_); frames_.clear(); }

        double t_sent = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();

        // pulse for width_ms
        double t0 = t_sent;
        while ((std::chrono::duration<double>(Clock::now().time_since_epoch()).count() - t0) * 1000.0
               < width_ms) {
            pub_->Write(cmd_step_);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        pub_->Write(cmd_zero_);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));

        std::vector<TFrame> data;
        {
            std::lock_guard<rt::PIMutex> lk(mtx_);
            for (auto& f : frames_) data.push_back({f.t, f.tau});
        }

        if (data.size() < 2) return std::nullopt;

        TraceRecord tr;
        tr.width_ms = width_ms;
        tr.rep = rep;

        bool detected = false;
        double latency_ms = 0.0;
        float peak = 0.0f;

        for (auto& [t, tau] : data) {
            double dt_ms = (t - t_sent) * 1000.0;
            tr.points.push_back({dt_ms, tau});
            if (!detected && tau >= threshold) {
                detected   = true;
                latency_ms = dt_ms;
            }
            peak = std::max(peak, tau);
        }
        traces_.push_back(std::move(tr));

        return RunResult{detected, latency_ms, peak};
    }

    void save() {
        // summary JSON
        std::ofstream jf("pulse_summary.json");
        jf << "{\n";
        bool first_entry = true;
        for (auto& [w, s] : summary_) {
            if (!first_entry) jf << ",\n";
            first_entry = false;
            jf << "  \"" << w << "\": {\n"
               << "    \"hit_rate\": " << std::fixed << std::setprecision(4) << s.hit_rate << ",\n";
            if (s.median_latency_ms)
                jf << "    \"median_latency_ms\": " << *s.median_latency_ms << ",\n";
            else
                jf << "    \"median_latency_ms\": null,\n";
            if (s.median_peak)
                jf << "    \"median_peak\": " << *s.median_peak << "\n";
            else
                jf << "    \"median_peak\": null\n";
            jf << "  }";
        }
        jf << "\n}\n";

        // traces CSV
        std::ofstream cf("pulse_traces.csv");
        cf << "width_ms,rep,t_ms,tau\n";
        for (auto& tr : traces_) {
            for (auto& pt : tr.points)
                cf << tr.width_ms << "," << tr.rep << ","
                   << std::fixed << std::setprecision(6) << pt.dt_ms << ","
                   << pt.tau << "\n";
        }

        std::cout << "\nSaved:\n"
                  << "  pulse_summary.json\n"
                  << "  pulse_traces.csv\n";
    }

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>    pub_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    unitree_go::msg::dds_::LowCmd_ cmd_zero_{};
    unitree_go::msg::dds_::LowCmd_ cmd_step_{};

    rt::PIMutex                  mtx_;
    std::condition_variable_any  cv_;
    bool first_ = false;

    struct BufFrame { double t; float tau; };
    std::deque<BufFrame> frames_;

    std::vector<TraceRecord> traces_;
    std::map<int, SweepSummary> summary_;
};

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <iface>\n";
        return 1;
    }
    try {
        rt::lock_memory();
        std::cout << "Memory locked (MCL_CURRENT | MCL_FUTURE)\n";
    } catch (const std::exception& e) {
        std::cerr << "Warning: " << e.what()
                  << "\n  Run with sudo or: sudo setcap cap_ipc_lock+ep " << argv[0] << "\n";
    }
    rt::prefault_stack();

    PulseSweep ps;
    ps.init(argv[1]);
    ps.sweep();
    return 0;
}
