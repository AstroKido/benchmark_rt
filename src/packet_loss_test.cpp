// packet_loss_test — Go2 Pro LowState packet loss benchmark
// ─────────────────────────────────────────────────────────────────
// Publishes a keep-alive LowCmd at 200 Hz while subscribing to rt/lowstate
// and timestamping every arriving message.  Two independent methods detect
// packet loss:
//
//   Wall-clock gap analysis
//     Inter-arrival gaps > 1.5 × 2 ms (3 ms) are counted as dropout events.
//
//   Tick-counter gap analysis
//     The uint32 MCU tick in LowState increments ~2 per 2 ms message.
//     Jumps > 3× the normal step indicate missing messages; the estimated
//     missing count is accumulated across the run.
//
// Usage
//   packet_loss_test <iface> [duration_s]
//
//   <iface>       network interface connected to Go2 (e.g. eth0)
//   duration_s    test duration in seconds  (default 1800 = 30 min)
//
// Examples
//   packet_loss_test eth0 60      # quick 60 s smoke test
//   packet_loss_test eth0         # full 30 min production test
//
// Output
//   packet_loss_results.json    — full statistics, verdict, drop event list
//   packet_loss_intervals.csv   — per-gap t_s / interval_ms / dropout flag
//
// Target thresholds
//   < 0.1 %  — acceptable for hard real-time control
//   0.1–0.5% — marginal; controller should handle occasional dropouts
//   > 0.5 %  — critical; architectural mitigation required
//
// Notes
//   • Robot can be in any state; no torque is commanded (keep-alive only).
//   • Run on an isolated Ethernet link for representative results; shared
//     network traffic inflates the dropout count.
// ─────────────────────────────────────────────────────────────────
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdint>
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

static constexpr int    CMD_HZ          = 200;
static constexpr int    LOWSTATE_HZ     = 500;
static constexpr double EXPECTED_PERIOD = 1.0 / LOWSTATE_HZ;      // 2 ms
static constexpr double DROP_THRESH     = 1.5 * EXPECTED_PERIOD;   // 3 ms
static constexpr double DEFAULT_DUR     = 30 * 60.0;               // 30 min
static constexpr int    TICK_STEP       = 2;

static constexpr float POS_STOP_F = 2.146e9f;
static constexpr float VEL_STOP_F = 16000.0f;

using Clock = std::chrono::steady_clock;

struct Arrival { double t; uint32_t tick; };

class PacketLossBench {
public:
    explicit PacketLossBench(double duration_s) : duration_s_(duration_s) {
        log_.reserve(static_cast<std::size_t>(duration_s * LOWSTATE_HZ * 1.1));
    }

    void init(const std::string& iface) {
        unitree::robot::ChannelFactory::Instance()->Init(0, iface);

        make_idle_cmd();

        pub_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
        pub_->InitChannel();

        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        sub_->InitChannel([this](const void* m){ on_lowstate(m); }, 10);

        std::cout << "Waiting for first LowState..." << std::flush;
        std::unique_lock<rt::PIMutex> lk(mtx_);
        cv_.wait_for(lk, std::chrono::seconds(5), [this]{ return started_; });
        if (!started_) throw std::runtime_error("No LowState in 5 s");
        std::cout << " OK\n";
    }

    void run() {
        // Publish thread
        pub_thread_ = std::thread([this]{ publish_loop(); });

        int expected_total = (int)(duration_s_ * LOWSTATE_HZ);
        std::cout << "Running " << (int)duration_s_ << " s at " << CMD_HZ << " Hz LowCmd..."
                  << "  (expected ~" << expected_total << " LowState msgs)\n";

        double interval = std::max(10.0, duration_s_ / 10.0);
        double elapsed  = 0.0;
        while (elapsed < duration_s_) {
            double sleep_t = std::min(interval, duration_s_ - elapsed);
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_t));
            elapsed += sleep_t;
            size_t n;
            { std::lock_guard<rt::PIMutex> lk(mtx_); n = log_.size(); }
            if (n > 0)
                std::cout << "  " << std::setw(6) << (int)elapsed
                          << " s — " << n << " msgs received\n";
        }

        done_.store(true);
        if (pub_thread_.joinable()) pub_thread_.join();

        std::vector<Arrival> local;
        { std::lock_guard<rt::PIMutex> lk(mtx_); local = log_; }
        std::cout << "Collected " << local.size() << " LowState messages.\n";
        report(local);
    }

private:
    void make_idle_cmd() {
        cmd_.head()[0]    = 0xFE;
        cmd_.head()[1]    = 0xEF;
        cmd_.level_flag() = 0xFF;
        cmd_.gpio()       = 0;
        for (int i = 0; i < 20; i++) {
            cmd_.motor_cmd()[i].mode() = 0x01;
            cmd_.motor_cmd()[i].q()    = POS_STOP_F;
            cmd_.motor_cmd()[i].kp()   = 0.0f;
            cmd_.motor_cmd()[i].dq()   = VEL_STOP_F;
            cmd_.motor_cmd()[i].kd()   = 0.0f;
            cmd_.motor_cmd()[i].tau()  = 0.0f;
        }
        cmd_.crc() = crc32_core(
            reinterpret_cast<uint32_t*>(&cmd_),
            (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    }

    void publish_loop() {
        // Publish thread runs SCHED_FIFO at lower priority than the DDS callback.
        try { rt::set_thread_fifo(75); } catch (...) {}

        // Absolute-time sleep: immune to cumulative drift unlike sleep_for + delta.
        constexpr long period_ns = 1'000'000'000L / CMD_HZ;
        timespec next = rt::mono_now_ts();
        while (!done_.load()) {
            pub_->Write(cmd_);
            next = rt::ts_add_ns(next, period_ns);
            rt::sleep_until(next);
        }
    }

    void on_lowstate(const void* raw) {
        static std::once_flag rt_flag;
        std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...) {} });

        if (done_.load()) return;
        double t = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
        const auto& msg = *reinterpret_cast<const unitree_go::msg::dds_::LowState_*>(raw);
        std::lock_guard<rt::PIMutex> lk(mtx_);
        log_.push_back({t, msg.tick()});
        if (!started_) { started_ = true; cv_.notify_all(); }
    }

    void report(const std::vector<Arrival>& log) {
        size_t n_rx = log.size();
        if (n_rx < 10) { std::cout << "Too few samples.\n"; return; }

        double total_time = log.back().t - log.front().t;

        // Wall-clock intervals
        std::vector<double> intervals_ms;
        for (size_t i = 1; i < n_rx; i++)
            intervals_ms.push_back((log[i].t - log[i-1].t) * 1e3);

        // Wall-clock dropouts
        double drop_thresh_ms = DROP_THRESH * 1e3;
        std::vector<size_t> drop_idx;
        for (size_t i = 0; i < intervals_ms.size(); i++)
            if (intervals_ms[i] > drop_thresh_ms) drop_idx.push_back(i);

        int n_wc_drops = (int)drop_idx.size();
        double drop_rate_wc = (double)n_wc_drops / n_rx * 100.0;
        double worst_gap = 0.0;
        for (size_t i : drop_idx) worst_gap = std::max(worst_gap, intervals_ms[i]);

        // Tick-based gap analysis (uint32 wraparound safe)
        std::vector<int64_t> tick_diffs;
        for (size_t i = 1; i < n_rx; i++) {
            int64_t d = (int64_t)log[i].tick - (int64_t)log[i-1].tick;
            if (d < 0) d += (int64_t)0x100000000LL; // wraparound
            tick_diffs.push_back(d);
        }
        int tick_thresh = TICK_STEP * 3;
        int n_tick_drops = 0, tick_missing = 0;
        for (int64_t d : tick_diffs) {
            if (d > tick_thresh) {
                n_tick_drops++;
                tick_missing += (int)std::max((int64_t)0, d / TICK_STEP - 1);
            }
        }

        int n_expected = (int)std::round(total_time * LOWSTATE_HZ);
        double drop_rate_tk = (n_expected > 0) ? (double)tick_missing / n_expected * 100.0 : 0.0;

        auto verdict = (drop_rate_wc < 0.1) ? "ACCEPTABLE (<0.1%)" :
                       (drop_rate_wc < 0.5) ? "MARGINAL (0.1-0.5%)" :
                                              "CRITICAL (>0.5%) — control loop must handle dropouts";

        std::cout << "\n" << std::string(55, '=') << "\n"
                  << "  Packet Loss Report\n"
                  << std::string(55, '-') << "\n"
                  << "  Total time             : " << std::fixed << std::setprecision(1) << total_time << " s\n"
                  << "  LowState msgs received : " << n_rx << "\n"
                  << "  Expected (@ " << LOWSTATE_HZ << " Hz) : " << n_expected << "\n"
                  << "\n  - Wall-clock gap analysis (threshold "
                  << std::setprecision(1) << drop_thresh_ms << " ms) -\n"
                  << "  Dropout events         : " << n_wc_drops << "\n"
                  << "  Drop rate              : " << std::setprecision(4) << drop_rate_wc << " %\n";
        if (n_wc_drops)
            std::cout << "  Worst gap              : " << worst_gap << " ms\n";
        std::cout << "\n  - Tick-counter gap analysis -\n"
                  << "  Tick-gap events        : " << n_tick_drops << "\n"
                  << "  Estimated missing msgs : " << tick_missing << "\n"
                  << "  Estimated loss rate    : " << drop_rate_tk << " %\n"
                  << "\n  - Interval stats (ms) -\n"
                  << "  min    " << stats::vmin(intervals_ms)           << "\n"
                  << "  median " << stats::median(intervals_ms)         << "\n"
                  << "  mean   " << stats::mean(intervals_ms)
                  << " +- "      << stats::stddev(intervals_ms)         << "\n"
                  << "  p99    " << stats::percentile(intervals_ms, 99) << "\n"
                  << "  max    " << stats::vmax(intervals_ms)           << "\n"
                  << "\n  Verdict: " << verdict << "\n"
                  << std::string(55, '=') << "\n";

        // JSON
        std::ofstream jf("packet_loss_results.json");
        jf << std::fixed << std::setprecision(6)
           << "{\n"
           << "  \"duration_s\":             " << total_time << ",\n"
           << "  \"cmd_hz\":                 " << CMD_HZ << ",\n"
           << "  \"lowstate_hz_nominal\":    " << LOWSTATE_HZ << ",\n"
           << "  \"n_received\":             " << n_rx << ",\n"
           << "  \"n_expected\":             " << n_expected << ",\n"
           << "  \"interval_ms\": {\n"
           << "    \"min\":    " << stats::vmin(intervals_ms)           << ",\n"
           << "    \"median\": " << stats::median(intervals_ms)         << ",\n"
           << "    \"mean\":   " << stats::mean(intervals_ms)           << ",\n"
           << "    \"std\":    " << stats::stddev(intervals_ms)         << ",\n"
           << "    \"p99\":    " << stats::percentile(intervals_ms, 99) << ",\n"
           << "    \"max\":    " << stats::vmax(intervals_ms)           << "\n"
           << "  },\n"
           << "  \"wall_clock_drops\": {\n"
           << "    \"threshold_ms\":  " << drop_thresh_ms << ",\n"
           << "    \"n_events\":      " << n_wc_drops << ",\n"
           << "    \"drop_rate_pct\": " << drop_rate_wc << ",\n"
           << "    \"worst_gap_ms\":  " << worst_gap << "\n"
           << "  },\n"
           << "  \"tick_drops\": {\n"
           << "    \"n_events\":              " << n_tick_drops << ",\n"
           << "    \"estimated_missing\":      " << tick_missing << ",\n"
           << "    \"estimated_loss_rate_pct\":" << drop_rate_tk << "\n"
           << "  },\n"
           << "  \"verdict\": \"" << verdict << "\"\n"
           << "}\n";

        // Interval timeline CSV for plotting
        std::ofstream cf("packet_loss_intervals.csv");
        cf << "t_s,interval_ms,dropout\n";
        for (size_t i = 0; i < intervals_ms.size(); i++) {
            bool is_drop = (intervals_ms[i] > drop_thresh_ms);
            cf << std::fixed << std::setprecision(6)
               << (log[i+1].t - log.front().t) << ","
               << intervals_ms[i] << ","
               << (is_drop ? 1 : 0) << "\n";
        }

        std::cout << "Saved: packet_loss_results.json\n"
                  << "Saved: packet_loss_intervals.csv\n";
    }

    double duration_s_;
    std::atomic<bool> done_{false};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>    pub_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> sub_;

    unitree_go::msg::dds_::LowCmd_ cmd_{};

    rt::PIMutex                  mtx_;
    std::condition_variable_any  cv_;
    bool started_ = false;

    std::vector<Arrival> log_;
    std::thread pub_thread_;
};

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <iface> [duration_s]\n";
        return 1;
    }
    double duration_s = (argc >= 3) ? std::stod(argv[2]) : DEFAULT_DUR;

    std::cout << std::string(55, '=') << "\n"
              << "  Go2 Packet Loss Benchmark\n"
              << "  Interface : " << argv[1] << "\n"
              << "  Duration  : " << (int)duration_s << " s  ("
              << std::fixed << std::setprecision(1) << duration_s/60.0 << " min)\n"
              << "  LowCmd Hz : " << CMD_HZ << "\n"
              << "  Drop thresh: " << DROP_THRESH*1e3 << " ms\n"
              << std::string(55, '=') << "\n"
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

    PacketLossBench bench(duration_s);
    bench.init(argv[1]);
    bench.run();
    return 0;
}
