#pragma once
// RT kernel utilities: memory locking, SCHED_FIFO, PI mutexes, precise sleep.
// Requires: -D_GNU_SOURCE, link with -lrt -lpthread.
//
// Quick-start (run as root, or grant capabilities):
//   sudo setcap cap_sys_nice,cap_ipc_lock+ep <binary>
//
// Usage pattern in main():
//   try { rt::lock_memory(); } catch (const std::exception& e) { warn(e); }
//   rt::prefault_stack();
//
// Usage pattern in DDS callback (sets that thread to SCHED_FIFO once):
//   static std::once_flag rt_flag;
//   std::call_once(rt_flag, []{ try { rt::set_thread_fifo(80); } catch (...){} });

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <string>

namespace rt {

// Lock all current and future memory pages — eliminates page-fault latency spikes.
inline void lock_memory() {
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        throw std::runtime_error(std::string("mlockall: ") + ::strerror(errno));
}

// Touch each page of a stack region to pre-fault it before entering the RT loop.
// Must be called after lock_memory() so the faulted pages stay resident.
inline void prefault_stack() {
    constexpr std::size_t kSize     = 256 * 1024;
    constexpr std::size_t kPageSize = 4096;
    volatile char buf[kSize];
    for (std::size_t i = 0; i < kSize; i += kPageSize)
        buf[i] = 0;
}

// Set the calling thread to SCHED_FIFO at 'priority' (1–98).
// Needs CAP_SYS_NICE or root; throws on failure.
inline void set_thread_fifo(int priority = 80) {
    ::sched_param p{};
    p.sched_priority = priority;
    int r = ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &p);
    if (r != 0)
        throw std::runtime_error(std::string("set_thread_fifo: ") + ::strerror(r));
}

// Pin the calling thread to a specific CPU to reduce cache invalidation.
// Combine with isolcpus= kernel cmdline for full isolation.
inline void pin_to_cpu(int cpu) {
    ::cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    int r = ::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set);
    if (r != 0)
        throw std::runtime_error(std::string("pin_to_cpu: ") + ::strerror(r));
}

// Priority-inheriting mutex — prevents priority inversion between the DDS
// callback thread (high priority) and any low-priority thread that holds mtx_.
//
// Satisfies BasicLockable: works with std::lock_guard and std::unique_lock.
// Use with std::condition_variable_any (not std::condition_variable).
class PIMutex {
public:
    PIMutex() {
        ::pthread_mutexattr_t attr;
        ::pthread_mutexattr_init(&attr);
        ::pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        ::pthread_mutex_init(&m_, &attr);
        ::pthread_mutexattr_destroy(&attr);
    }
    ~PIMutex()      { ::pthread_mutex_destroy(&m_); }
    void lock()     { ::pthread_mutex_lock(&m_); }
    void unlock()   { ::pthread_mutex_unlock(&m_); }
    bool try_lock() { return ::pthread_mutex_trylock(&m_) == 0; }
    ::pthread_mutex_t* native_handle() { return &m_; }
private:
    ::pthread_mutex_t m_ = PTHREAD_MUTEX_INITIALIZER;
    PIMutex(const PIMutex&) = delete;
    PIMutex& operator=(const PIMutex&) = delete;
};

// Get CLOCK_MONOTONIC as a timespec (for use with clock_nanosleep TIMER_ABSTIME).
inline ::timespec mono_now_ts() {
    ::timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

// Get CLOCK_MONOTONIC as seconds (double, nanosecond resolution).
inline double mono_now() {
    ::timespec ts = mono_now_ts();
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Add nanoseconds to a timespec (handles carry and borrow).
inline ::timespec ts_add_ns(::timespec ts, long long ns) {
    ts.tv_nsec += ns;
    while (ts.tv_nsec >= 1'000'000'000LL) { ts.tv_nsec -= 1'000'000'000LL; ++ts.tv_sec; }
    while (ts.tv_nsec <  0LL)             { ts.tv_nsec += 1'000'000'000LL; --ts.tv_sec; }
    return ts;
}

// Absolute-time sleep on CLOCK_MONOTONIC — immune to cumulative drift.
// 'abs_time' is the target wakeup time, not a delta from now.
inline void sleep_until(const ::timespec& abs_time) {
    ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &abs_time, nullptr);
}

} // namespace rt
