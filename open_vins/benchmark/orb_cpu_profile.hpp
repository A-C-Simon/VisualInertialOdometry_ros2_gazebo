#pragma once

// Diagnostic build only. Inclusive thread CPU excludes sleeping and work on
// other threads. Parent and child scopes overlap and must not be added.
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <time.h>
#include <vector>

namespace orb_cpu_profile {

struct Counter {
    explicit Counter(const char* name) : label(name) {}
    const char* label;
    std::atomic<std::uint64_t> calls{0};
    std::atomic<std::uint64_t> cpu_ns{0};
};

struct Registry {
    Registry() {
        const char* path = std::getenv("ORB_PROFILE_OUTPUT");
        if (path) output = path;
    }
    ~Registry() {
        if (output.empty()) return;
        std::lock_guard<std::mutex> lock(mutex);
        std::ofstream file(output);
        file << "stage,calls,inclusive_thread_cpu_seconds\n";
        file << std::fixed << std::setprecision(9);
        for (const Counter* counter : counters)
            file << counter->label << ',' << counter->calls.load() << ','
                 << counter->cpu_ns.load() * 1e-9 << '\n';
    }
    std::string output;
    std::mutex mutex;
    // Counters live until process exit so static scope registrations remain
    // valid regardless of translation-unit destructor ordering.
    std::vector<Counter*> counters;
};

inline Registry& registry() {
    static Registry value;
    return value;
}

inline Counter* register_counter(const char* label) {
    Registry& state = registry();
    std::lock_guard<std::mutex> lock(state.mutex);
    Counter* counter = new Counter(label);
    state.counters.push_back(counter);
    return counter;
}

class Scope {
public:
    explicit Scope(Counter* counter) : counter_(counter) {
        valid_ = !registry().output.empty() &&
                 clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start_) == 0;
    }
    ~Scope() {
        timespec end;
        if (!valid_ || clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end) != 0) return;
        const std::int64_t elapsed = (end.tv_sec - start_.tv_sec) * 1000000000LL
                                    + end.tv_nsec - start_.tv_nsec;
        if (elapsed < 0) return;
        counter_->cpu_ns.fetch_add(static_cast<std::uint64_t>(elapsed),
                                   std::memory_order_relaxed);
        counter_->calls.fetch_add(1, std::memory_order_relaxed);
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
private:
    Counter* counter_;
    timespec start_{};
    bool valid_{false};
};

} // namespace orb_cpu_profile

#define ORB_CPU_JOIN_IMPL(a, b) a##b
#define ORB_CPU_JOIN(a, b) ORB_CPU_JOIN_IMPL(a, b)
#define ORB_CPU_SCOPE(label) \
    static auto* ORB_CPU_JOIN(orb_cpu_counter_, __LINE__) = \
        orb_cpu_profile::register_counter(label); \
    orb_cpu_profile::Scope ORB_CPU_JOIN(orb_cpu_scope_, __LINE__)( \
        ORB_CPU_JOIN(orb_cpu_counter_, __LINE__))
