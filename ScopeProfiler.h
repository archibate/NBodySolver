#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>

class ScopeProfiler {
    std::chrono::high_resolution_clock::time_point last;
    const char *name;

public:
    explicit ScopeProfiler(const char *name) : last(std::chrono::high_resolution_clock::now()), name(name) {
    }

    ScopeProfiler(ScopeProfiler &&) = delete;

    ~ScopeProfiler() {
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - last).count();
        std::cout << name << ": " << dt << "s" << std::endl;
    }
};

#define _DefScopeProfiler_UNIQUEID(x, y) __ScopeProfiler_##x##_##y
#define DefScopeProfiler ScopeProfiler _DefScopeProfiler_UNIQUEID(__func__, __LINE__)(__func__);

class ProgressPredicter {
    std::chrono::high_resolution_clock::time_point last;
public:
    ProgressPredicter() : last(std::chrono::high_resolution_clock::now()) {
    }

    ProgressPredicter(ScopeProfiler &&) = delete;

    void operator()(double rate, std::ostream &out = std::cout) const {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last).count();
        out << '\r';
        out << "elapsed " << std::setw(4) << (int)elapsed << "s ";
        if (rate <= 0.01)
            out << "remain ????? ";
        else
            out << "remain " << std::setw(4) << (int)(elapsed * (1.0 / rate - 1.0)) << "s ";
        out << '(' << (int)(rate * 100.0) << '%' << ')';
        out << std::flush;
    }

    void operator()(size_t i, size_t maxI, std::ostream &out = std::cout) const {
        operator()((double)i / (double)maxI, out);
    }
};
