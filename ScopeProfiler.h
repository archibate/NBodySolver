#pragma once

#include <chrono>
#include <iostream>

class ScopeProfiler {
    std::chrono::high_resolution_clock::time_point last;
    const char *name;

public:
    ScopeProfiler(const char *name) : last(std::chrono::high_resolution_clock::now()), name(name) {
    }

    ScopeProfiler(ScopeProfiler &&) = delete;

    ~ScopeProfiler() {
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - last).count();
        std::cout << name << " takes " << dt << "s" << std::endl;
    }
};

#define _DefScopeProfiler_UNIQUEID(x, y) __ScopeProfiler_##x##_##y
#define DefScopeProfiler ScopeProfiler _DefScopeProfiler_UNIQUEID(__func__, __LINE__)(__func__);
