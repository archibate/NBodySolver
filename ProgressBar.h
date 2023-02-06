#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>

class ProgressBar {
    std::chrono::high_resolution_clock::time_point last;
public:
    ProgressBar() : last(std::chrono::high_resolution_clock::now()) {
    }

    ProgressBar(ScopeProfiler &&) = delete;

    void operator()(double rate, std::ostream &out = std::cout) const {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - last).count();
        out << '\r';
        out << "elapsed" << std::setw(4) << (int)elapsed << "s ";
        if (rate <= 0.01)
            out << "remain ---s ";
        else
            out << "remain" << std::setw(4) << (int)(elapsed * (1.0 / rate - 1.0)) << "s ";
        out << '[';
        for (int minI = (int)(rate * 20.0 - 0.5), i = 0; i < 20; i++) {
            out << " >="[(i < minI) + (i <= minI)];
        }
        out << "] ";
        out << '(' << (int)(rate * 100.0) << '%' << ')';
        out << std::flush;
    }

    void operator()(size_t i, size_t maxI, std::ostream &out = std::cout) const {
        operator()((double)i / (double)maxI, out);
    }
};
