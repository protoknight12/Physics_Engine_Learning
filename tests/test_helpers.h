#pragma once
#include <cmath>
#include <iostream>
#include <string>

inline int runTest(const std::string &testName, const bool passed) {
    if (passed) {
        std::cout << "[PASS] " << testName << std::endl;
        return 0;
    }

    std::cerr << "[FAIL] " << testName << std::endl;
    return 1;
}

inline bool nearlyEqual(const double firstValue, const double secondValue, const double epsilon = 1e-6) {
    return std::abs(firstValue - secondValue) <= epsilon;
}

void registerVector2dTests(int &failureCount);
void registerCollisionTests(int &failureCount);
