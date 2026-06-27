#include "test_helpers.h"

int main() {
    int failureCount = 0;
    registerVector2dTests(failureCount);
    registerCollisionTests(failureCount);

    if (failureCount == 0) {
        std::cout << "All tests passed." << std::endl;
        return 0;
    }

    std::cerr << failureCount << " test(s) failed." << std::endl;
    return 1;
}
