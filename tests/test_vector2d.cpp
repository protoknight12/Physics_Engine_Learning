#include "test_helpers.h"
#include "vector2d/vector2d.h"

void registerVector2dTests(int &failureCount) {
    failureCount += runTest("dotProduct orthogonal",
                            nearlyEqual(dotProduct({1, 0}, {0, 1}), 0.0));
    failureCount += runTest("addVector sums components",
                            nearlyEqual(addVector({1, 2}, {3, 4}).x, 4.0) &&
                            nearlyEqual(addVector({1, 2}, {3, 4}).y, 6.0));
    failureCount += runTest("normalizeVector unit length",
                            nearlyEqual(dotProduct(normalizeVector({3, 4}), normalizeVector({3, 4})), 1.0));
    failureCount += runTest("normalizeVector zero input",
                            nearlyEqual(normalizeVector({0, 0}).x, 0.0) &&
                            nearlyEqual(normalizeVector({0, 0}).y, 0.0));
    failureCount += runTest("crossVectors sign",
                            nearlyEqual(crossVectors({1, 0}, {0, 1}), 1.0));
}
