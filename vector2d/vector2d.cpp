#include "vector2d.h"
#include <cmath>

vector2d negateVector(const vector2d vector) { return vector2d{-vector.x, -vector.y}; }

vector2d addVector(const vector2d firstVector, const vector2d secondVector) {
    return vector2d{firstVector.x + secondVector.x, firstVector.y + secondVector.y};
}

vector2d subtractVector(const vector2d firstVector, const vector2d secondVector) {
    return vector2d{firstVector.x - secondVector.x, firstVector.y - secondVector.y};
}

vector2d scalarMultiplyVector(const vector2d vector, const double scalar) {
    return vector2d{vector.x * scalar, vector.y * scalar};
}

double dotProduct(const vector2d firstVector, const vector2d secondVector) {
    return firstVector.x * secondVector.x + firstVector.y * secondVector.y;
}

vector2d crossScalarVector(const double scalar, const vector2d vector) {
    return vector2d{-scalar * vector.y, scalar * vector.x};
}

double crossVectors(const vector2d firstVector, const vector2d secondVector) {
    return firstVector.x * secondVector.y - firstVector.y * secondVector.x;
}

vector2d normalizeVector(vector2d vector) {
    const double length = sqrt(vector.x * vector.x + vector.y * vector.y);
    if (length == 0) {
        return vector2d{0, 0};
    }
    return vector2d{vector.x / length, vector.y / length};
}
