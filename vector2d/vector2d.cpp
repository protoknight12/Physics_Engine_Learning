#include "vector2d.h"

vector2d negateVector(const vector2d v) {
    return vector2d{-v.x, -v.y};
}

vector2d addVector(const vector2d v1, const vector2d v2) {
    return vector2d{v1.x + v2.x, v1.y + v2.y};
}

vector2d subtractVector(const vector2d v1, const vector2d v2) {
    return vector2d{v1.x - v2.x, v1.y - v2.y};
}

vector2d scalarMultiplyVector(const vector2d v, const double s) {
    return vector2d{v.x * s, v.y * s};
}

double dotProduct(const vector2d v1, const vector2d v2) {
    return v1.x * v2.x + v1.y * v2.y;
}
