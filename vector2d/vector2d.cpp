#include "vector2d.h"
#include <cmath>
vector2d negateVector(const vector2d v) { return vector2d{-v.x, -v.y}; }
vector2d addVector(const vector2d v1, const vector2d v2) { return vector2d{v1.x + v2.x, v1.y + v2.y}; }
vector2d subtractVector(const vector2d v1, const vector2d v2) { return vector2d{v1.x - v2.x, v1.y - v2.y}; }
vector2d scalarMultiplyVector(const vector2d v, const double s) { return vector2d{v.x * s, v.y * s}; }
double dotProduct(const vector2d v1, const vector2d v2) { return v1.x * v2.x + v1.y * v2.y; }
vector2d crossScalarVector(double s, const vector2d v) { return vector2d{-s * v.y, s * v.x}; }
double crossVectors(vector2d v1, vector2d v2) { return v1.x * v2.y - v1.y * v2.x; }
vector2d normalizeVector(vector2d v) {
    double len = sqrt(v.x * v.x + v.y * v.y);
    if (len == 0) return vector2d{0, 0};
    return vector2d{v.x / len, v.y / len};
}
