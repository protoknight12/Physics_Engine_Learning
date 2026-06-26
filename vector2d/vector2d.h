#pragma once
struct vector2d {
    double x, y;
};

vector2d negateVector(vector2d v);

vector2d addVector(vector2d v1, vector2d v2);

vector2d subtractVector(vector2d v1, vector2d v2);

vector2d scalarMultiplyVector(vector2d v, double s);

double dotProduct(vector2d v1, vector2d v2);

vector2d crossScalarVector(double s, vector2d v);

double crossVectors(vector2d v1, vector2d v2);

vector2d normalizeVector(vector2d v);