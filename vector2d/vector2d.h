#pragma once

struct vector2d {
    double x, y;
};

vector2d negateVector(vector2d vector);

vector2d addVector(vector2d firstVector, vector2d secondVector);

vector2d subtractVector(vector2d firstVector, vector2d secondVector);

vector2d scalarMultiplyVector(vector2d vector, double scalar);

double dotProduct(vector2d firstVector, vector2d secondVector);

vector2d crossScalarVector(double scalar, vector2d vector);

double crossVectors(vector2d firstVector, vector2d secondVector);

vector2d normalizeVector(vector2d vector);
