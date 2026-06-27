#pragma once

#include "config.h"
#include "collider/collider.h"

extern RigidBody rigidBodies[RIGIDBODY_COUNT];

void initRigidBodies();
void computeForceAndTorque(RigidBody *rigidBody);
void integrateBodies(double deltaTime);
void solveCollisions(int impulseIterations);
void screenBoundaryConstraints(int screenWidth, int screenHeight);
