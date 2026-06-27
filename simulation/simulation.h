#pragma once

#include "config.h"
#include "collider/collider.h"

extern RigidBody rigidBodies[RIGIDBODY_COUNT];

void initRigidBodies();
void computeForceAndTorque(RigidBody *rigidBody);
void integrateBodies(double deltaTime);
void solveCollisions(int impulseIterations);
void screenBoundaryConstraints(int screenWidth, int screenHeight);
void dampSmallVelocities();
void wakeBody(RigidBody &body);

int findFreeBodySlot();
int findBodyIndexAtPoint(const vector2d &point);
bool spawnCircle(const vector2d &position, double radius, BodyColor color);
bool spawnBox(const vector2d &position, double width, double height, BodyColor color);
bool spawnRegularPolygon(const vector2d &position, int sides, double radius, BodyColor color);
bool setBodyColor(int bodyIndex, BodyColor color);
bool resizeCircleBody(int bodyIndex, double radius);
bool resizeBoxBody(int bodyIndex, double width, double height);
bool resizeRegularPolygonBody(int bodyIndex, int sides, double radius);
bool deleteBody(int bodyIndex);
bool deleteBodyAtPoint(const vector2d &point);
