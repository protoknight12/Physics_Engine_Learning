#pragma once

#include "collider/collider.h"

typedef CollisionManifold (*CollisionHandler)(RigidBody *, RigidBody *);

extern CollisionHandler CollisionMatrix[2][2];

double calculateBoxInertia(double mass, double width, double height);
double calculateCircleInertia(double mass, double radius);

bool getBoxVertices(const RigidBody *body, vector2d vertices[4]);

CollisionManifold boxVsBox(RigidBody *bodyA, RigidBody *bodyB);
CollisionManifold circleVsCircle(RigidBody *bodyA, RigidBody *bodyB);
CollisionManifold circleVsBox(RigidBody *circleBody, RigidBody *boxBody);
CollisionManifold boxVsCircle(RigidBody *boxBody, RigidBody *circleBody);

void resolveCollision(RigidBody *bodyA, RigidBody *bodyB, const CollisionManifold &manifold);
