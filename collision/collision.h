#pragma once

#include "collider/collider.h"

typedef CollisionManifold (*CollisionHandler)(RigidBody *, RigidBody *);

extern CollisionHandler CollisionMatrix[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT];

double calculateBoxInertia(double mass, double width, double height);
double calculateCircleInertia(double mass, double radius);
double calculatePolygonInertia(double mass,
                               const vector2d *localVertices,
                               int vertexCount);

bool getBoxVertices(const RigidBody *body, vector2d vertices[4]);
bool getPolygonWorldVertices(const RigidBody *body,
                             vector2d *vertices,
                             int &vertexCount);
bool isPointInRigidBody(const vector2d &point, const RigidBody &body);

CollisionManifold boxVsBox(RigidBody *bodyA, RigidBody *bodyB);
CollisionManifold circleVsCircle(RigidBody *bodyA, RigidBody *bodyB);
CollisionManifold circleVsBox(RigidBody *circleBody, RigidBody *boxBody);
CollisionManifold boxVsCircle(RigidBody *boxBody, RigidBody *circleBody);
CollisionManifold circleVsPolygon(RigidBody *circleBody, RigidBody *polygonBody);
CollisionManifold polygonVsCircle(RigidBody *polygonBody, RigidBody *circleBody);
CollisionManifold boxVsPolygon(RigidBody *boxBody, RigidBody *polygonBody);
CollisionManifold polygonVsBox(RigidBody *polygonBody, RigidBody *boxBody);
CollisionManifold polygonVsPolygon(RigidBody *bodyA, RigidBody *bodyB);

void resolveCollision(RigidBody *bodyA, RigidBody *bodyB, const CollisionManifold &manifold);
