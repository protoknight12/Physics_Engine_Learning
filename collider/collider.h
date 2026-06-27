#pragma once

#include "vector2d/vector2d.h"

enum ShapeType {
    CIRCLE,
    BOX
};

struct Collider {
    ShapeType type;
    double friction;
    double restitution;
    virtual ~Collider() = default;
};

struct CircleCollider : Collider {
    double radius;
};

struct BoxCollider : Collider {
    double width;
    double height;
};

struct RigidBody {
    vector2d pos;
    vector2d linearVel;
    double angle;
    double angularVel;
    vector2d force;
    double torque;
    double mass;
    double inverseMass;
    double momentOfInertia;
    double inverseMomentOfInertia;
    Collider *collider;
};

struct CollisionManifold {
    bool collision;
    double penetration;
    vector2d normal;
    vector2d contactPoint;
};

CircleCollider *asCircleCollider(Collider *collider);
const CircleCollider *asCircleCollider(const Collider *collider);
BoxCollider *asBoxCollider(Collider *collider);
const BoxCollider *asBoxCollider(const Collider *collider);
