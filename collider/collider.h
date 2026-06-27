#pragma once

#include "config.h"
#include "vector2d/vector2d.h"

enum ShapeType {
    CIRCLE,
    BOX,
    POLYGON
};

inline constexpr int SHAPE_TYPE_COUNT = 3;

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

struct PolygonCollider : Collider {
    int vertexCount = 0;
    vector2d localVertices[MAX_POLYGON_VERTICES];
};

struct BodyColor {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
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
    BodyColor color;
    bool isSleeping;
    int sleepFrameCount;
    bool hasRestingContact;
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
PolygonCollider *asPolygonCollider(Collider *collider);
const PolygonCollider *asPolygonCollider(const Collider *collider);
