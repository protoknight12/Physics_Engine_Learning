#define USE_MATH_DEFINES
#include <cfloat>
#include <iostream>
#include <random>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "vector2d/vector2d.h"
#include "config.h"
#include <C:/raylib/raylib/src/raylib.h>
using namespace std;



enum ShapeType {
    CIRCLE,
    BOX,
    POLYGON
};

struct Collider {
    ShapeType type;
    double friction;
    double restitution;
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

RigidBody rigidBodies[RIGIDBODY_COUNT];

// --- MOMENT OF INERTIA CALCULATORS ---
double calculateBoxInertia(double mass, double width, double height) {
    return (mass * (width * width + height * height)) / 12.0;
}

double calculateCircleInertia(double mass, double radius) {
    return (mass * radius * radius) / 2.0;
}

// --- SAT GEOMETRY HELPER FUNCTIONS ---
void GetBoxVertices(RigidBody *body, vector2d vertices[4]) {
    BoxCollider *box = static_cast<BoxCollider *>(body->collider);
    double halfW = box->width / 2.0;
    double halfH = box->height / 2.0;

    double c = cos(body->angle);
    double s = sin(body->angle);

    vector2d localVertices[4] = {
        {-halfW, -halfH},
        {halfW, -halfH},
        {halfW, halfH},
        {-halfW, halfH}
    };

    for (int i = 0; i < 4; ++i) {
        vertices[i].x = body->pos.x + (localVertices[i].x * c - localVertices[i].y * s);
        vertices[i].y = body->pos.y + (localVertices[i].x * s + localVertices[i].y * c);
    }
}

void ProjectVertices(vector2d vertices[4], vector2d axis, double &minProj, double &maxProj) {
    minProj = dotProduct(vertices[0], axis);
    maxProj = minProj;
    for (int i = 1; i < 4; ++i) {
        double proj = dotProduct(vertices[i], axis);
        if (proj < minProj) minProj = proj;
        if (proj > maxProj) maxProj = proj;
    }
}

// --- ADVANCED NARROW PHASE INTERSECTION (SAT) ---

CollisionManifold BoxVsBox(RigidBody *a, RigidBody *b) {
    CollisionManifold manifold;
    manifold.collision = false;
    manifold.penetration = DBL_MAX;
    manifold.normal = vector2d{0.0, 0.0};
    manifold.contactPoint = vector2d{0.0, 0.0};

    vector2d vertsA[4];
    vector2d vertsB[4];
    GetBoxVertices(a, vertsA);
    GetBoxVertices(b, vertsB);

    // Generate the 4 unique projection axes (face normals) from both boxes
    vector2d axes[4] = {
        normalizeVector(subtractVector(vertsA[1], vertsA[0])),
        normalizeVector(subtractVector(vertsA[3], vertsA[0])),
        normalizeVector(subtractVector(vertsB[1], vertsB[0])),
        normalizeVector(subtractVector(vertsB[3], vertsB[0]))
    };

    for (int i = 0; i < 4; ++i) {
        vector2d axis = axes[i];
        double minA, maxA, minB, maxB;

        ProjectVertices(vertsA, axis, minA, maxA);
        ProjectVertices(vertsB, axis, minB, maxB);

        // Check for an overlapping gap along the current projection axis
        if (maxA < minB || maxB < minA) {
            return manifold; // Separation axis found, break early
        }

        double overlap = min(maxA, maxB) - max(minA, minB);
        if (overlap < manifold.penetration) {
            manifold.penetration = overlap;
            manifold.normal = axis;
        }
    }

    manifold.collision = true;

    // Ensure the collision normal points consistently from A to B
    vector2d direction = subtractVector(b->pos, a->pos);
    if (dotProduct(manifold.normal, direction) < 0) {
        manifold.normal = negateVector(manifold.normal);
    }

    // Contact point approximation: Average center of overlapping features
    manifold.contactPoint = scalarMultiplyVector(addVector(a->pos, b->pos), 0.5);
    return manifold;
}

CollisionManifold CircleVsCircle(RigidBody *a, RigidBody *b) {
    CollisionManifold manifold;
    manifold.collision = false;
    manifold.penetration = 0.0;
    manifold.normal = vector2d{0.0, 0.0};
    manifold.contactPoint = vector2d{0.0, 0.0};

    CircleCollider *cA = static_cast<CircleCollider *>(a->collider);
    CircleCollider *cB = static_cast<CircleCollider *>(b->collider);

    double dx = b->pos.x - a->pos.x;
    double dy = b->pos.y - a->pos.y;
    double distanceSquared = (dx * dx) + (dy * dy);
    double radiusSum = cA->radius + cB->radius;

    if (distanceSquared >= (radiusSum * radiusSum)) return manifold;

    double distance = sqrt(distanceSquared);
    manifold.collision = true;

    if (distance != 0.0) {
        manifold.penetration = radiusSum - distance;
        manifold.normal = vector2d{dx / distance, dy / distance};
        manifold.contactPoint = addVector(a->pos, scalarMultiplyVector(manifold.normal, cA->radius));
    } else {
        manifold.penetration = cA->radius;
        manifold.normal = vector2d{1.0, 0.0};
        manifold.contactPoint = a->pos;
    }
    return manifold;
}

CollisionManifold CircleVsBox(RigidBody *circleBody, RigidBody *boxBody) {
    CollisionManifold manifold;
    manifold.collision = false;
    manifold.penetration = 0.0;
    manifold.normal = vector2d{0.0, 0.0};
    manifold.contactPoint = vector2d{0.0, 0.0};

    CircleCollider *circle = static_cast<CircleCollider *>(circleBody->collider);
    BoxCollider *box = static_cast<BoxCollider *>(boxBody->collider);

    // 1. Transform the circle position into the local space of the rotated box
    double dx = circleBody->pos.x - boxBody->pos.x;
    double dy = circleBody->pos.y - boxBody->pos.y;
    double cosTheta = cos(-boxBody->angle);
    double sinTheta = sin(-boxBody->angle);
    vector2d localCircle = {dx * cosTheta - dy * sinTheta, dx * sinTheta + dy * cosTheta};

    double halfW = box->width / 2.0;
    double halfH = box->height / 2.0;

    // 2. Clamp local circle center to the box edges to find the closest point
    double closestX = max(-halfW, min(localCircle.x, halfW));
    double closestY = max(-halfH, min(localCircle.y, halfH));

    double localDx = localCircle.x - closestX;
    double localDy = localCircle.y - closestY;
    double distanceSquared = (localDx * localDx) + (localDy * localDy);

    // No intersection
    if (distanceSquared >= (circle->radius * circle->radius)) return manifold;

    double distance = sqrt(distanceSquared);
    manifold.collision = true;

    vector2d localNormal;
    vector2d localContact = {closestX, closestY};

    if (distance != 0.0) {
        manifold.penetration = circle->radius - distance;
        // Inverted: Points from Circle (A) to Box (B) in local space
        localNormal = vector2d{-localDx / distance, -localDy / distance};
    } else {
        // Deep penetration: Circle center is inside the box. Find shallowest axis to escape.
        double distToLeft = localCircle.x - (-halfW);
        double distToRight = halfW - localCircle.x;
        double distToBottom = localCircle.y - (-halfH);
        double distToTop = halfH - localCircle.y;

        double minDist = min({distToLeft, distToRight, distToBottom, distToTop});

        // Normals point from Circle to Box. Since the solver applies -normal to Body A,
        // the normal must point opposite to the escape direction.
        if (minDist == distToLeft) {
            localNormal = vector2d{1.0, 0.0};
            manifold.penetration = circle->radius + distToLeft;
        } else if (minDist == distToRight) {
            localNormal = vector2d{-1.0, 0.0};
            manifold.penetration = circle->radius + distToRight;
        } else if (minDist == distToBottom) {
            localNormal = vector2d{0.0, 1.0};
            manifold.penetration = circle->radius + distToBottom;
        } else {
            localNormal = vector2d{0.0, -1.0};
            manifold.penetration = circle->radius + distToTop;
        }
        localContact = localCircle;
    }

    // 3. Transform local metrics back into world space coordinates
    double cosOut = cos(boxBody->angle);
    double sinOut = sin(boxBody->angle);

    manifold.normal.x = localNormal.x * cosOut - localNormal.y * sinOut;
    manifold.normal.y = localNormal.x * sinOut + localNormal.y * cosOut;

    manifold.contactPoint.x = boxBody->pos.x + (localContact.x * cosOut - localContact.y * sinOut);
    manifold.contactPoint.y = boxBody->pos.y + (localContact.x * sinOut + localContact.y * cosOut);

    return manifold;
}

CollisionManifold BoxVsCircle(RigidBody *boxBody, RigidBody *circleBody) {
    // Treat circle as A and box as B inside the sub-call (returns Circle->Box normal)
    CollisionManifold manifold = CircleVsBox(circleBody, boxBody);

    // Invert the normal so it correctly represents Box (A) -> Circle (B)
    manifold.normal = negateVector(manifold.normal);
    return manifold;
}

// --- FUNCTION POINTER DISPATCH TABLE ---
typedef CollisionManifold (*CollisionHandler)(RigidBody *, RigidBody *);

CollisionHandler CollisionMatrix[3][3] = {
    {CircleVsCircle, CircleVsBox, nullptr},
    {BoxVsCircle, BoxVsBox, nullptr},
    {nullptr, nullptr, nullptr}
};

// --- IMPULSE RESOLUTION ENGINE ---
void ResolveCollision(RigidBody *bodyA, RigidBody *bodyB, const CollisionManifold &manifold) {
    const double totalInverseMass = bodyA->inverseMass + bodyB->inverseMass;
    if (totalInverseMass == 0) return;

    const vector2d rA = subtractVector(manifold.contactPoint, bodyA->pos);
    const vector2d rB = subtractVector(manifold.contactPoint, bodyB->pos);

    vector2d vA_contact = addVector(bodyA->linearVel, crossScalarVector(bodyA->angularVel, rA));
    vector2d vB_contact = addVector(bodyB->linearVel, crossScalarVector(bodyB->angularVel, rB));

    vector2d relativeVelocity = subtractVector(vB_contact, vA_contact);
    const double velocityAlongNormal = dotProduct(relativeVelocity, manifold.normal);

    if (velocityAlongNormal > 0) return;

    double e = min(bodyA->collider ? bodyA->collider->restitution : 0.0,
                   bodyB->collider ? bodyB->collider->restitution : 0.0);
    if (abs(velocityAlongNormal) < 20.0) {
        e = 0.0;
    }

    const double rA_cross_n = crossVectors(rA, manifold.normal);
    const double rB_cross_n = crossVectors(rB, manifold.normal);
    const double angularComponentA = rA_cross_n * rA_cross_n * bodyA->inverseMomentOfInertia;
    const double angularComponentB = rB_cross_n * rB_cross_n * bodyB->inverseMomentOfInertia;

    const double denominator = totalInverseMass + angularComponentA + angularComponentB;
    double j = -(1.0 + e) * velocityAlongNormal / denominator;

    const vector2d impulseVector = scalarMultiplyVector(manifold.normal, j);

    bodyA->linearVel = subtractVector(bodyA->linearVel, scalarMultiplyVector(impulseVector, bodyA->inverseMass));
    bodyA->angularVel -= rA_cross_n * j * bodyA->inverseMomentOfInertia;

    bodyB->linearVel = addVector(bodyB->linearVel, scalarMultiplyVector(impulseVector, bodyB->inverseMass));
    bodyB->angularVel += rB_cross_n * j * bodyB->inverseMomentOfInertia;

    // Tangential Friction Sub-Pass
    vA_contact = addVector(bodyA->linearVel, crossScalarVector(bodyA->angularVel, rA));
    vB_contact = addVector(bodyB->linearVel, crossScalarVector(bodyB->angularVel, rB));
    relativeVelocity = subtractVector(vB_contact, vA_contact);

    vector2d tangent = subtractVector(relativeVelocity,
                                      scalarMultiplyVector(manifold.normal,
                                                           dotProduct(relativeVelocity, manifold.normal)));
    double tangentMag = sqrt(dotProduct(tangent, tangent));

    if (tangentMag > 0.0001) {
        tangent = vector2d{tangent.x / tangentMag, tangent.y / tangentMag};

        double rA_cross_t = crossVectors(rA, tangent);
        double rB_cross_t = crossVectors(rB, tangent);
        double frictionDenominator = totalInverseMass +
                                     (rA_cross_t * rA_cross_t * bodyA->inverseMomentOfInertia) +
                                     (rB_cross_t * rB_cross_t * bodyB->inverseMomentOfInertia);

        double jt = -dotProduct(relativeVelocity, tangent) / frictionDenominator;
        double mu = min(bodyA->collider ? bodyA->collider->friction : 0.0,
                        bodyB->collider ? bodyB->collider->friction : 0.0);

        if (abs(jt) > j * mu) {
            jt = (jt > 0 ? 1.0 : -1.0) * j * mu;
        }

        vector2d frictionImpulse = scalarMultiplyVector(tangent, jt);
        bodyA->linearVel = subtractVector(bodyA->linearVel, scalarMultiplyVector(frictionImpulse, bodyA->inverseMass));
        bodyA->angularVel -= rA_cross_t * jt * bodyA->inverseMomentOfInertia;

        bodyB->linearVel = addVector(bodyB->linearVel, scalarMultiplyVector(frictionImpulse, bodyB->inverseMass));
        bodyB->angularVel += rB_cross_t * jt * bodyB->inverseMomentOfInertia;
    }

    // Positional Anti-Sink Correction Block
    const double slop = 0.01;
    const double percent = 0.4;
    if (manifold.penetration > slop) {
        const double correctionScalar = (manifold.penetration - slop) / totalInverseMass * percent;
        const vector2d correctionVector = scalarMultiplyVector(manifold.normal, correctionScalar);

        bodyA->pos = subtractVector(bodyA->pos, scalarMultiplyVector(correctionVector, bodyA->inverseMass));
        bodyB->pos = addVector(bodyB->pos, scalarMultiplyVector(correctionVector, bodyB->inverseMass));
    }
}

// --- POPULATE WORLD ENTITIES ---
void initRigidBodies() {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> randX(150.0, 1130.0);
    std::uniform_real_distribution<double> randY(100.0, 400.0);
    std::uniform_real_distribution<double> randAng(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> randSize(40.0, 70.0);
    std::uniform_real_distribution<double> randVel(-100.0, 100.0);

    for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
        RigidBody &body = rigidBodies[i];
        body.pos = vector2d{randX(mt), randY(mt)};
        body.angle = randAng(mt);
        body.linearVel = vector2d{randVel(mt), randVel(mt)};
        body.angularVel = randVel(mt) * 0.02;
        body.force = vector2d{0.0, 0.0};
        body.torque = 0.0;
        body.mass = 15.0;
        body.inverseMass = 1.0 / body.mass;

        if (i % 2 == 0) {
            double width = randSize(mt);
            double height = randSize(mt);
            body.momentOfInertia = calculateBoxInertia(body.mass, width, height);
            body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;

            BoxCollider *box = new BoxCollider();
            box->type = ShapeType::BOX;
            box->width = width;
            box->height = height;
            box->friction = 0.3;
            box->restitution = 0.3;
            body.collider = box;
        } else {
            double radius = randSize(mt) / 2.0;
            body.momentOfInertia = calculateCircleInertia(body.mass, radius);
            body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;

            CircleCollider *circle = new CircleCollider();
            circle->type = ShapeType::CIRCLE;
            circle->radius = radius;
            circle->friction = 0.2;
            circle->restitution = 0.5;
            body.collider = circle;
        }
    }
}

void computeForceAndTorque(RigidBody *rigidBody) {
    // Gravity application scalar
    rigidBody->force = vector2d{0.0, rigidBody->mass * 400.0};
    rigidBody->torque = 0.0;
}

void ScreenBoundaryConstraints(int width, int height) {
    for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
        RigidBody &body = rigidBodies[i];
        if (!body.collider) continue;

        // Create a template for an immovable, infinite-mass boundary environment
        RigidBody environment;
        environment.linearVel = vector2d{0.0, 0.0};
        environment.angle = 0.0;
        environment.angularVel = 0.0;
        environment.inverseMass = 0.0; // 0 means infinite mass (immovable)
        environment.inverseMomentOfInertia = 0.0; // Cannot be rotated by impulses

        Collider envCollider;
        envCollider.type = ShapeType::BOX;
        envCollider.friction = 0.5; // Ground/Wall friction coefficient
        envCollider.restitution = 0.2; // Ground/Wall bounciness
        environment.collider = &envCollider;

        // --- HANDLE CIRCLE BOUNDARIES ---
        if (body.collider->type == ShapeType::CIRCLE) {
            CircleCollider *circle = static_cast<CircleCollider *>(body.collider);
            double radius = circle->radius;

            // Ground
            if (body.pos.y + radius > height) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = (body.pos.y + radius) - height;
                manifold.normal = vector2d{0.0, 1.0}; // Points from Body A into Ground B
                manifold.contactPoint = vector2d{body.pos.x, body.pos.y + radius};
                environment.pos = manifold.contactPoint;
                ResolveCollision(&body, &environment, manifold);
            }
            // Ceiling
            else if (body.pos.y - radius < 0) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = 0 - (body.pos.y - radius);
                manifold.normal = vector2d{0.0, -1.0};
                manifold.contactPoint = vector2d{body.pos.x, body.pos.y - radius};
                environment.pos = manifold.contactPoint;
                ResolveCollision(&body, &environment, manifold);
            }

            // Left Wall
            if (body.pos.x - radius < 0) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = 0 - (body.pos.x - radius);
                manifold.normal = vector2d{-1.0, 0.0};
                manifold.contactPoint = vector2d{body.pos.x - radius, body.pos.y};
                environment.pos = manifold.contactPoint;
                ResolveCollision(&body, &environment, manifold);
            }
            // Right Wall
            else if (body.pos.x + radius > width) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = (body.pos.x + radius) - width;
                manifold.normal = vector2d{1.0, 0.0};
                manifold.contactPoint = vector2d{body.pos.x + radius, body.pos.y};
                environment.pos = manifold.contactPoint;
                ResolveCollision(&body, &environment, manifold);
            }
        }
        // --- HANDLE BOX BOUNDARIES ---
        else if (body.collider->type == ShapeType::BOX) {
            vector2d verts[4];
            GetBoxVertices(&body, verts);

            // Find the deepest penetrating vertex for each potential boundary strike
            vector2d deepestFloorVertex = verts[0];
            vector2d deepestCeilVertex = verts[0];
            vector2d deepestLeftVertex = verts[0];
            vector2d deepestRightVertex = verts[0];

            for (int j = 1; j < 4; ++j) {
                if (verts[j].y > deepestFloorVertex.y) deepestFloorVertex = verts[j];
                if (verts[j].y < deepestCeilVertex.y) deepestCeilVertex = verts[j];
                if (verts[j].x < deepestLeftVertex.x) deepestLeftVertex = verts[j];
                if (verts[j].x > deepestRightVertex.x) deepestRightVertex = verts[j];
            }

            // Ground Contact
            if (deepestFloorVertex.y > height) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = deepestFloorVertex.y - height;
                manifold.normal = vector2d{0.0, 1.0};
                manifold.contactPoint = deepestFloorVertex;
                environment.pos = deepestFloorVertex;
                ResolveCollision(&body, &environment, manifold);
            }
            // Ceiling Contact
            else if (deepestCeilVertex.y < 0) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = 0 - deepestCeilVertex.y;
                manifold.normal = vector2d{0.0, -1.0};
                manifold.contactPoint = deepestCeilVertex;
                environment.pos = deepestCeilVertex;
                ResolveCollision(&body, &environment, manifold);
            }

            // Left Wall Contact
            if (deepestLeftVertex.x < 0) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = 0 - deepestLeftVertex.x;
                manifold.normal = vector2d{-1.0, 0.0};
                manifold.contactPoint = deepestLeftVertex;
                environment.pos = deepestLeftVertex;
                ResolveCollision(&body, &environment, manifold);
            }
            // Right Wall Contact
            else if (deepestRightVertex.x > width) {
                CollisionManifold manifold;
                manifold.collision = true;
                manifold.penetration = deepestRightVertex.x - width;
                manifold.normal = vector2d{1.0, 0.0};
                manifold.contactPoint = deepestRightVertex;
                environment.pos = deepestRightVertex;
                ResolveCollision(&body, &environment, manifold);
            }
        }
    }
}

// --- MAIN SYSTEM EXECUTION ENTRYPOINT ---
int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "RigidBody Physics Core (SAT & Iteration Enabled)");
    SetTargetFPS(GetMonitorRefreshRate(0));

    initRigidBodies();

    const double deltaTime = 0.01666; // Fixed timestep slice
    double accumulator = 0.0;
    const int impulseIterations = 6; // Solves chaining issues and eliminates sinking completely

    while (!WindowShouldClose()) {
        double frameTime = GetFrameTime();
        if (frameTime > 0.25) frameTime = 0.25;
        accumulator += frameTime;

        // --- TICK SUB-STEP TIMESTEP LOOPS ---
        while (accumulator >= deltaTime) {
            // Pass 1: Integration & Kinematics Movement Update
            for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
                RigidBody *body = &rigidBodies[i];
                computeForceAndTorque(body);

                vector2d linearAccel = scalarMultiplyVector(body->force, body->inverseMass);
                body->linearVel = addVector(body->linearVel, scalarMultiplyVector(linearAccel, deltaTime));
                body->pos = addVector(body->pos, scalarMultiplyVector(body->linearVel, deltaTime));

                double angularAccel = body->torque * body->inverseMomentOfInertia;
                body->angularVel += angularAccel * deltaTime;
                double speedSquared = body->linearVel.x * body->linearVel.x + body->linearVel.y * body->linearVel.y;
                if (speedSquared < 0.05) {
                    body->linearVel = vector2d{0.0, 0.0};
                }
                if (abs(body->angularVel) < 0.01) {
                    body->angularVel = 0.0;
                }
                body->pos = addVector(body->pos, scalarMultiplyVector(body->linearVel, deltaTime));
                body->angle += body->angularVel * deltaTime;

                body->force = vector2d{0.0, 0.0};
                body->torque = 0.0;
            }

            // Pass 2: Iterative Impulse Solver Pass (Solves the Whack-A-Mole stack problem)
            for (int k = 0; k < impulseIterations; ++k) {
                for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
                    for (int j = i + 1; j < RIGIDBODY_COUNT; ++j) {
                        RigidBody *bodyA = &rigidBodies[i];
                        RigidBody *bodyB = &rigidBodies[j];

                        if (!bodyA->collider || !bodyB->collider) continue;

                        int typeA = bodyA->collider->type;
                        int typeB = bodyB->collider->type;

                        CollisionHandler handler = CollisionMatrix[typeA][typeB];
                        if (handler != nullptr) {
                            CollisionManifold manifold = handler(bodyA, bodyB);
                            if (manifold.collision) {
                                ResolveCollision(bodyA, bodyB, manifold);
                            }
                        }
                    }
                }
            }

            // Pass 3: Edge Wall Enforcement
            ScreenBoundaryConstraints(screenWidth, screenHeight);

            accumulator -= deltaTime;
        }

        // --- RENDER PIPELINE ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("Physics Engine Engine: SAT ORIENTED COLLIDERS ACTIVE", 15, 15, 16, DARKGRAY);
        DrawFPS(screenWidth - 100, 15);

        for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
            RigidBody &body = rigidBodies[i];
            if (!body.collider) continue;

            Vector2 raylibPos = {static_cast<float>(body.pos.x), static_cast<float>(body.pos.y)};

            if (body.collider->type == ShapeType::CIRCLE) {
                CircleCollider *circle = static_cast<CircleCollider *>(body.collider);

                DrawCircleV(raylibPos, static_cast<float>(circle->radius), PURPLE);

                // Rotation vector line tracker
                Vector2 lineEnd = {
                    raylibPos.x + static_cast<float>(cos(body.angle) * circle->radius),
                    raylibPos.y + static_cast<float>(sin(body.angle) * circle->radius)
                };
                DrawLineV(raylibPos, lineEnd, GOLD);
            } else if (body.collider->type == ShapeType::BOX) {
                BoxCollider *box = static_cast<BoxCollider *>(body.collider);

                float w = static_cast<float>(box->width);
                float h = static_cast<float>(box->height);

                Rectangle rect = {raylibPos.x, raylibPos.y, w, h};
                Vector2 origin = {w / 2.0f, h / 2.0f};
                float rotationDegrees = static_cast<float>(body.angle * (180.0 / M_PI));

                DrawRectanglePro(rect, origin, rotationDegrees, PURPLE);
            }
        }

        EndDrawing();
    }

    // Resource Deallocation Loop
    for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
        delete rigidBodies[i].collider;
    }

    CloseWindow();
    return 0;
}
