#define USE_MATH_DEFINES
#include "simulation.h"
#include "collision/collision.h"
#include <cfloat>
#include <cmath>

RigidBody rigidBodies[RIGIDBODY_COUNT];

namespace {
constexpr double BOUNDARY_CONTACT_EPSILON = 1e-6;

void resolveBoundaryContact(RigidBody &body,
                            RigidBody &environment,
                            const CollisionManifold &manifold) {
    environment.pos = manifold.contactPoint;
    resolveCollision(&body, &environment, manifold);
}

RigidBody makeBoundaryEnvironment(Collider &environmentCollider) {
    RigidBody environment{};
    environment.linearVel = vector2d{0.0, 0.0};
    environment.angle = 0.0;
    environment.angularVel = 0.0;
    environment.inverseMass = 0.0;
    environment.inverseMomentOfInertia = 0.0;
    environmentCollider.type = ShapeType::BOX;
    environmentCollider.friction = 0.5;
    environmentCollider.restitution = 0.0;
    environment.collider = &environmentCollider;
    return environment;
}

struct BoundaryPenetrations {
    double bottom = 0.0;
    double top = 0.0;
    double left = 0.0;
    double right = 0.0;
    double maxY = -DBL_MAX;
    vector2d bottomContact{};
    vector2d topContact{};
    vector2d leftContact{};
    vector2d rightContact{};
    int bottomContactCount = 0;
    int topContactCount = 0;
    int leftContactCount = 0;
    int rightContactCount = 0;
};

void collectDeepestBoundaryContact(const double penetration,
                                   const vector2d &vertex,
                                   double &deepestPenetration,
                                   vector2d &contactSum,
                                   int &contactCount) {
    if (penetration <= 0.0) {
        return;
    }

    if (penetration > deepestPenetration + BOUNDARY_CONTACT_EPSILON) {
        deepestPenetration = penetration;
        contactSum = vertex;
        contactCount = 1;
    } else if (std::abs(penetration - deepestPenetration) <= BOUNDARY_CONTACT_EPSILON) {
        contactSum = addVector(contactSum, vertex);
        ++contactCount;
    }
}

void averageBoundaryContacts(BoundaryPenetrations &penetrations) {
    if (penetrations.bottomContactCount > 0) {
        penetrations.bottomContact =
            scalarMultiplyVector(penetrations.bottomContact,
                                 1.0 / penetrations.bottomContactCount);
    }
    if (penetrations.topContactCount > 0) {
        penetrations.topContact =
            scalarMultiplyVector(penetrations.topContact,
                                 1.0 / penetrations.topContactCount);
    }
    if (penetrations.leftContactCount > 0) {
        penetrations.leftContact =
            scalarMultiplyVector(penetrations.leftContact,
                                 1.0 / penetrations.leftContactCount);
    }
    if (penetrations.rightContactCount > 0) {
        penetrations.rightContact =
            scalarMultiplyVector(penetrations.rightContact,
                                 1.0 / penetrations.rightContactCount);
    }
}

BoundaryPenetrations collectBoundaryPenetrations(const vector2d *vertices,
                                                 const int vertexCount,
                                                 const int screenWidth,
                                                 const int screenHeight) {
    BoundaryPenetrations penetrations{};
    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        const vector2d &vertex = vertices[vertexIndex];
        if (vertex.y > penetrations.maxY) {
            penetrations.maxY = vertex.y;
        }

        collectDeepestBoundaryContact(vertex.y - screenHeight,
                                      vertex,
                                      penetrations.bottom,
                                      penetrations.bottomContact,
                                      penetrations.bottomContactCount);
        collectDeepestBoundaryContact(0.0 - vertex.y,
                                      vertex,
                                      penetrations.top,
                                      penetrations.topContact,
                                      penetrations.topContactCount);
        collectDeepestBoundaryContact(0.0 - vertex.x,
                                      vertex,
                                      penetrations.left,
                                      penetrations.leftContact,
                                      penetrations.leftContactCount);
        collectDeepestBoundaryContact(vertex.x - screenWidth,
                                      vertex,
                                      penetrations.right,
                                      penetrations.rightContact,
                                      penetrations.rightContactCount);
    }
    averageBoundaryContacts(penetrations);
    return penetrations;
}

BoundaryPenetrations collectBoxBoundaryPenetrations(const vector2d vertices[4],
                                                    const int screenWidth,
                                                    const int screenHeight) {
    return collectBoundaryPenetrations(vertices, 4, screenWidth, screenHeight);
}

void resolveBoxBoundaryPenetrations(RigidBody &body,
                                    RigidBody &environment,
                                    const BoundaryPenetrations &penetrations,
                                    const int screenHeight) {
    if (screenHeight - penetrations.maxY <= SLEEP_POSITION_SLOP) {
        body.hasRestingContact = true;
    }

    if (penetrations.bottom > 0.0) {
        CollisionManifold manifold{};
        manifold.collision = true;
        manifold.penetration = penetrations.bottom;
        manifold.normal = vector2d{0.0, 1.0};
        manifold.contactPoint = penetrations.bottomContact;
        resolveBoundaryContact(body, environment, manifold);
    }
    if (penetrations.top > 0.0) {
        CollisionManifold manifold{};
        manifold.collision = true;
        manifold.penetration = penetrations.top;
        manifold.normal = vector2d{0.0, -1.0};
        manifold.contactPoint = penetrations.topContact;
        resolveBoundaryContact(body, environment, manifold);
    }
    if (penetrations.left > 0.0) {
        CollisionManifold manifold{};
        manifold.collision = true;
        manifold.penetration = penetrations.left;
        manifold.normal = vector2d{-1.0, 0.0};
        manifold.contactPoint = penetrations.leftContact;
        resolveBoundaryContact(body, environment, manifold);
    }
    if (penetrations.right > 0.0) {
        CollisionManifold manifold{};
        manifold.collision = true;
        manifold.penetration = penetrations.right;
        manifold.normal = vector2d{1.0, 0.0};
        manifold.contactPoint = penetrations.rightContact;
        resolveBoundaryContact(body, environment, manifold);
    }
}

void setupRegularPolygonVertices(PolygonCollider *polygonCollider,
                                 const int sides,
                                 const double radius) {
    polygonCollider->vertexCount = sides;
    for (int vertexIndex = 0; vertexIndex < sides; ++vertexIndex) {
        const double angle = (2.0 * M_PI * vertexIndex / sides) - (M_PI / 2.0);
        polygonCollider->localVertices[vertexIndex] = {
            radius * cos(angle),
            radius * sin(angle)
        };
    }
}

void resetRigidBody(RigidBody &body) {
    body.pos = vector2d{0.0, 0.0};
    body.linearVel = vector2d{0.0, 0.0};
    body.angle = 0.0;
    body.angularVel = 0.0;
    body.force = vector2d{0.0, 0.0};
    body.torque = 0.0;
    body.mass = 15.0;
    body.inverseMass = 1.0 / body.mass;
    body.momentOfInertia = 1.0;
    body.inverseMomentOfInertia = 1.0;
    body.color = BodyColor{128, 0, 128, 255};
    body.isSleeping = false;
    body.sleepFrameCount = 0;
    body.hasRestingContact = false;
}

void initializeSpawnedBody(RigidBody &body, const vector2d &position) {
    resetRigidBody(body);
    body.pos = position;
}
} // namespace

void initRigidBodies() {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        resetRigidBody(rigidBodies[bodyIndex]);
        rigidBodies[bodyIndex].collider = nullptr;
    }
}

void computeForceAndTorque(RigidBody *rigidBody) {
    rigidBody->force = vector2d{0.0, rigidBody->mass * GRAVITY};
    rigidBody->torque = 0.0;
}

void wakeBody(RigidBody &body) {
    body.isSleeping = false;
    body.sleepFrameCount = 0;
    body.hasRestingContact = false;
}

void integrateBodies(const double deltaTime) {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        RigidBody *body = &rigidBodies[bodyIndex];
        if (!body->collider) {
            continue;
        }
        if (body->isSleeping) {
            body->force = vector2d{0.0, 0.0};
            body->torque = 0.0;
            continue;
        }

        computeForceAndTorque(body);

        const vector2d linearAcceleration =
            scalarMultiplyVector(body->force, body->inverseMass);
        body->linearVel = addVector(body->linearVel,
            scalarMultiplyVector(linearAcceleration, deltaTime));

        const double angularAcceleration = body->torque * body->inverseMomentOfInertia;
        body->angularVel += angularAcceleration * deltaTime;

        body->pos = addVector(body->pos, scalarMultiplyVector(body->linearVel, deltaTime));
        body->angle += body->angularVel * deltaTime;

        body->force = vector2d{0.0, 0.0};
        body->torque = 0.0;
    }
}

void solveCollisions(const int impulseIterations) {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        rigidBodies[bodyIndex].hasRestingContact = false;
    }

    for (int iterationIndex = 0; iterationIndex < impulseIterations; ++iterationIndex) {
        for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
            for (int partnerIndex = bodyIndex + 1; partnerIndex < RIGIDBODY_COUNT; ++partnerIndex) {
                RigidBody *bodyA = &rigidBodies[bodyIndex];
                RigidBody *bodyB = &rigidBodies[partnerIndex];

                if (!bodyA->collider || !bodyB->collider) {
                    continue;
                }

                const int colliderTypeA = static_cast<int>(bodyA->collider->type);
                const int colliderTypeB = static_cast<int>(bodyB->collider->type);

                CollisionHandler handler = CollisionMatrix[colliderTypeA][colliderTypeB];
                if (handler != nullptr) {
                    const CollisionManifold manifold = handler(bodyA, bodyB);
                    if (manifold.collision) {
                        resolveCollision(bodyA, bodyB, manifold);
                    }
                }
            }
        }
    }
}

void screenBoundaryConstraints(const int screenWidth, const int screenHeight) {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        RigidBody &body = rigidBodies[bodyIndex];
        if (!body.collider) {
            continue;
        }

        Collider environmentCollider;
        RigidBody environment = makeBoundaryEnvironment(environmentCollider);

        if (body.collider->type == ShapeType::CIRCLE) {
            const CircleCollider *circleCollider = asCircleCollider(body.collider);
            if (!circleCollider) {
                continue;
            }

            const double radius = circleCollider->radius;
            if (screenHeight - (body.pos.y + radius) <= SLEEP_POSITION_SLOP) {
                body.hasRestingContact = true;
            }

            if (body.pos.y + radius > screenHeight) {
                CollisionManifold manifold{};
                manifold.collision = true;
                manifold.penetration = (body.pos.y + radius) - screenHeight;
                manifold.normal = vector2d{0.0, 1.0};
                manifold.contactPoint = vector2d{body.pos.x, body.pos.y + radius};
                resolveBoundaryContact(body, environment, manifold);
            }
            if (body.pos.y - radius < 0) {
                CollisionManifold manifold{};
                manifold.collision = true;
                manifold.penetration = 0 - (body.pos.y - radius);
                manifold.normal = vector2d{0.0, -1.0};
                manifold.contactPoint = vector2d{body.pos.x, body.pos.y - radius};
                resolveBoundaryContact(body, environment, manifold);
            }
            if (body.pos.x - radius < 0) {
                CollisionManifold manifold{};
                manifold.collision = true;
                manifold.penetration = 0 - (body.pos.x - radius);
                manifold.normal = vector2d{-1.0, 0.0};
                manifold.contactPoint = vector2d{body.pos.x - radius, body.pos.y};
                resolveBoundaryContact(body, environment, manifold);
            }
            if (body.pos.x + radius > screenWidth) {
                CollisionManifold manifold{};
                manifold.collision = true;
                manifold.penetration = (body.pos.x + radius) - screenWidth;
                manifold.normal = vector2d{1.0, 0.0};
                manifold.contactPoint = vector2d{body.pos.x + radius, body.pos.y};
                resolveBoundaryContact(body, environment, manifold);
            }
        } else if (body.collider->type == ShapeType::BOX) {
            vector2d vertices[4];
            if (!getBoxVertices(&body, vertices)) {
                continue;
            }

            const BoundaryPenetrations penetrations =
                collectBoxBoundaryPenetrations(vertices, screenWidth, screenHeight);
            resolveBoxBoundaryPenetrations(body, environment, penetrations, screenHeight);
        } else if (body.collider->type == ShapeType::POLYGON) {
            vector2d vertices[MAX_POLYGON_VERTICES];
            int vertexCount = 0;
            if (!getPolygonWorldVertices(&body, vertices, vertexCount)) {
                continue;
            }

            const BoundaryPenetrations penetrations =
                collectBoundaryPenetrations(vertices, vertexCount, screenWidth, screenHeight);
            resolveBoxBoundaryPenetrations(body, environment, penetrations, screenHeight);
        }
    }
}

void dampSmallVelocities() {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        RigidBody *body = &rigidBodies[bodyIndex];
        if (!body->collider) {
            continue;
        }

        const double speedSquared = body->linearVel.x * body->linearVel.x +
                                    body->linearVel.y * body->linearVel.y;
        if (speedSquared < LINEAR_SLEEP_THRESHOLD_SQ) {
            body->linearVel = vector2d{0.0, 0.0};
        }
        if (std::abs(body->angularVel) < ANGULAR_SLEEP_THRESHOLD) {
            body->angularVel = 0.0;
        }

        if (!body->hasRestingContact) {
            if (!body->isSleeping) {
                body->sleepFrameCount = 0;
            }
            continue;
        }

        const double dampedSpeedSquared = body->linearVel.x * body->linearVel.x +
                                          body->linearVel.y * body->linearVel.y;
        if (dampedSpeedSquared <= SLEEP_LINEAR_THRESHOLD_SQ &&
            std::abs(body->angularVel) <= SLEEP_ANGULAR_THRESHOLD) {
            ++body->sleepFrameCount;
            if (body->sleepFrameCount >= SLEEP_FRAME_THRESHOLD) {
                body->linearVel = vector2d{0.0, 0.0};
                body->angularVel = 0.0;
                body->force = vector2d{0.0, 0.0};
                body->torque = 0.0;
                body->isSleeping = true;
            }
        } else {
            body->sleepFrameCount = 0;
            body->isSleeping = false;
        }
    }
}

int findFreeBodySlot() {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        if (!rigidBodies[bodyIndex].collider) {
            return bodyIndex;
        }
    }
    return -1;
}

int findBodyIndexAtPoint(const vector2d &point) {
    for (int bodyIndex = RIGIDBODY_COUNT - 1; bodyIndex >= 0; --bodyIndex) {
        if (isPointInRigidBody(point, rigidBodies[bodyIndex])) {
            return bodyIndex;
        }
    }
    return -1;
}

bool spawnCircle(const vector2d &position, const double radius, const BodyColor color) {
    const int bodyIndex = findFreeBodySlot();
    if (bodyIndex < 0) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    initializeSpawnedBody(body, position);
    body.color = color;
    body.momentOfInertia = calculateCircleInertia(body.mass, radius);
    body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;

    auto *circleCollider = new CircleCollider();
    circleCollider->type = ShapeType::CIRCLE;
    circleCollider->radius = radius;
    circleCollider->friction = 0.2;
    circleCollider->restitution = 0.5;
    body.collider = circleCollider;
    return true;
}

bool spawnBox(const vector2d &position,
              const double width,
              const double height,
              const BodyColor color) {
    const int bodyIndex = findFreeBodySlot();
    if (bodyIndex < 0) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    initializeSpawnedBody(body, position);
    body.color = color;
    body.momentOfInertia = calculateBoxInertia(body.mass, width, height);
    body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;

    auto *boxCollider = new BoxCollider();
    boxCollider->type = ShapeType::BOX;
    boxCollider->width = width;
    boxCollider->height = height;
    boxCollider->friction = 0.3;
    boxCollider->restitution = 0.3;
    body.collider = boxCollider;
    return true;
}

bool spawnRegularPolygon(const vector2d &position,
                         const int sides,
                         const double radius,
                         const BodyColor color) {
    if (sides < 3 || sides > MAX_POLYGON_VERTICES) {
        return false;
    }

    const int bodyIndex = findFreeBodySlot();
    if (bodyIndex < 0) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    initializeSpawnedBody(body, position);
    body.color = color;

    auto *polygonCollider = new PolygonCollider();
    polygonCollider->type = ShapeType::POLYGON;
    polygonCollider->friction = 0.3;
    polygonCollider->restitution = 0.3;
    setupRegularPolygonVertices(polygonCollider, sides, radius);
    body.momentOfInertia = calculatePolygonInertia(body.mass,
                                                   polygonCollider->localVertices,
                                                   polygonCollider->vertexCount);
    body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;
    body.collider = polygonCollider;
    return true;
}

bool setBodyColor(const int bodyIndex, const BodyColor color) {
    if (bodyIndex < 0 || bodyIndex >= RIGIDBODY_COUNT || !rigidBodies[bodyIndex].collider) {
        return false;
    }

    rigidBodies[bodyIndex].color = color;
    return true;
}

bool resizeCircleBody(const int bodyIndex, const double radius) {
    if (bodyIndex < 0 || bodyIndex >= RIGIDBODY_COUNT) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    CircleCollider *circleCollider = asCircleCollider(body.collider);
    if (!circleCollider || radius <= 0.0) {
        return false;
    }

    circleCollider->radius = radius;
    body.momentOfInertia = calculateCircleInertia(body.mass, radius);
    body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;
    return true;
}

bool resizeBoxBody(const int bodyIndex, const double width, const double height) {
    if (bodyIndex < 0 || bodyIndex >= RIGIDBODY_COUNT) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    BoxCollider *boxCollider = asBoxCollider(body.collider);
    if (!boxCollider || width <= 0.0 || height <= 0.0) {
        return false;
    }

    boxCollider->width = width;
    boxCollider->height = height;
    body.momentOfInertia = calculateBoxInertia(body.mass, width, height);
    body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;
    return true;
}

bool resizeRegularPolygonBody(const int bodyIndex, const int sides, const double radius) {
    if (bodyIndex < 0 ||
        bodyIndex >= RIGIDBODY_COUNT ||
        sides < 3 ||
        sides > MAX_POLYGON_VERTICES ||
        radius <= 0.0) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    PolygonCollider *polygonCollider = asPolygonCollider(body.collider);
    if (!polygonCollider) {
        return false;
    }

    setupRegularPolygonVertices(polygonCollider, sides, radius);
    body.momentOfInertia = calculatePolygonInertia(body.mass,
                                                   polygonCollider->localVertices,
                                                   polygonCollider->vertexCount);
    body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;
    return true;
}

bool deleteBody(const int bodyIndex) {
    if (bodyIndex < 0 || bodyIndex >= RIGIDBODY_COUNT) {
        return false;
    }

    RigidBody &body = rigidBodies[bodyIndex];
    if (!body.collider) {
        return false;
    }

    delete body.collider;
    body.collider = nullptr;
    resetRigidBody(body);
    return true;
}

bool deleteBodyAtPoint(const vector2d &point) {
    const int bodyIndex = findBodyIndexAtPoint(point);
    if (bodyIndex < 0) {
        return false;
    }
    return deleteBody(bodyIndex);
}
