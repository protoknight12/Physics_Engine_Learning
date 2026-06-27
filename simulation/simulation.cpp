#define USE_MATH_DEFINES
#include "simulation.h"
#include "collision/collision.h"
#include <random>
#include <cmath>

RigidBody rigidBodies[RIGIDBODY_COUNT];

namespace {
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
    environmentCollider.restitution = 0.2;
    environment.collider = &environmentCollider;
    return environment;
}
} // namespace

void initRigidBodies() {
    std::random_device randomDevice;
    std::mt19937 randomGenerator(randomDevice());
    std::uniform_real_distribution<double> randomXDistribution(150.0, 1130.0);
    std::uniform_real_distribution<double> randomYDistribution(100.0, 400.0);
    std::uniform_real_distribution<double> randomAngleDistribution(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> randomSizeDistribution(MIN_COLLIDER_SIZE, MAX_COLLIDER_SIZE);
    std::uniform_real_distribution<double> randomVelocityDistribution(-100.0, 100.0);

    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        RigidBody &body = rigidBodies[bodyIndex];
        body.pos = vector2d{randomXDistribution(randomGenerator),
                            randomYDistribution(randomGenerator)};
        body.angle = randomAngleDistribution(randomGenerator);
        body.linearVel = vector2d{randomVelocityDistribution(randomGenerator),
                                  randomVelocityDistribution(randomGenerator)};
        body.angularVel = randomVelocityDistribution(randomGenerator) * 0.02;
        body.force = vector2d{0.0, 0.0};
        body.torque = 0.0;
        body.mass = 15.0;
        body.inverseMass = 1.0 / body.mass;

        if (bodyIndex % 2 == 0) {
            const double width = randomSizeDistribution(randomGenerator);
            const double height = randomSizeDistribution(randomGenerator);
            body.momentOfInertia = calculateBoxInertia(body.mass, width, height);
            body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;

            auto *boxCollider = new BoxCollider();
            boxCollider->type = ShapeType::BOX;
            boxCollider->width = width;
            boxCollider->height = height;
            boxCollider->friction = 0.3;
            boxCollider->restitution = 0.3;
            body.collider = boxCollider;
        } else {
            const double radius = randomSizeDistribution(randomGenerator) / 2.0;
            body.momentOfInertia = calculateCircleInertia(body.mass, radius);
            body.inverseMomentOfInertia = 1.0 / body.momentOfInertia;

            auto *circleCollider = new CircleCollider();
            circleCollider->type = ShapeType::CIRCLE;
            circleCollider->radius = radius;
            circleCollider->friction = 0.2;
            circleCollider->restitution = 0.5;
            body.collider = circleCollider;
        }
    }
}

void computeForceAndTorque(RigidBody *rigidBody) {
    rigidBody->force = vector2d{0.0, rigidBody->mass * GRAVITY};
    rigidBody->torque = 0.0;
}

void integrateBodies(const double deltaTime) {
    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        RigidBody *body = &rigidBodies[bodyIndex];
        computeForceAndTorque(body);

        const vector2d linearAcceleration =
            scalarMultiplyVector(body->force, body->inverseMass);
        body->linearVel = addVector(body->linearVel,
            scalarMultiplyVector(linearAcceleration, deltaTime));

        const double angularAcceleration = body->torque * body->inverseMomentOfInertia;
        body->angularVel += angularAcceleration * deltaTime;

        const double speedSquared = body->linearVel.x * body->linearVel.x +
                                    body->linearVel.y * body->linearVel.y;
        if (speedSquared < LINEAR_SLEEP_THRESHOLD_SQ) {
            body->linearVel = vector2d{0.0, 0.0};
        }
        if (std::abs(body->angularVel) < ANGULAR_SLEEP_THRESHOLD) {
            body->angularVel = 0.0;
        }

        body->pos = addVector(body->pos, scalarMultiplyVector(body->linearVel, deltaTime));
        body->angle += body->angularVel * deltaTime;

        body->force = vector2d{0.0, 0.0};
        body->torque = 0.0;
    }
}

void solveCollisions(const int impulseIterations) {
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

            for (int vertexIndex = 0; vertexIndex < 4; ++vertexIndex) {
                const vector2d &vertex = vertices[vertexIndex];

                if (vertex.y > screenHeight) {
                    CollisionManifold manifold{};
                    manifold.collision = true;
                    manifold.penetration = vertex.y - screenHeight;
                    manifold.normal = vector2d{0.0, 1.0};
                    manifold.contactPoint = vertex;
                    resolveBoundaryContact(body, environment, manifold);
                }
                if (vertex.y < 0) {
                    CollisionManifold manifold{};
                    manifold.collision = true;
                    manifold.penetration = 0 - vertex.y;
                    manifold.normal = vector2d{0.0, -1.0};
                    manifold.contactPoint = vertex;
                    resolveBoundaryContact(body, environment, manifold);
                }
                if (vertex.x < 0) {
                    CollisionManifold manifold{};
                    manifold.collision = true;
                    manifold.penetration = 0 - vertex.x;
                    manifold.normal = vector2d{-1.0, 0.0};
                    manifold.contactPoint = vertex;
                    resolveBoundaryContact(body, environment, manifold);
                }
                if (vertex.x > screenWidth) {
                    CollisionManifold manifold{};
                    manifold.collision = true;
                    manifold.penetration = vertex.x - screenWidth;
                    manifold.normal = vector2d{1.0, 0.0};
                    manifold.contactPoint = vertex;
                    resolveBoundaryContact(body, environment, manifold);
                }
            }
        }
    }
}
