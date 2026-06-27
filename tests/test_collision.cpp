#include "test_helpers.h"
#include "collision/collision.h"
#include "collider/collider.h"

namespace {
RigidBody makeCircleBody(const double positionX, const double positionY, const double radius) {
    RigidBody body{};
    body.pos = {positionX, positionY};
    body.collider = new CircleCollider();
    body.collider->type = ShapeType::CIRCLE;
    static_cast<CircleCollider *>(body.collider)->radius = radius;
    return body;
}

RigidBody makeBoxBody(const double positionX,
                      const double positionY,
                      const double width,
                      const double height) {
    RigidBody body{};
    body.pos = {positionX, positionY};
    body.angle = 0.0;
    body.collider = new BoxCollider();
    body.collider->type = ShapeType::BOX;
    static_cast<BoxCollider *>(body.collider)->width = width;
    static_cast<BoxCollider *>(body.collider)->height = height;
    return body;
}
} // namespace

void registerCollisionTests(int &failureCount) {
    {
        RigidBody firstBody = makeCircleBody(0, 0, 10);
        RigidBody secondBody = makeCircleBody(25, 0, 10);
        const CollisionManifold manifold = circleVsCircle(&firstBody, &secondBody);
        failureCount += runTest("circleVsCircle separated", !manifold.collision);
        delete firstBody.collider;
        delete secondBody.collider;
    }

    {
        RigidBody firstBody = makeCircleBody(0, 0, 10);
        RigidBody secondBody = makeCircleBody(19.9, 0, 10);
        const CollisionManifold manifold = circleVsCircle(&firstBody, &secondBody);
        failureCount += runTest("circleVsCircle barely overlapping", manifold.collision);
        failureCount += runTest("circleVsCircle barely overlapping penetration",
                                manifold.penetration > 0.0);
        delete firstBody.collider;
        delete secondBody.collider;
    }

    {
        RigidBody firstBody = makeCircleBody(0, 0, 10);
        RigidBody secondBody = makeCircleBody(0, 0, 12);
        const CollisionManifold manifold = circleVsCircle(&firstBody, &secondBody);
        failureCount += runTest("circleVsCircle coincident centers", manifold.collision);
        failureCount += runTest("circleVsCircle coincident penetration",
                                nearlyEqual(manifold.penetration, 22.0));
        delete firstBody.collider;
        delete secondBody.collider;
    }

    {
        RigidBody firstBody = makeBoxBody(0, 0, 40, 40);
        RigidBody secondBody = makeBoxBody(30, 0, 40, 40);
        const CollisionManifold manifold = boxVsBox(&firstBody, &secondBody);
        failureCount += runTest("boxVsBox overlap", manifold.collision);
        failureCount += runTest("boxVsBox positive penetration", manifold.penetration > 0.0);
        delete firstBody.collider;
        delete secondBody.collider;
    }

    {
        RigidBody firstBody = makeBoxBody(0, 0, 40, 40);
        RigidBody secondBody = makeBoxBody(80, 0, 40, 40);
        const CollisionManifold manifold = boxVsBox(&firstBody, &secondBody);
        failureCount += runTest("boxVsBox separated", !manifold.collision);
        delete firstBody.collider;
        delete secondBody.collider;
    }

    {
        RigidBody circleBody = makeCircleBody(0, 0, 10);
        RigidBody boxBody = makeBoxBody(25, 0, 40, 40);
        const CollisionManifold manifold = circleVsBox(&circleBody, &boxBody);
        failureCount += runTest("circleVsBox overlap", manifold.collision);
        delete circleBody.collider;
        delete boxBody.collider;
    }

    {
        RigidBody body{};
        body.pos = {0, 0};
        body.collider = new CircleCollider();
        body.collider->type = ShapeType::CIRCLE;
        static_cast<CircleCollider *>(body.collider)->radius = 10;

        vector2d vertices[4];
        failureCount += runTest("getBoxVertices rejects circle collider", !getBoxVertices(&body, vertices));
        delete body.collider;
    }
}
