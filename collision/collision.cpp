#define USE_MATH_DEFINES
#include "collision.h"
#include "config.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace {
constexpr double kAxisEpsilonSq = 1e-12;

bool isValidAxis(const vector2d &axis) {
    return dotProduct(axis, axis) > kAxisEpsilonSq;
}
} // namespace

double calculateBoxInertia(const double mass, const double width, const double height) {
    return (mass * (width * width + height * height)) / 12.0;
}

double calculateCircleInertia(const double mass, const double radius) {
    return (mass * radius * radius) / 2.0;
}

bool getBoxVertices(const RigidBody *body, vector2d vertices[4]) {
    const BoxCollider *boxCollider = asBoxCollider(body->collider);
    if (!boxCollider) {
        return false;
    }

    const double halfWidth = boxCollider->width / 2.0;
    const double halfHeight = boxCollider->height / 2.0;
    const double cosAngle = cos(body->angle);
    const double sinAngle = sin(body->angle);

    const vector2d localVertices[4] = {
        {-halfWidth, -halfHeight},
        {halfWidth, -halfHeight},
        {halfWidth, halfHeight},
        {-halfWidth, halfHeight}
    };

    for (int vertexIndex = 0; vertexIndex < 4; ++vertexIndex) {
        vertices[vertexIndex].x = body->pos.x +
            (localVertices[vertexIndex].x * cosAngle - localVertices[vertexIndex].y * sinAngle);
        vertices[vertexIndex].y = body->pos.y +
            (localVertices[vertexIndex].x * sinAngle + localVertices[vertexIndex].y * cosAngle);
    }

    return true;
}

namespace {
void projectVertices(const vector2d vertices[4],
                     const vector2d axis,
                     double &minProjection,
                     double &maxProjection) {
    minProjection = dotProduct(vertices[0], axis);
    maxProjection = minProjection;
    for (int vertexIndex = 1; vertexIndex < 4; ++vertexIndex) {
        const double projection = dotProduct(vertices[vertexIndex], axis);
        if (projection < minProjection) {
            minProjection = projection;
        }
        if (projection > maxProjection) {
            maxProjection = projection;
        }
    }
}
} // namespace

CollisionManifold boxVsBox(RigidBody *bodyA, RigidBody *bodyB) {
    CollisionManifold manifold{};
    manifold.penetration = DBL_MAX;

    vector2d verticesA[4];
    vector2d verticesB[4];
    if (!getBoxVertices(bodyA, verticesA) || !getBoxVertices(bodyB, verticesB)) {
        return manifold;
    }

    const vector2d edgeVectors[4] = {
        subtractVector(verticesA[1], verticesA[0]),
        subtractVector(verticesA[3], verticesA[0]),
        subtractVector(verticesB[1], verticesB[0]),
        subtractVector(verticesB[3], verticesB[0])
    };

    for (int axisIndex = 0; axisIndex < 4; ++axisIndex) {
        const vector2d axis = normalizeVector(edgeVectors[axisIndex]);
        if (!isValidAxis(axis)) {
            continue;
        }

        double minProjectionA, maxProjectionA, minProjectionB, maxProjectionB;
        projectVertices(verticesA, axis, minProjectionA, maxProjectionA);
        projectVertices(verticesB, axis, minProjectionB, maxProjectionB);

        if (maxProjectionA < minProjectionB || maxProjectionB < minProjectionA) {
            return manifold;
        }

        const double overlap = std::min(maxProjectionA, maxProjectionB) -
                               std::max(minProjectionA, minProjectionB);
        if (overlap < manifold.penetration) {
            manifold.penetration = overlap;
            manifold.normal = axis;
        }
    }

    if (manifold.penetration == DBL_MAX) {
        return manifold;
    }

    manifold.collision = true;

    const vector2d direction = subtractVector(bodyB->pos, bodyA->pos);
    if (dotProduct(manifold.normal, direction) < 0) {
        manifold.normal = negateVector(manifold.normal);
    }

    manifold.contactPoint = scalarMultiplyVector(addVector(bodyA->pos, bodyB->pos), 0.5);
    return manifold;
}

CollisionManifold circleVsCircle(RigidBody *bodyA, RigidBody *bodyB) {
    CollisionManifold manifold{};

    const CircleCollider *circleColliderA = asCircleCollider(bodyA->collider);
    const CircleCollider *circleColliderB = asCircleCollider(bodyB->collider);
    if (!circleColliderA || !circleColliderB) {
        return manifold;
    }

    const double deltaX = bodyB->pos.x - bodyA->pos.x;
    const double deltaY = bodyB->pos.y - bodyA->pos.y;
    const double distanceSquared = (deltaX * deltaX) + (deltaY * deltaY);
    const double radiusSum = circleColliderA->radius + circleColliderB->radius;

    if (distanceSquared >= (radiusSum * radiusSum)) {
        return manifold;
    }

    manifold.collision = true;

    if (distanceSquared > 0.0) {
        const double distance = sqrt(distanceSquared);
        manifold.penetration = radiusSum - distance;
        manifold.normal = vector2d{deltaX / distance, deltaY / distance};
        manifold.contactPoint = addVector(bodyA->pos,
            scalarMultiplyVector(manifold.normal, circleColliderA->radius));
    } else {
        manifold.penetration = radiusSum;
        manifold.normal = vector2d{1.0, 0.0};
        manifold.contactPoint = bodyA->pos;
    }

    return manifold;
}

CollisionManifold circleVsBox(RigidBody *circleBody, RigidBody *boxBody) {
    CollisionManifold manifold{};

    const CircleCollider *circleCollider = asCircleCollider(circleBody->collider);
    const BoxCollider *boxCollider = asBoxCollider(boxBody->collider);
    if (!circleCollider || !boxCollider) {
        return manifold;
    }

    const double deltaX = circleBody->pos.x - boxBody->pos.x;
    const double deltaY = circleBody->pos.y - boxBody->pos.y;
    const double cosLocalAngle = cos(-boxBody->angle);
    const double sinLocalAngle = sin(-boxBody->angle);
    const vector2d localCirclePosition = {
        deltaX * cosLocalAngle - deltaY * sinLocalAngle,
        deltaX * sinLocalAngle + deltaY * cosLocalAngle
    };

    const double halfWidth = boxCollider->width / 2.0;
    const double halfHeight = boxCollider->height / 2.0;

    const double closestX = std::max(-halfWidth, std::min(localCirclePosition.x, halfWidth));
    const double closestY = std::max(-halfHeight, std::min(localCirclePosition.y, halfHeight));

    const double localDeltaX = localCirclePosition.x - closestX;
    const double localDeltaY = localCirclePosition.y - closestY;
    const double distanceSquared = (localDeltaX * localDeltaX) + (localDeltaY * localDeltaY);

    if (distanceSquared >= (circleCollider->radius * circleCollider->radius)) {
        return manifold;
    }

    manifold.collision = true;

    vector2d localNormal;
    vector2d localContactPoint = {closestX, closestY};

    if (distanceSquared > 0.0) {
        const double distance = sqrt(distanceSquared);
        manifold.penetration = circleCollider->radius - distance;
        localNormal = vector2d{-localDeltaX / distance, -localDeltaY / distance};
    } else {
        const double distanceToLeft = localCirclePosition.x - (-halfWidth);
        const double distanceToRight = halfWidth - localCirclePosition.x;
        const double distanceToBottom = localCirclePosition.y - (-halfHeight);
        const double distanceToTop = halfHeight - localCirclePosition.y;

        const double minimumEscapeDistance = std::min({
            distanceToLeft, distanceToRight, distanceToBottom, distanceToTop
        });

        if (minimumEscapeDistance == distanceToLeft) {
            localNormal = vector2d{1.0, 0.0};
            manifold.penetration = circleCollider->radius + distanceToLeft;
        } else if (minimumEscapeDistance == distanceToRight) {
            localNormal = vector2d{-1.0, 0.0};
            manifold.penetration = circleCollider->radius + distanceToRight;
        } else if (minimumEscapeDistance == distanceToBottom) {
            localNormal = vector2d{0.0, 1.0};
            manifold.penetration = circleCollider->radius + distanceToBottom;
        } else {
            localNormal = vector2d{0.0, -1.0};
            manifold.penetration = circleCollider->radius + distanceToTop;
        }
        localContactPoint = localCirclePosition;
    }

    const double cosWorldAngle = cos(boxBody->angle);
    const double sinWorldAngle = sin(boxBody->angle);

    manifold.normal.x = localNormal.x * cosWorldAngle - localNormal.y * sinWorldAngle;
    manifold.normal.y = localNormal.x * sinWorldAngle + localNormal.y * cosWorldAngle;

    manifold.contactPoint.x = boxBody->pos.x +
        (localContactPoint.x * cosWorldAngle - localContactPoint.y * sinWorldAngle);
    manifold.contactPoint.y = boxBody->pos.y +
        (localContactPoint.x * sinWorldAngle + localContactPoint.y * cosWorldAngle);

    return manifold;
}

CollisionManifold boxVsCircle(RigidBody *boxBody, RigidBody *circleBody) {
    CollisionManifold manifold = circleVsBox(circleBody, boxBody);
    manifold.normal = negateVector(manifold.normal);
    return manifold;
}

CollisionHandler CollisionMatrix[2][2] = {
    {circleVsCircle, circleVsBox},
    {boxVsCircle, boxVsBox}
};

void resolveCollision(RigidBody *bodyA, RigidBody *bodyB, const CollisionManifold &manifold) {
    const double totalInverseMass = bodyA->inverseMass + bodyB->inverseMass;
    if (totalInverseMass == 0) {
        return;
    }

    const vector2d leverArmA = subtractVector(manifold.contactPoint, bodyA->pos);
    const vector2d leverArmB = subtractVector(manifold.contactPoint, bodyB->pos);

    vector2d velocityAtContactA =
        addVector(bodyA->linearVel, crossScalarVector(bodyA->angularVel, leverArmA));
    vector2d velocityAtContactB =
        addVector(bodyB->linearVel, crossScalarVector(bodyB->angularVel, leverArmB));

    vector2d relativeVelocity = subtractVector(velocityAtContactB, velocityAtContactA);
    const double velocityAlongNormal = dotProduct(relativeVelocity, manifold.normal);

    if (velocityAlongNormal > 0) {
        return;
    }

    double restitution = std::min(bodyA->collider ? bodyA->collider->restitution : 0.0,
                                  bodyB->collider ? bodyB->collider->restitution : 0.0);
    if (std::abs(velocityAlongNormal) < REST_VELOCITY_THRESHOLD) {
        restitution = 0.0;
    }

    const double leverArmCrossNormalA = crossVectors(leverArmA, manifold.normal);
    const double leverArmCrossNormalB = crossVectors(leverArmB, manifold.normal);
    const double angularComponentA =
        leverArmCrossNormalA * leverArmCrossNormalA * bodyA->inverseMomentOfInertia;
    const double angularComponentB =
        leverArmCrossNormalB * leverArmCrossNormalB * bodyB->inverseMomentOfInertia;

    const double impulseDenominator = totalInverseMass + angularComponentA + angularComponentB;
    const double normalImpulseMagnitude =
        -(1.0 + restitution) * velocityAlongNormal / impulseDenominator;

    const vector2d impulseVector = scalarMultiplyVector(manifold.normal, normalImpulseMagnitude);

    bodyA->linearVel = subtractVector(bodyA->linearVel,
        scalarMultiplyVector(impulseVector, bodyA->inverseMass));
    bodyA->angularVel -= leverArmCrossNormalA * normalImpulseMagnitude * bodyA->inverseMomentOfInertia;

    bodyB->linearVel = addVector(bodyB->linearVel,
        scalarMultiplyVector(impulseVector, bodyB->inverseMass));
    bodyB->angularVel += leverArmCrossNormalB * normalImpulseMagnitude * bodyB->inverseMomentOfInertia;

    velocityAtContactA =
        addVector(bodyA->linearVel, crossScalarVector(bodyA->angularVel, leverArmA));
    velocityAtContactB =
        addVector(bodyB->linearVel, crossScalarVector(bodyB->angularVel, leverArmB));
    relativeVelocity = subtractVector(velocityAtContactB, velocityAtContactA);

    vector2d tangent = subtractVector(relativeVelocity,
        scalarMultiplyVector(manifold.normal, dotProduct(relativeVelocity, manifold.normal)));
    const double tangentMagnitude = sqrt(dotProduct(tangent, tangent));

    if (tangentMagnitude > 0.0001) {
        tangent = vector2d{tangent.x / tangentMagnitude, tangent.y / tangentMagnitude};

        const double leverArmCrossTangentA = crossVectors(leverArmA, tangent);
        const double leverArmCrossTangentB = crossVectors(leverArmB, tangent);
        const double frictionDenominator = totalInverseMass +
            (leverArmCrossTangentA * leverArmCrossTangentA * bodyA->inverseMomentOfInertia) +
            (leverArmCrossTangentB * leverArmCrossTangentB * bodyB->inverseMomentOfInertia);

        double tangentImpulseMagnitude =
            -dotProduct(relativeVelocity, tangent) / frictionDenominator;
        const double frictionCoefficient = std::min(
            bodyA->collider ? bodyA->collider->friction : 0.0,
            bodyB->collider ? bodyB->collider->friction : 0.0);

        if (std::abs(tangentImpulseMagnitude) > normalImpulseMagnitude * frictionCoefficient) {
            tangentImpulseMagnitude = (tangentImpulseMagnitude > 0 ? 1.0 : -1.0) *
                                      normalImpulseMagnitude * frictionCoefficient;
        }

        const vector2d frictionImpulse = scalarMultiplyVector(tangent, tangentImpulseMagnitude);
        bodyA->linearVel = subtractVector(bodyA->linearVel,
            scalarMultiplyVector(frictionImpulse, bodyA->inverseMass));
        bodyA->angularVel -= leverArmCrossTangentA * tangentImpulseMagnitude *
                             bodyA->inverseMomentOfInertia;

        bodyB->linearVel = addVector(bodyB->linearVel,
            scalarMultiplyVector(frictionImpulse, bodyB->inverseMass));
        bodyB->angularVel += leverArmCrossTangentB * tangentImpulseMagnitude *
                             bodyB->inverseMomentOfInertia;
    }

    if (manifold.penetration > COLLISION_SLOP) {
        const double correctionScalar = (manifold.penetration - COLLISION_SLOP) /
                                        totalInverseMass * POSITION_CORRECTION_PERCENT;
        const vector2d correctionVector = scalarMultiplyVector(manifold.normal, correctionScalar);

        const vector2d positionCorrectionA =
            scalarMultiplyVector(correctionVector, bodyA->inverseMass);
        const vector2d positionCorrectionB =
            scalarMultiplyVector(correctionVector, bodyB->inverseMass);

        bodyA->pos = subtractVector(bodyA->pos, positionCorrectionA);
        bodyB->pos = addVector(bodyB->pos, positionCorrectionB);

        bodyA->angle -= crossVectors(leverArmA, positionCorrectionA) * bodyA->inverseMomentOfInertia;
        bodyB->angle += crossVectors(leverArmB, positionCorrectionB) * bodyB->inverseMomentOfInertia;
    }
}
