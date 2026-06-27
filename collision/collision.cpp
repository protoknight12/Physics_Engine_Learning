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

double speedSquared(const vector2d &velocity) {
    return dotProduct(velocity, velocity);
}

bool isVerticalRestingContact(const CollisionManifold &manifold,
                              const double velocityAlongNormal) {
    return std::abs(velocityAlongNormal) <= RESTING_CONTACT_VELOCITY &&
           std::abs(manifold.normal.y) > RESTING_CONTACT_NORMAL_Y;
}

bool shouldLockRestingRotation(const RigidBody *body, const bool isRestingContact) {
    return isRestingContact &&
           body->inverseMass > 0.0 &&
           speedSquared(body->linearVel) <= WAKE_LINEAR_THRESHOLD_SQ &&
           std::abs(body->angularVel) <= RESTING_ANGULAR_LOCK_THRESHOLD;
}

void markRestingSupport(RigidBody *bodyA,
                        RigidBody *bodyB,
                        const CollisionManifold &manifold,
                        const double velocityAlongNormal) {
    if (!isVerticalRestingContact(manifold, velocityAlongNormal)) {
        return;
    }

    if (bodyA->inverseMass > 0.0 && manifold.normal.y > RESTING_CONTACT_NORMAL_Y) {
        bodyA->hasRestingContact = true;
    }
    if (bodyB->inverseMass > 0.0 && manifold.normal.y < -RESTING_CONTACT_NORMAL_Y) {
        bodyB->hasRestingContact = true;
    }
}

void wakeForImpact(RigidBody *bodyA,
                   RigidBody *bodyB,
                   const double velocityAlongNormal) {
    const bool meaningfulImpact =
        std::abs(velocityAlongNormal) > RESTING_CONTACT_VELOCITY ||
        speedSquared(bodyA->linearVel) > WAKE_LINEAR_THRESHOLD_SQ ||
        speedSquared(bodyB->linearVel) > WAKE_LINEAR_THRESHOLD_SQ ||
        std::abs(bodyA->angularVel) > WAKE_ANGULAR_THRESHOLD ||
        std::abs(bodyB->angularVel) > WAKE_ANGULAR_THRESHOLD;

    if (!meaningfulImpact) {
        return;
    }

    if (bodyA->inverseMass > 0.0) {
        bodyA->isSleeping = false;
        bodyA->sleepFrameCount = 0;
    }
    if (bodyB->inverseMass > 0.0) {
        bodyB->isSleeping = false;
        bodyB->sleepFrameCount = 0;
    }
}
} // namespace

double calculateBoxInertia(const double mass, const double width, const double height) {
    return (mass * (width * width + height * height)) / 12.0;
}

double calculateCircleInertia(const double mass, const double radius) {
    return (mass * radius * radius) / 2.0;
}

double calculatePolygonInertia(const double mass,
                               const vector2d *localVertices,
                               const int vertexCount) {
    double numerator = 0.0;
    double denominator = 0.0;

    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        const int nextIndex = (vertexIndex + 1) % vertexCount;
        const double crossValue = std::abs(crossVectors(localVertices[vertexIndex],
                                                        localVertices[nextIndex]));
        const double vertexTerm =
            dotProduct(localVertices[vertexIndex], localVertices[vertexIndex]) +
            dotProduct(localVertices[vertexIndex], localVertices[nextIndex]) +
            dotProduct(localVertices[nextIndex], localVertices[nextIndex]);
        numerator += vertexTerm * crossValue;
        denominator += crossValue;
    }

    if (denominator == 0.0) {
        return mass;
    }

    return mass * numerator / (6.0 * denominator);
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

bool getPolygonWorldVertices(const RigidBody *body,
                             vector2d *vertices,
                             int &vertexCount) {
    const PolygonCollider *polygonCollider = asPolygonCollider(body->collider);
    if (!polygonCollider || polygonCollider->vertexCount < 3) {
        vertexCount = 0;
        return false;
    }

    vertexCount = polygonCollider->vertexCount;
    const double cosAngle = cos(body->angle);
    const double sinAngle = sin(body->angle);

    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        const vector2d &localVertex = polygonCollider->localVertices[vertexIndex];
        vertices[vertexIndex].x = body->pos.x +
            (localVertex.x * cosAngle - localVertex.y * sinAngle);
        vertices[vertexIndex].y = body->pos.y +
            (localVertex.x * sinAngle + localVertex.y * cosAngle);
    }

    return true;
}

namespace {
bool isPointInConvexPolygon(const vector2d &point,
                            const vector2d *vertices,
                            const int vertexCount) {
    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        const int nextIndex = (vertexIndex + 1) % vertexCount;
        const vector2d edge = subtractVector(vertices[nextIndex], vertices[vertexIndex]);
        const vector2d toPoint = subtractVector(point, vertices[vertexIndex]);
        if (crossVectors(edge, toPoint) < 0.0) {
            return false;
        }
    }
    return true;
}
} // namespace

bool isPointInRigidBody(const vector2d &point, const RigidBody &body) {
    if (!body.collider) {
        return false;
    }

    if (const CircleCollider *circleCollider = asCircleCollider(body.collider)) {
        const double deltaX = point.x - body.pos.x;
        const double deltaY = point.y - body.pos.y;
        const double radius = circleCollider->radius;
        return (deltaX * deltaX) + (deltaY * deltaY) <= radius * radius;
    }

    if (const BoxCollider *boxCollider = asBoxCollider(body.collider)) {
        const double deltaX = point.x - body.pos.x;
        const double deltaY = point.y - body.pos.y;
        const double cosAngle = cos(-body.angle);
        const double sinAngle = sin(-body.angle);
        const double localX = deltaX * cosAngle - deltaY * sinAngle;
        const double localY = deltaX * sinAngle + deltaY * cosAngle;

        return std::abs(localX) <= boxCollider->width / 2.0 &&
               std::abs(localY) <= boxCollider->height / 2.0;
    }

    if (const PolygonCollider *polygonCollider = asPolygonCollider(body.collider)) {
        const double deltaX = point.x - body.pos.x;
        const double deltaY = point.y - body.pos.y;
        const double cosAngle = cos(-body.angle);
        const double sinAngle = sin(-body.angle);
        const vector2d localPoint = {
            deltaX * cosAngle - deltaY * sinAngle,
            deltaX * sinAngle + deltaY * cosAngle
        };
        return isPointInConvexPolygon(localPoint,
                                      polygonCollider->localVertices,
                                      polygonCollider->vertexCount);
    }

    return false;
}

namespace {
void projectVertices(const vector2d *vertices,
                     const int vertexCount,
                     const vector2d axis,
                     double &minProjection,
                     double &maxProjection) {
    minProjection = dotProduct(vertices[0], axis);
    maxProjection = minProjection;
    for (int vertexIndex = 1; vertexIndex < vertexCount; ++vertexIndex) {
        const double projection = dotProduct(vertices[vertexIndex], axis);
        if (projection < minProjection) {
            minProjection = projection;
        }
        if (projection > maxProjection) {
            maxProjection = projection;
        }
    }
}

vector2d closestPointOnSegment(const vector2d &point,
                               const vector2d &segmentStart,
                               const vector2d &segmentEnd) {
    const vector2d segment = subtractVector(segmentEnd, segmentStart);
    const double segmentLengthSquared = dotProduct(segment, segment);
    if (segmentLengthSquared == 0.0) {
        return segmentStart;
    }

    double projection = dotProduct(subtractVector(point, segmentStart), segment) /
                        segmentLengthSquared;
    projection = std::max(0.0, std::min(1.0, projection));
    return addVector(segmentStart, scalarMultiplyVector(segment, projection));
}

CollisionManifold satConvexVsConvex(const vector2d *verticesA,
                                    const int vertexCountA,
                                    const vector2d *verticesB,
                                    const int vertexCountB,
                                    const vector2d &centerA,
                                    const vector2d &centerB) {
    CollisionManifold manifold{};
    manifold.penetration = DBL_MAX;

    auto testAxes = [&](const vector2d *vertices, const int vertexCount) {
        for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            const int nextIndex = (vertexIndex + 1) % vertexCount;
            const vector2d edge =
                subtractVector(vertices[nextIndex], vertices[vertexIndex]);
            const vector2d axis = normalizeVector(vector2d{-edge.y, edge.x});
            if (!isValidAxis(axis)) {
                continue;
            }

            double minProjectionA, maxProjectionA, minProjectionB, maxProjectionB;
            projectVertices(verticesA, vertexCountA, axis, minProjectionA, maxProjectionA);
            projectVertices(verticesB, vertexCountB, axis, minProjectionB, maxProjectionB);

            if (maxProjectionA < minProjectionB || maxProjectionB < minProjectionA) {
                manifold.penetration = DBL_MAX;
                return;
            }

            const double overlap = std::min(maxProjectionA, maxProjectionB) -
                                   std::max(minProjectionA, minProjectionB);
            if (overlap < manifold.penetration) {
                manifold.penetration = overlap;
                manifold.normal = axis;
            }
        }
    };

    testAxes(verticesA, vertexCountA);
    if (manifold.penetration == DBL_MAX) {
        return manifold;
    }
    testAxes(verticesB, vertexCountB);
    if (manifold.penetration == DBL_MAX) {
        return manifold;
    }

    manifold.collision = true;

    const vector2d direction = subtractVector(centerB, centerA);
    if (dotProduct(manifold.normal, direction) < 0) {
        manifold.normal = negateVector(manifold.normal);
    }

    manifold.contactPoint = scalarMultiplyVector(addVector(centerA, centerB), 0.5);
    return manifold;
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
        projectVertices(verticesA, 4, axis, minProjectionA, maxProjectionA);
        projectVertices(verticesB, 4, axis, minProjectionB, maxProjectionB);

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

CollisionManifold circleVsPolygon(RigidBody *circleBody, RigidBody *polygonBody) {
    CollisionManifold manifold{};

    const CircleCollider *circleCollider = asCircleCollider(circleBody->collider);
    vector2d polygonVertices[MAX_POLYGON_VERTICES];
    int vertexCount = 0;
    if (!circleCollider ||
        !getPolygonWorldVertices(polygonBody, polygonVertices, vertexCount)) {
        return manifold;
    }

    const vector2d &circleCenter = circleBody->pos;
    vector2d closestPoint = polygonVertices[0];
    double closestDistanceSquared = dotProduct(subtractVector(circleCenter, closestPoint),
                                               subtractVector(circleCenter, closestPoint));

    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        const int nextIndex = (vertexIndex + 1) % vertexCount;
        const vector2d segmentClosest = closestPointOnSegment(
            circleCenter,
            polygonVertices[vertexIndex],
            polygonVertices[nextIndex]);
        const vector2d delta = subtractVector(circleCenter, segmentClosest);
        const double distanceSquared = dotProduct(delta, delta);
        if (distanceSquared < closestDistanceSquared) {
            closestDistanceSquared = distanceSquared;
            closestPoint = segmentClosest;
        }
    }

    const double radius = circleCollider->radius;
    if (closestDistanceSquared >= radius * radius &&
        !isPointInConvexPolygon(circleCenter, polygonVertices, vertexCount)) {
        return manifold;
    }

    manifold.collision = true;
    manifold.contactPoint = closestPoint;

    if (closestDistanceSquared > 0.0) {
        const double distance = sqrt(closestDistanceSquared);
        manifold.penetration = radius - distance;
        manifold.normal = scalarMultiplyVector(subtractVector(circleCenter, closestPoint),
                                               1.0 / distance);
    } else {
        double minimumPenetration = DBL_MAX;
        for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            const int nextIndex = (vertexIndex + 1) % vertexCount;
            const vector2d edge =
                subtractVector(polygonVertices[nextIndex], polygonVertices[vertexIndex]);
            const vector2d inwardNormal = normalizeVector(vector2d{-edge.y, edge.x});
            const double penetrationDepth = dotProduct(
                subtractVector(polygonVertices[vertexIndex], circleCenter),
                inwardNormal);
            if (penetrationDepth < minimumPenetration) {
                minimumPenetration = penetrationDepth;
                manifold.normal = inwardNormal;
            }
        }
        manifold.penetration = radius + minimumPenetration;
    }

    return manifold;
}

CollisionManifold polygonVsCircle(RigidBody *polygonBody, RigidBody *circleBody) {
    CollisionManifold manifold = circleVsPolygon(circleBody, polygonBody);
    manifold.normal = negateVector(manifold.normal);
    return manifold;
}

CollisionManifold boxVsPolygon(RigidBody *boxBody, RigidBody *polygonBody) {
    vector2d boxVertices[4];
    vector2d polygonVertices[MAX_POLYGON_VERTICES];
    int polygonVertexCount = 0;
    if (!getBoxVertices(boxBody, boxVertices) ||
        !getPolygonWorldVertices(polygonBody, polygonVertices, polygonVertexCount)) {
        return CollisionManifold{};
    }

    return satConvexVsConvex(boxVertices,
                           4,
                           polygonVertices,
                           polygonVertexCount,
                           boxBody->pos,
                           polygonBody->pos);
}

CollisionManifold polygonVsBox(RigidBody *polygonBody, RigidBody *boxBody) {
    CollisionManifold manifold = boxVsPolygon(boxBody, polygonBody);
    manifold.normal = negateVector(manifold.normal);
    return manifold;
}

CollisionManifold polygonVsPolygon(RigidBody *bodyA, RigidBody *bodyB) {
    vector2d verticesA[MAX_POLYGON_VERTICES];
    vector2d verticesB[MAX_POLYGON_VERTICES];
    int vertexCountA = 0;
    int vertexCountB = 0;
    if (!getPolygonWorldVertices(bodyA, verticesA, vertexCountA) ||
        !getPolygonWorldVertices(bodyB, verticesB, vertexCountB)) {
        return CollisionManifold{};
    }

    return satConvexVsConvex(verticesA,
                             vertexCountA,
                             verticesB,
                             vertexCountB,
                             bodyA->pos,
                             bodyB->pos);
}

CollisionHandler CollisionMatrix[SHAPE_TYPE_COUNT][SHAPE_TYPE_COUNT] = {
    {circleVsCircle, circleVsBox, circleVsPolygon},
    {boxVsCircle, boxVsBox, boxVsPolygon},
    {polygonVsCircle, polygonVsBox, polygonVsPolygon}
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
    const bool restingContact = isVerticalRestingContact(manifold, velocityAlongNormal);
    const bool lockRestingRotationA = shouldLockRestingRotation(bodyA, restingContact);
    const bool lockRestingRotationB = shouldLockRestingRotation(bodyB, restingContact);

    markRestingSupport(bodyA, bodyB, manifold, velocityAlongNormal);
    wakeForImpact(bodyA, bodyB, velocityAlongNormal);

    if (lockRestingRotationA) {
        bodyA->angularVel = 0.0;
    }
    if (lockRestingRotationB) {
        bodyB->angularVel = 0.0;
    }

    if (bodyA->isSleeping &&
        bodyB->isSleeping &&
        std::abs(velocityAlongNormal) <= RESTING_CONTACT_VELOCITY &&
        manifold.penetration <= SLEEP_POSITION_SLOP) {
        return;
    }

    if (velocityAlongNormal > 0) {
        return;
    }

    double restitution = std::min(bodyA->collider ? bodyA->collider->restitution : 0.0,
                                  bodyB->collider ? bodyB->collider->restitution : 0.0);
    if (bodyA->inverseMass == 0.0 || bodyB->inverseMass == 0.0) {
        restitution = 0.0;
    } else if (std::abs(velocityAlongNormal) < REST_VELOCITY_THRESHOLD) {
        restitution = 0.0;
    }

    const double leverArmCrossNormalA =
        lockRestingRotationA ? 0.0 : crossVectors(leverArmA, manifold.normal);
    const double leverArmCrossNormalB =
        lockRestingRotationB ? 0.0 : crossVectors(leverArmB, manifold.normal);
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
    if (!lockRestingRotationA) {
        bodyA->angularVel -=
            leverArmCrossNormalA * normalImpulseMagnitude * bodyA->inverseMomentOfInertia;
    }

    bodyB->linearVel = addVector(bodyB->linearVel,
        scalarMultiplyVector(impulseVector, bodyB->inverseMass));
    if (!lockRestingRotationB) {
        bodyB->angularVel +=
            leverArmCrossNormalB * normalImpulseMagnitude * bodyB->inverseMomentOfInertia;
    }

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

        const double leverArmCrossTangentA =
            lockRestingRotationA ? 0.0 : crossVectors(leverArmA, tangent);
        const double leverArmCrossTangentB =
            lockRestingRotationB ? 0.0 : crossVectors(leverArmB, tangent);
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
        if (!lockRestingRotationA) {
            bodyA->angularVel -= leverArmCrossTangentA * tangentImpulseMagnitude *
                                 bodyA->inverseMomentOfInertia;
        }

        bodyB->linearVel = addVector(bodyB->linearVel,
            scalarMultiplyVector(frictionImpulse, bodyB->inverseMass));
        if (!lockRestingRotationB) {
            bodyB->angularVel += leverArmCrossTangentB * tangentImpulseMagnitude *
                                 bodyB->inverseMomentOfInertia;
        }
    }

    if (manifold.penetration > COLLISION_SLOP) {
        const bool hasStaticBody = bodyA->inverseMass == 0.0 || bodyB->inverseMass == 0.0;
        const double correctionPercent =
            hasStaticBody ? 1.0 : POSITION_CORRECTION_PERCENT;
        const double correctionScalar = (manifold.penetration - COLLISION_SLOP) /
                                        totalInverseMass * correctionPercent;
        const vector2d correctionVector = scalarMultiplyVector(manifold.normal, correctionScalar);

        const vector2d positionCorrectionA =
            scalarMultiplyVector(correctionVector, bodyA->inverseMass);
        const vector2d positionCorrectionB =
            scalarMultiplyVector(correctionVector, bodyB->inverseMass);

        bodyA->pos = subtractVector(bodyA->pos, positionCorrectionA);
        bodyB->pos = addVector(bodyB->pos, positionCorrectionB);
    }
}
