#include "collider.h"

CircleCollider *asCircleCollider(Collider *collider) {
    if (!collider || collider->type != ShapeType::CIRCLE) {
        return nullptr;
    }
    return static_cast<CircleCollider *>(collider);
}

const CircleCollider *asCircleCollider(const Collider *collider) {
    if (!collider || collider->type != ShapeType::CIRCLE) {
        return nullptr;
    }
    return static_cast<const CircleCollider *>(collider);
}

BoxCollider *asBoxCollider(Collider *collider) {
    if (!collider || collider->type != ShapeType::BOX) {
        return nullptr;
    }
    return static_cast<BoxCollider *>(collider);
}

const BoxCollider *asBoxCollider(const Collider *collider) {
    if (!collider || collider->type != ShapeType::BOX) {
        return nullptr;
    }
    return static_cast<const BoxCollider *>(collider);
}
