#include <cfloat>
#include <iostream>
#include <random>
using namespace std;
#include "config.h"

struct vector2d {
    double x, y;
};

vector2d negateVector(const vector2d v) {
    return vector2d{-v.x, -v.y};
}

vector2d addVector(const vector2d v1, const vector2d v2) {
    return vector2d{v1.x + v2.x, v1.y + v2.y};
}

vector2d subtractVector(const vector2d v1, const vector2d v2) {
    return vector2d{v1.x - v2.x, v1.y - v2.y};
}

vector2d scalarMultiplyVector(const vector2d v, const double s) {
    return vector2d{v.x * s, v.y * s};
}

double dotProduct(const vector2d v1, const vector2d v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

struct Particle2d {
    vector2d pos;
    vector2d vel;
    double mass; // in kilograms
};


struct Box2d {
    double width, height, mass, momentOfInertia;
};



struct RigidBody {
    vector2d pos;
    vector2d linearVel;
    double angle;
    double angularVel;
    vector2d force;
    double torque;
    Box2d shape;
};

struct AABB {
    //axis-aligned bounding boxes
    vector2d min; //min is the bottom left part of the bounding box
    vector2d max; //max is the upper right part of the bounding box
};

struct Circle {
    double centerX;
    double centerY;
    double radius;
};

Particle2d particles[PARTICLE_COUNT];
RigidBody rigidBodies[RIGIDBODY_COUNT];

//PARTICLE METHODS:
void drawParticles() {
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        const auto &particle = particles[i];
        cout << "Particle" << i << " (" << particle.pos.x << ", " << particle.pos.y << ")" << endl;
    }
}

vector2d computeForce(const Particle2d &particle) {
    return vector2d{0, particle.mass * -GRAVITY};
}

void initParticles() {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> randPos(0, 50);
    for (auto &particle: particles) {
        particle.mass = 1;
        particle.vel = vector2d{0, 0};
        particle.pos = vector2d{randPos(mt), randPos(mt)};
    }
}

void runParticleSim() {
    double totalTimeElapsed = 0;
    initParticles();
    drawParticles();
    while (totalTimeElapsed < TOTAL_SIM_TIME) {
        double deltaTime = 1;
        for (auto &particle: particles) {
            vector2d force = computeForce(particle);
            auto accel = vector2d{force.x / particle.mass, force.y / particle.mass};
            particle.vel.x += accel.x * deltaTime;
            particle.vel.y += accel.y * deltaTime;
            particle.pos.x += particle.vel.x * deltaTime;
            particle.pos.y += particle.vel.y * deltaTime;
        }
        drawParticles();
        totalTimeElapsed += deltaTime;
    }
}

//RIGIDBODY METHODS
void drawRigidBodies() {
    for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
        RigidBody &rigidBody = rigidBodies[i];
        cout << "body" << i << " (" << rigidBody.pos.x << ", " << rigidBody.pos.y << ")" << ", " << "angle = " <<
                rigidBody.
                angle << endl;
    }
}


void calculateBoxIntertia(Box2d *box) {
    const auto m = box->mass;
    const auto w = box->width;
    const auto h = box->height;
    box->momentOfInertia = m * (pow(w, 2) + pow(h, 2)) / 12;
}


void initRigidBodies() {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> randPos(0, 50);
    std::uniform_real_distribution<double> randAng(0, 2 * M_PI);
    std::uniform_real_distribution<double> randLength(1, 2);
    for (auto &rigidBody: rigidBodies) {
        rigidBody.pos = vector2d{randPos(mt), randPos(mt)};
        rigidBody.angle = randAng(mt);
        rigidBody.linearVel = vector2d{0, 0};
        rigidBody.angularVel = 0;

        rigidBody.shape.mass = 10;
        rigidBody.shape.width = 1 + randLength(mt);
        rigidBody.shape.height = 1 + randLength(mt);
        calculateBoxIntertia(&rigidBody.shape);
    }
}

void computeForceAndTorque(RigidBody *rigidBody) {
    auto force = vector2d{0, 100};
    rigidBody->force = force;
    auto armVector = vector2d{rigidBody->shape.width / 2, rigidBody->shape.height / 2};
    rigidBody->torque = armVector.x * force.y - armVector.y * force.x;
}

void runRigidbodySim() {
    double totalTimeElapsed = 0;
    double deltaTime = 1;
    initRigidBodies();
    drawRigidBodies();
    while (totalTimeElapsed < TOTAL_SIM_TIME) {
        for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
            RigidBody *rigidBody = &rigidBodies[i];
            computeForceAndTorque(rigidBody);
            vector2d linearAccel = vector2d{
                rigidBody->force.x / rigidBody->shape.mass, rigidBody->force.y / rigidBody->shape.mass
            };
            rigidBody->linearVel.x += linearAccel.x * deltaTime;
            rigidBody->linearVel.y += linearAccel.y * deltaTime;
            rigidBody->pos.x += rigidBody->linearVel.x * deltaTime;
            rigidBody->pos.y += rigidBody->linearVel.y * deltaTime;
            double angularAccel = rigidBody->torque / rigidBody->shape.momentOfInertia;
            rigidBody->angularVel += angularAccel * deltaTime;
            rigidBody->angle += rigidBody->angularVel * deltaTime;
        }
        drawRigidBodies();
        totalTimeElapsed += deltaTime;
    }
}

//BOUNDING BOX METHODS
bool testAABBIntersection(const AABB *a, const AABB *b) {
    const double d1x = b->min.x - a->max.x;
    const double d1y = b->min.y - a->max.y;
    const double d2x = a->min.x - b->max.x;
    const double d2y = a->min.y - b->max.y;
    if (d1x > 0 || d1y > 0) return false;
    if (d2x > 0 || d2y > 0) return false;
    return true;
}

vector2d getSupportPoint(const vector2d *vertices, const int count, const vector2d dir) {
    double highest = -DBL_MAX;
    auto support = vector2d{0, 0};
    for (int i = 0; i < count; ++i) {
        const vector2d v = vertices[i];
        if (const double dot = dotProduct(v, dir); dot > highest) {
            highest = dot;
            support = v;
        }
    }
    return support;
}

//CIRCLE METHODS
bool CirclesCollisionCheck(Circle *A, Circle *B) {
    double x = A->centerX - B->centerX;
    double y = A->centerY - B->centerY;
    double centerDistSquare = pow(x, 2) + pow(y, 2);
    double radius = A->radius + B->radius;
    return centerDistSquare <= pow(radius, 2);
}

int main() {
}
