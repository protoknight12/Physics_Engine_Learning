#include <iostream>
#include <random>
#include "vector2d/vector2d.h"
using namespace std;
#include "config.h"


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

Particle2d particles[PARTICLE_COUNT];
RigidBody rigidBodies[RIGIDBODY_COUNT];

void drawParticles() {
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
        const auto &particle = particles[i];
        cout << "Particle" << i << " (" << particle.pos.x << ", " << particle.pos.y << ")" << endl;
    }
}

void drawRigidBodies() {
    for (int i = 0; i < RIGIDBODY_COUNT; ++i) {
        RigidBody &rigidBody = rigidBodies[i];
        cout << "body" << i << " (" << rigidBody.pos.x << ", " << rigidBody.pos.y << ")" << ", " << "angle = " <<
                rigidBody.
                angle << endl;
    }
}


vector2d computeForce(const Particle2d &particle) {
    return vector2d{0, particle.mass * -GRAVITY};
}

void calculateBoxIntertia(Box2d *box) {
    const auto m = box->mass;
    const auto w = box->width;
    const auto h = box->height;
    box->momentOfInertia = m * (pow(w, 2) + pow(h, 2)) / 2;
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
        rigidBody.shape.width = 1 + randLength(mt);
        calculateBoxIntertia(&rigidBody.shape);
    }
}

void computeForceAndTorque(RigidBody *rigidBody) {
    auto force = vector2d{0, 100};
    rigidBody->force = force;
    auto armVector = vector2d{rigidBody->shape.width / 2, rigidBody->shape.height / 2};
    rigidBody->torque = armVector.x * force.y + armVector.y * force.x;
}

void runRigidbodySim() {
    double totalSimTime = 10;
    double totalTimeElapsed = 0;
    double deltaTime = 1;
    initRigidBodies();
    drawRigidBodies();
    while (totalTimeElapsed < totalSimTime) {
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

void runParticleSim() {
    double totalSimulationTime = 10; //in seconds
    double totalTimeElapsed = 0;
    initParticles();
    drawParticles();
    while (totalTimeElapsed < totalSimulationTime) {
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

int main() {
    runParticleSim();
    runRigidbodySim();
}
