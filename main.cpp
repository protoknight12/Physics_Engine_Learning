#define USE_MATH_DEFINES
#include <cmath>

#include "config.h"
#include "raylib.h"
#include "collider/collider.h"
#include "simulation/simulation.h"

int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "RigidBody Physics Core (SAT & Iteration Enabled)");
    SetTargetFPS(GetMonitorRefreshRate(0));

    initRigidBodies();

    const double deltaTime = 0.01666;
    double timeAccumulator = 0.0;
    const int impulseIterations = 6;

    while (!WindowShouldClose()) {
        double frameDeltaTime = GetFrameTime();
        if (frameDeltaTime > 0.25) {
            frameDeltaTime = 0.25;
        }
        timeAccumulator += frameDeltaTime;

        while (timeAccumulator >= deltaTime) {
            integrateBodies(deltaTime);
            solveCollisions(impulseIterations);
            screenBoundaryConstraints(screenWidth, screenHeight);
            timeAccumulator -= deltaTime;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("Physics Engine: SAT ORIENTED COLLIDERS ACTIVE", 15, 15, 16, DARKGRAY);
        DrawFPS(screenWidth - 100, 15);

        for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
            RigidBody &body = rigidBodies[bodyIndex];
            if (!body.collider) {
                continue;
            }

            const Vector2 raylibPosition = {
                static_cast<float>(body.pos.x),
                static_cast<float>(body.pos.y)
            };

            if (const CircleCollider *circleCollider = asCircleCollider(body.collider)) {
                DrawCircleV(raylibPosition, static_cast<float>(circleCollider->radius), PURPLE);

                const Vector2 rotationLineEnd = {
                    raylibPosition.x + static_cast<float>(cos(body.angle) * circleCollider->radius),
                    raylibPosition.y + static_cast<float>(sin(body.angle) * circleCollider->radius)
                };
                DrawLineV(raylibPosition, rotationLineEnd, GOLD);
            } else if (const BoxCollider *boxCollider = asBoxCollider(body.collider)) {
                const float boxWidth = static_cast<float>(boxCollider->width);
                const float boxHeight = static_cast<float>(boxCollider->height);

                const Rectangle rectangle = {raylibPosition.x, raylibPosition.y, boxWidth, boxHeight};
                const Vector2 origin = {boxWidth / 2.0f, boxHeight / 2.0f};
                const float rotationDegrees = static_cast<float>(body.angle * (180.0 / M_PI));

                DrawRectanglePro(rectangle, origin, rotationDegrees, PURPLE);
            }
        }

        EndDrawing();
    }

    for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
        delete rigidBodies[bodyIndex].collider;
        rigidBodies[bodyIndex].collider = nullptr;
    }

    CloseWindow();
    return 0;
}
