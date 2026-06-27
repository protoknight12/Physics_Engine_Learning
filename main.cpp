#define USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

#include "config.h"
#include "raylib.h"
#include "collider/collider.h"
#include "collision/collision.h"
#include "simulation/simulation.h"
#include "vector2d/vector2d.h"

namespace {
enum class EditorTool {
    SLINGSHOT,
    SPAWN_CIRCLE,
    SPAWN_BOX,
    SPAWN_POLYGON,
    DELETE_BODY
};

constexpr double SLINGSHOT_FORCE_SCALE = 4.0;
constexpr double MIN_SLINGSHOT_LENGTH = 8.0;
constexpr double TRAJECTORY_TIME_STEP = 0.033;
constexpr int MAX_TRAJECTORY_STEPS = 150;
constexpr double SPAWN_CIRCLE_RADIUS = 25.0;
constexpr double SPAWN_BOX_WIDTH = 50.0;
constexpr double SPAWN_BOX_HEIGHT = 50.0;
constexpr double SPAWN_POLYGON_RADIUS = 28.0;
constexpr double MIN_EDITOR_SIZE = 8.0;
constexpr double MAX_EDITOR_SIZE = 160.0;
constexpr double SIZE_STEP = 5.0;
constexpr int FALLBACK_REFRESH_RATE = 60;
constexpr int MAX_PHYSICS_STEPS_PER_FRAME = 4;

constexpr BodyColor COLOR_PALETTE[] = {
    {128, 0, 128, 255},
    {70, 130, 180, 255},
    {46, 139, 87, 255},
    {220, 20, 60, 255},
    {255, 140, 0, 255},
    {30, 30, 30, 255}
};

constexpr const char *COLOR_NAMES[] = {
    "Purple",
    "Blue",
    "Green",
    "Red",
    "Orange",
    "Charcoal"
};

constexpr int COLOR_PALETTE_COUNT = sizeof(COLOR_PALETTE) / sizeof(COLOR_PALETTE[0]);

struct EditorSettings {
    double circleRadius = SPAWN_CIRCLE_RADIUS;
    double boxWidth = SPAWN_BOX_WIDTH;
    double boxHeight = SPAWN_BOX_HEIGHT;
    double polygonRadius = SPAWN_POLYGON_RADIUS;
    int polygonSides = DEFAULT_POLYGON_SIDES;
    int colorIndex = 0;
};

const char *toolName(const EditorTool tool) {
    switch (tool) {
        case EditorTool::SLINGSHOT: return "Slingshot";
        case EditorTool::SPAWN_CIRCLE: return "Spawn Circle";
        case EditorTool::SPAWN_BOX: return "Spawn Box";
        case EditorTool::SPAWN_POLYGON: return "Spawn Polygon";
        case EditorTool::DELETE_BODY: return "Delete";
    }
    return "Unknown";
}

int getDisplayRefreshRate() {
    const int refreshRate = GetMonitorRefreshRate(GetCurrentMonitor());
    return refreshRate > 0 ? refreshRate : FALLBACK_REFRESH_RATE;
}

double clampEditorSize(const double value) {
    return std::max(MIN_EDITOR_SIZE, std::min(MAX_EDITOR_SIZE, value));
}

int clampPolygonSides(const int sides) {
    return std::max(3, std::min(MAX_POLYGON_VERTICES, sides));
}

BodyColor getSelectedBodyColor(const EditorSettings &settings) {
    return COLOR_PALETTE[settings.colorIndex];
}

const char *getSelectedColorName(const EditorSettings &settings) {
    return COLOR_NAMES[settings.colorIndex];
}

Color toRaylibColor(const BodyColor color) {
    return Color{color.r, color.g, color.b, color.a};
}

double getPolygonRadius(const PolygonCollider &polygonCollider) {
    double radius = 0.0;
    for (int vertexIndex = 0; vertexIndex < polygonCollider.vertexCount; ++vertexIndex) {
        const vector2d &vertex = polygonCollider.localVertices[vertexIndex];
        const double distance = sqrt(dotProduct(vertex, vertex));
        if (distance > radius) {
            radius = distance;
        }
    }
    return radius;
}

bool tryComputeSlingshotVelocity(const vector2d &anchorPosition,
                                 const vector2d &mousePosition,
                                 vector2d &launchVelocity) {
    const vector2d dragVector = subtractVector(mousePosition, anchorPosition);
    const double dragLengthSquared = dotProduct(dragVector, dragVector);
    if (dragLengthSquared < MIN_SLINGSHOT_LENGTH * MIN_SLINGSHOT_LENGTH) {
        return false;
    }

    launchVelocity = scalarMultiplyVector(dragVector, -SLINGSHOT_FORCE_SCALE);
    return true;
}

double getBodyBottomOffset(const RigidBody &body) {
    if (const CircleCollider *circleCollider = asCircleCollider(body.collider)) {
        return circleCollider->radius;
    }

    if (const BoxCollider *boxCollider = asBoxCollider(body.collider)) {
        const double halfWidth = boxCollider->width / 2.0;
        const double halfHeight = boxCollider->height / 2.0;
        const double sinAngle = sin(body.angle);
        const double cosAngle = cos(body.angle);
        const vector2d localVertices[4] = {
            {-halfWidth, -halfHeight},
            {halfWidth, -halfHeight},
            {halfWidth, halfHeight},
            {-halfWidth, halfHeight}
        };

        double maxOffsetY = 0.0;
        for (int vertexIndex = 0; vertexIndex < 4; ++vertexIndex) {
            const double offsetY = localVertices[vertexIndex].x * sinAngle +
                                   localVertices[vertexIndex].y * cosAngle;
            if (offsetY > maxOffsetY) {
                maxOffsetY = offsetY;
            }
        }
        return maxOffsetY;
    }

    if (const PolygonCollider *polygonCollider = asPolygonCollider(body.collider)) {
        const double sinAngle = sin(body.angle);
        const double cosAngle = cos(body.angle);
        double maxOffsetY = 0.0;
        for (int vertexIndex = 0; vertexIndex < polygonCollider->vertexCount; ++vertexIndex) {
            const vector2d &localVertex = polygonCollider->localVertices[vertexIndex];
            const double offsetY = localVertex.x * sinAngle + localVertex.y * cosAngle;
            if (offsetY > maxOffsetY) {
                maxOffsetY = offsetY;
            }
        }
        return maxOffsetY;
    }

    return 0.0;
}

void freezeSlingshotBody(RigidBody &body, const vector2d &anchorPosition) {
    wakeBody(body);
    body.pos = anchorPosition;
    body.linearVel = vector2d{0.0, 0.0};
    body.angularVel = 0.0;
}

void applySlingshotLaunch(RigidBody &body,
                          const vector2d &anchorPosition,
                          const vector2d &mousePosition) {
    vector2d launchVelocity{};
    if (tryComputeSlingshotVelocity(anchorPosition, mousePosition, launchVelocity)) {
        wakeBody(body);
        body.linearVel = launchVelocity;
    }
}

void cancelSlingshot(int &slingshotBodyIndex, bool &isSlingshotActive) {
    isSlingshotActive = false;
    slingshotBodyIndex = -1;
}

void drawSlingshotPreview(const RigidBody &body,
                          const vector2d &anchorPosition,
                          const vector2d &mousePosition,
                          const int screenWidth,
                          const int screenHeight) {
    const Vector2 center = {
        static_cast<float>(body.pos.x),
        static_cast<float>(body.pos.y)
    };
    const Vector2 mouse = {
        static_cast<float>(mousePosition.x),
        static_cast<float>(mousePosition.y)
    };

    DrawLineV(center, mouse, MAROON);
    DrawCircleV(center, 5.0f, MAROON);

    vector2d launchVelocity{};
    if (!tryComputeSlingshotVelocity(anchorPosition, mousePosition, launchVelocity)) {
        return;
    }

    vector2d trajectoryPosition = anchorPosition;
    vector2d trajectoryVelocity = launchVelocity;
    const double groundY = static_cast<double>(screenHeight) - getBodyBottomOffset(body);

    Vector2 previousPoint = {
        static_cast<float>(trajectoryPosition.x),
        static_cast<float>(trajectoryPosition.y)
    };

    for (int stepIndex = 0; stepIndex < MAX_TRAJECTORY_STEPS; ++stepIndex) {
        trajectoryVelocity.y += GRAVITY * TRAJECTORY_TIME_STEP;
        const vector2d nextPosition = addVector(
            trajectoryPosition,
            scalarMultiplyVector(trajectoryVelocity, TRAJECTORY_TIME_STEP));

        vector2d currentPosition = nextPosition;
        if (nextPosition.y >= groundY) {
            currentPosition.y = groundY;
        }

        const Vector2 currentPoint = {
            static_cast<float>(currentPosition.x),
            static_cast<float>(currentPosition.y)
        };
        DrawLineV(previousPoint, currentPoint, DARKGREEN);
        previousPoint = currentPoint;
        trajectoryPosition = nextPosition;

        if (nextPosition.y >= groundY ||
            nextPosition.x < 0.0 ||
            nextPosition.x > static_cast<double>(screenWidth) ||
            nextPosition.y < 0.0) {
            break;
        }
    }
}

void drawPolygonBody(const RigidBody &body, const Color bodyColor) {
    vector2d worldVertices[MAX_POLYGON_VERTICES];
    int vertexCount = 0;
    if (!getPolygonWorldVertices(&body, worldVertices, vertexCount)) {
        return;
    }

    Vector2 raylibVertices[MAX_POLYGON_VERTICES];
    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        raylibVertices[vertexIndex] = {
            static_cast<float>(worldVertices[vertexIndex].x),
            static_cast<float>(worldVertices[vertexIndex].y)
        };
    }

    DrawTriangleFan(raylibVertices, vertexCount, bodyColor);

    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        const int nextIndex = (vertexIndex + 1) % vertexCount;
        DrawLineV(raylibVertices[vertexIndex], raylibVertices[nextIndex], DARKPURPLE);
    }

    const Vector2 center = {
        static_cast<float>(body.pos.x),
        static_cast<float>(body.pos.y)
    };
    const PolygonCollider *polygonCollider = asPolygonCollider(body.collider);
    const double polygonRadius = polygonCollider ? getPolygonRadius(*polygonCollider) : 0.0;
    const Vector2 rotationLineEnd = {
        center.x + static_cast<float>(cos(body.angle) * polygonRadius),
        center.y + static_cast<float>(sin(body.angle) * polygonRadius)
    };
    DrawLineV(center, rotationLineEnd, GOLD);
}

void drawSpawnPreview(const EditorTool tool,
                      const vector2d &mousePosition,
                      const EditorSettings &settings) {
    const Vector2 previewCenter = {
        static_cast<float>(mousePosition.x),
        static_cast<float>(mousePosition.y)
    };
    const Color previewColor = Fade(toRaylibColor(getSelectedBodyColor(settings)), 0.5f);

    switch (tool) {
        case EditorTool::SPAWN_CIRCLE:
            DrawCircleV(previewCenter, static_cast<float>(settings.circleRadius), previewColor);
            break;
        case EditorTool::SPAWN_BOX:
            DrawRectangleV(
                {
                    previewCenter.x - static_cast<float>(settings.boxWidth / 2.0),
                    previewCenter.y - static_cast<float>(settings.boxHeight / 2.0)
                },
                {
                    static_cast<float>(settings.boxWidth),
                    static_cast<float>(settings.boxHeight)
                },
                previewColor);
            break;
        case EditorTool::SPAWN_POLYGON:
            DrawPoly(previewCenter,
                     settings.polygonSides,
                     static_cast<float>(settings.polygonRadius),
                     0.0f,
                     previewColor);
            break;
        default:
            break;
    }
}

void drawRigidBody(const RigidBody &body,
                   const int bodyIndex,
                   const int slingshotBodyIndex,
                   const bool isSlingshotActive) {
    if (!body.collider) {
        return;
    }

    const Vector2 raylibPosition = {
        static_cast<float>(body.pos.x),
        static_cast<float>(body.pos.y)
    };

    const Color bodyColor =
        isSlingshotActive && bodyIndex == slingshotBodyIndex ? ORANGE : toRaylibColor(body.color);

    if (const CircleCollider *circleCollider = asCircleCollider(body.collider)) {
        DrawCircleV(raylibPosition, static_cast<float>(circleCollider->radius), bodyColor);

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

        DrawRectanglePro(rectangle, origin, rotationDegrees, bodyColor);
    } else if (asPolygonCollider(body.collider)) {
        drawPolygonBody(body, bodyColor);
    }
}

void handleEditorAdjustments(const EditorTool activeTool,
                             const vector2d &mousePosition,
                             EditorSettings &settings) {
    const int hoveredBodyIndex = findBodyIndexAtPoint(mousePosition);
    RigidBody *hoveredBody =
        hoveredBodyIndex >= 0 ? &rigidBodies[hoveredBodyIndex] : nullptr;

    if (IsKeyPressed(KEY_C)) {
        settings.colorIndex = (settings.colorIndex + 1) % COLOR_PALETTE_COUNT;
        if (hoveredBody) {
            wakeBody(*hoveredBody);
            setBodyColor(hoveredBodyIndex, getSelectedBodyColor(settings));
        }
    }

    const int radiusDelta =
        (IsKeyPressed(KEY_E) ? 1 : 0) - (IsKeyPressed(KEY_Q) ? 1 : 0);
    if (radiusDelta != 0) {
        if (hoveredBody) {
            if (CircleCollider *circleCollider = asCircleCollider(hoveredBody->collider)) {
                wakeBody(*hoveredBody);
                resizeCircleBody(hoveredBodyIndex,
                                 clampEditorSize(circleCollider->radius +
                                                 radiusDelta * SIZE_STEP));
            } else if (PolygonCollider *polygonCollider =
                           asPolygonCollider(hoveredBody->collider)) {
                wakeBody(*hoveredBody);
                resizeRegularPolygonBody(hoveredBodyIndex,
                                         polygonCollider->vertexCount,
                                         clampEditorSize(getPolygonRadius(*polygonCollider) +
                                                         radiusDelta * SIZE_STEP));
            }
        } else if (activeTool == EditorTool::SPAWN_CIRCLE) {
            settings.circleRadius =
                clampEditorSize(settings.circleRadius + radiusDelta * SIZE_STEP);
        } else if (activeTool == EditorTool::SPAWN_POLYGON) {
            settings.polygonRadius =
                clampEditorSize(settings.polygonRadius + radiusDelta * SIZE_STEP);
        }
    }

    const int widthDelta =
        (IsKeyPressed(KEY_D) ? 1 : 0) - (IsKeyPressed(KEY_A) ? 1 : 0);
    const int heightDelta =
        (IsKeyPressed(KEY_S) ? 1 : 0) - (IsKeyPressed(KEY_W) ? 1 : 0);
    if (widthDelta != 0 || heightDelta != 0) {
        if (hoveredBody) {
            if (BoxCollider *boxCollider = asBoxCollider(hoveredBody->collider)) {
                wakeBody(*hoveredBody);
                resizeBoxBody(hoveredBodyIndex,
                              clampEditorSize(boxCollider->width + widthDelta * SIZE_STEP),
                              clampEditorSize(boxCollider->height + heightDelta * SIZE_STEP));
            }
        } else if (activeTool == EditorTool::SPAWN_BOX) {
            settings.boxWidth = clampEditorSize(settings.boxWidth + widthDelta * SIZE_STEP);
            settings.boxHeight = clampEditorSize(settings.boxHeight + heightDelta * SIZE_STEP);
        }
    }

    const int sidesDelta =
        (IsKeyPressed(KEY_X) ? 1 : 0) - (IsKeyPressed(KEY_Z) ? 1 : 0);
    if (sidesDelta != 0) {
        if (hoveredBody) {
            if (PolygonCollider *polygonCollider = asPolygonCollider(hoveredBody->collider)) {
                wakeBody(*hoveredBody);
                resizeRegularPolygonBody(hoveredBodyIndex,
                                         clampPolygonSides(polygonCollider->vertexCount +
                                                           sidesDelta),
                                         getPolygonRadius(*polygonCollider));
            }
        } else if (activeTool == EditorTool::SPAWN_POLYGON) {
            settings.polygonSides = clampPolygonSides(settings.polygonSides + sidesDelta);
        }
    }
}

void handleToolInput(const EditorTool activeTool,
                     const vector2d &mousePosition,
                     const EditorSettings &settings,
                     int &slingshotBodyIndex,
                     vector2d &slingshotAnchorPosition,
                     bool &isSlingshotActive) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        switch (activeTool) {
            case EditorTool::SLINGSHOT: {
                const int hitBodyIndex = findBodyIndexAtPoint(mousePosition);
                if (hitBodyIndex >= 0) {
                    slingshotBodyIndex = hitBodyIndex;
                    slingshotAnchorPosition = rigidBodies[hitBodyIndex].pos;
                    isSlingshotActive = true;
                }
                break;
            }
            case EditorTool::SPAWN_CIRCLE:
                spawnCircle(mousePosition,
                            settings.circleRadius,
                            getSelectedBodyColor(settings));
                break;
            case EditorTool::SPAWN_BOX:
                spawnBox(mousePosition,
                         settings.boxWidth,
                         settings.boxHeight,
                         getSelectedBodyColor(settings));
                break;
            case EditorTool::SPAWN_POLYGON:
                spawnRegularPolygon(mousePosition,
                                    settings.polygonSides,
                                    settings.polygonRadius,
                                    getSelectedBodyColor(settings));
                break;
            case EditorTool::DELETE_BODY: {
                const int deletedBodyIndex = findBodyIndexAtPoint(mousePosition);
                if (deletedBodyIndex >= 0) {
                    if (isSlingshotActive && deletedBodyIndex == slingshotBodyIndex) {
                        cancelSlingshot(slingshotBodyIndex, isSlingshotActive);
                    }
                    deleteBody(deletedBodyIndex);
                }
                break;
            }
        }
    }

    if (activeTool == EditorTool::SLINGSHOT) {
        if (isSlingshotActive && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            freezeSlingshotBody(rigidBodies[slingshotBodyIndex], slingshotAnchorPosition);
        } else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && isSlingshotActive) {
            applySlingshotLaunch(rigidBodies[slingshotBodyIndex],
                                 slingshotAnchorPosition,
                                 mousePosition);
            cancelSlingshot(slingshotBodyIndex, isSlingshotActive);
        }
    } else if (isSlingshotActive) {
        cancelSlingshot(slingshotBodyIndex, isSlingshotActive);
    }
}
} // namespace

int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "RigidBody Physics Core (SAT & Iteration Enabled)");
    const int displayRefreshRate = getDisplayRefreshRate();
    SetTargetFPS(displayRefreshRate);

    initRigidBodies();

    const double deltaTime = 1.0 / static_cast<double>(displayRefreshRate);
    double timeAccumulator = 0.0;
    const int impulseIterations = 6;

    EditorTool activeTool = EditorTool::SLINGSHOT;
    EditorSettings editorSettings{};
    int slingshotBodyIndex = -1;
    vector2d slingshotAnchorPosition{};
    bool isSlingshotActive = false;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_ONE)) {
            activeTool = EditorTool::SLINGSHOT;
        } else if (IsKeyPressed(KEY_TWO)) {
            activeTool = EditorTool::SPAWN_CIRCLE;
        } else if (IsKeyPressed(KEY_THREE)) {
            activeTool = EditorTool::SPAWN_BOX;
        } else if (IsKeyPressed(KEY_FOUR)) {
            activeTool = EditorTool::SPAWN_POLYGON;
        } else if (IsKeyPressed(KEY_FIVE)) {
            activeTool = EditorTool::DELETE_BODY;
        }

        const Vector2 mousePositionRaylib = GetMousePosition();
        const vector2d mousePosition = {
            static_cast<double>(mousePositionRaylib.x),
            static_cast<double>(mousePositionRaylib.y)
        };

        handleEditorAdjustments(activeTool, mousePosition, editorSettings);
        handleToolInput(activeTool,
                        mousePosition,
                        editorSettings,
                        slingshotBodyIndex,
                        slingshotAnchorPosition,
                        isSlingshotActive);

        double frameDeltaTime = GetFrameTime();
        const double maxFrameDeltaTime = deltaTime * MAX_PHYSICS_STEPS_PER_FRAME;
        if (frameDeltaTime > maxFrameDeltaTime) {
            frameDeltaTime = maxFrameDeltaTime;
        }
        timeAccumulator += frameDeltaTime;

        int physicsStepCount = 0;
        while (timeAccumulator >= deltaTime &&
               physicsStepCount < MAX_PHYSICS_STEPS_PER_FRAME) {
            if (isSlingshotActive && slingshotBodyIndex >= 0) {
                freezeSlingshotBody(rigidBodies[slingshotBodyIndex], slingshotAnchorPosition);
            }

            integrateBodies(deltaTime);
            solveCollisions(impulseIterations);
            screenBoundaryConstraints(screenWidth, screenHeight);
            dampSmallVelocities();
            timeAccumulator -= deltaTime;
            ++physicsStepCount;
        }
        if (physicsStepCount == MAX_PHYSICS_STEPS_PER_FRAME) {
            timeAccumulator = std::min(timeAccumulator, deltaTime);
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawText("Physics Engine: SAT ORIENTED COLLIDERS ACTIVE", 15, 15, 16, DARKGRAY);
        DrawText(TextFormat("Tool: %s", toolName(activeTool)), 15, 35, 16, DARKGRAY);
        DrawText("1 Slingshot | 2 Circle | 3 Box | 4 Polygon | 5 Delete", 15, 55, 16, DARKGRAY);
        DrawText(TextFormat("Physics: %d Hz", displayRefreshRate), 15, 75, 16, DARKGRAY);
        DrawText(TextFormat("Color: %s | Circle R: %.0f | Box: %.0fx%.0f | Polygon: %d sides R %.0f",
                            getSelectedColorName(editorSettings),
                            editorSettings.circleRadius,
                            editorSettings.boxWidth,
                            editorSettings.boxHeight,
                            editorSettings.polygonSides,
                            editorSettings.polygonRadius),
                 15,
                 95,
                 16,
                 DARKGRAY);
        DrawText("Hover object to edit it | C color | Q/E radius | A/D width | W/S height | Z/X polygon sides",
                 15,
                 115,
                 16,
                 DARKGRAY);
        DrawFPS(screenWidth - 100, 15);

        for (int bodyIndex = 0; bodyIndex < RIGIDBODY_COUNT; ++bodyIndex) {
            drawRigidBody(rigidBodies[bodyIndex],
                          bodyIndex,
                          slingshotBodyIndex,
                          isSlingshotActive);
        }

        if (activeTool == EditorTool::SPAWN_CIRCLE ||
            activeTool == EditorTool::SPAWN_BOX ||
            activeTool == EditorTool::SPAWN_POLYGON) {
            drawSpawnPreview(activeTool, mousePosition, editorSettings);
        }

        if (isSlingshotActive && slingshotBodyIndex >= 0) {
            drawSlingshotPreview(rigidBodies[slingshotBodyIndex],
                                 slingshotAnchorPosition,
                                 mousePosition,
                                 screenWidth,
                                 screenHeight);
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
