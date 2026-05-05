#include "game.h"
#include "simulation/engine.h"
#include "ui.h"
#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"

static Rectangle make_rect(Vector2 a, Vector2 b) {
    return {
        (a.x < b.x ? a.x : b.x),
        (a.y < b.y ? a.y : b.y),
        fabsf(b.x - a.x),
        fabsf(b.y - a.y)
    };
}

void Game::run() {
    InitWindow(1280, 720, "Crowd Simulation");
    SetTargetFPS(60);
    rlImGuiSetup(true);

    Vector2 sel_start = {};
    bool selecting = false;
    Rectangle selection = {};

    while (!WindowShouldClose()) {
        Vector2 mouse = GetMousePosition();

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            sel_start = mouse;
            selecting = true;
        }
        if (selecting)
            selection = make_rect(sel_start, mouse);
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
            selecting = false;

        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            set_target_in_rect(selection, { mouse.x, mouse.y, 0.0f });
            selection = {};
        }

        sim_tick(GetFrameTime());

        BeginDrawing();
        ClearBackground(DARKGRAY);

        for (const auto& t : get_agent_targets())
            DrawCircleLines((int)t.x, (int)t.y, 5.0f, RED);

        auto positions = get_agent_positions();
        for (const auto& pos : positions) {
            bool selected = CheckCollisionPointRec({ pos.x, pos.y }, selection);
            DrawCircleV({ pos.x, pos.y }, 20.0f, selected ? YELLOW : RAYWHITE);
        }

        if (selecting)
            DrawRectangleLinesEx(selection, 1.0f, { 255, 255, 255, 180 });

        rlImGuiBegin();
        ui_draw();
        rlImGuiEnd();

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
}
