#include "game.h"
#include "simulation/engine.h"
#include "simulation/processors/pathfinding.h"
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

        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            const float ts = get_wall_config().tile_size;
            add_wall_tile((int)(mouse.x / ts), (int)(mouse.y / ts));
        }

        sim_tick(GetFrameTime());

        BeginDrawing();
        ClearBackground(DARKGRAY);

        const float ts = get_wall_config().tile_size;

        for (const auto& [gk, ff] : get_flow_fields()) {
            for (const auto& [tk, dir] : ff.dir) {
                int tx, ty; decode_tile(tk, tx, ty);
                float hue = fmodf(atan2f(dir.y, dir.x) * RAD2DEG + 360.0f, 360.0f);
                Color c   = ColorFromHSV(hue, 0.7f, 0.8f);
                c.a       = 120;
                DrawRectangle(tx * (int)ts, ty * (int)ts, (int)ts, (int)ts, c);
                float mx = tx * ts + ts * 0.5f, my = ty * ts + ts * 0.5f;
                DrawLineV({ mx, my }, { mx + dir.x * ts * 0.45f, my + dir.y * ts * 0.45f }, { 255, 255, 255, 160 });
            }
        }

        for (const auto& key : get_wall_tiles()) {
            int tx, ty;
            decode_tile(key, tx, ty);
            DrawRectangle(tx * (int)ts, ty * (int)ts, (int)ts, (int)ts, GRAY);
        }

        for (const auto& t : get_agent_targets())
            DrawCircleLines((int)t.x, (int)t.y, 5.0f, RED);

        auto positions = get_agent_positions();
        auto arrived   = get_agent_arrived();
        for (int i = 0; i < (int)positions.size(); ++i) {
            const auto& pos = positions[i];
            bool selected = CheckCollisionPointRec({ pos.x, pos.y }, selection);
            Color color = selected ? YELLOW : (arrived[i] != 0.0f ? GREEN : RAYWHITE);
            DrawCircleV({ pos.x, pos.y }, get_agent_config().agent_radius, color);
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
