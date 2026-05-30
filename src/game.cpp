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

    // World-space camera. At zoom 1 with target == offset the world maps 1:1 to
    // the screen, matching the original (camera-less) view. The mouse wheel zooms
    // toward the cursor with no lower bound, so the world can be zoomed out
    // indefinitely; WASD / arrows pan.
    Camera2D camera = {};
    camera.offset = { 640.0f, 360.0f };
    camera.target = { 640.0f, 360.0f };
    camera.zoom   = 1.0f;

    Vector2 sel_start = {};
    bool selecting = false;
    Rectangle selection = {};

    while (!WindowShouldClose()) {
        const bool    over_ui      = ImGui::GetIO().WantCaptureMouse;
        const Vector2 screen_mouse = GetMousePosition();

        // --- Camera control ---
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f && !over_ui) {
            // Keep the world point under the cursor fixed while zooming.
            Vector2 world_before = GetScreenToWorld2D(screen_mouse, camera);
            camera.offset = screen_mouse;
            camera.target = world_before;
            camera.zoom  *= (wheel > 0.0f) ? 1.1f : 1.0f / 1.1f;
            if (camera.zoom < 0.0001f) camera.zoom = 0.0001f;
        }
        // Pan faster the further out we are, so it feels constant on screen.
        const float pan = 600.0f * GetFrameTime() / camera.zoom;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))  camera.target.x -= pan;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) camera.target.x += pan;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))    camera.target.y -= pan;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))  camera.target.y += pan;

        // Everything below works in world space.
        const Vector2 mouse = GetScreenToWorld2D(screen_mouse, camera);

        // --- Input (ignored while the pointer is over the config panel) ---
        if (!over_ui && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            sel_start = mouse;
            selecting = true;
        }
        if (selecting)
            selection = make_rect(sel_start, mouse);
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
            selecting = false;

        if (!over_ui && IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            set_target_in_rect(selection, { mouse.x, mouse.y, 0.0f });
            selection = {};
        }

        if (!over_ui && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            const float ts = get_wall_config().tile_size;
            add_wall_tile((int)floorf(mouse.x / ts), (int)floorf(mouse.y / ts));
        }

        sim_tick(GetFrameTime());

        BeginDrawing();
        ClearBackground(DARKGRAY);

        BeginMode2D(camera);

        const float ts = get_wall_config().tile_size;

        for (const auto& key : get_wall_tiles()) {
            int tx, ty;
            decode_tile(key, tx, ty);
            DrawRectangle(tx * (int)ts, ty * (int)ts, (int)ts, (int)ts, GRAY);
        }

        const auto& slots = get_formation_slots();
        // The follower slots sit behind the agents; slot 0 (the destination) is
        // drawn last, on top of everything.
        for (int i = 1; i < (int)slots.size(); ++i)
            DrawCircleLines((int)slots[i].x, (int)slots[i].y, get_agent_config().radius, BLUE);

        auto positions    = get_agent_positions();
        const auto& tgts  = get_agent_targets();   // each agent's local slot goal
        const float settle = get_formation_config().settle_radius;

        for (const auto& t : tgts)
            DrawCircleLines((int)t.x, (int)t.y, 5.0f, RED);

        for (int i = 0; i < (int)positions.size(); ++i) {
            const auto& pos = positions[i];
            bool selected = CheckCollisionPointRec({ pos.x, pos.y }, selection);
            float dx = tgts[i].x - pos.x, dy = tgts[i].y - pos.y;
            bool settled = (dx * dx + dy * dy) <= settle * settle;
            Color color = selected ? YELLOW : (settled ? GREEN : RAYWHITE);
            DrawCircleV({ pos.x, pos.y }, get_agent_config().radius, color);
        }

        // Destination slot on top of everything.
        if (!slots.empty())
            DrawCircleLines((int)slots[0].x, (int)slots[0].y, get_agent_config().radius, SKYBLUE);

        if (selecting)
            DrawRectangleLinesEx(selection, 1.0f / camera.zoom, { 255, 255, 255, 180 });

        EndMode2D();

        DrawFPS(10, 10);

        rlImGuiBegin();
        ui_draw();
        rlImGuiEnd();

        EndDrawing();
    }

    rlImGuiShutdown();
    CloseWindow();
}
