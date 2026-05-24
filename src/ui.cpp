#include "ui.h"
#include "simulation/config.h"
#include "imgui.h"

static constexpr float panel_width   = 400.0f;
static constexpr float panel_padding = 20.0f;
static constexpr float panel_gap = 150.0f;

static float panel_cursor_y = panel_padding;

static void draw_panel(const char* title, std::vector<ConfigField> fields) {
    const float x = ImGui::GetIO().DisplaySize.x - panel_width - panel_padding;
    ImGui::SetNextWindowPos({ x, panel_cursor_y }, ImGuiCond_Once);
    ImGui::SetNextWindowSize({ panel_width, 0.0f }, ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Once);

    ImGui::Begin(title);
    for (auto& field : fields) {
        std::visit([](auto& f) {
            using T = std::decay_t<decltype(f)>;
            if constexpr (std::is_same_v<T, ConfigFloatField>)
                ImGui::SliderFloat(f.name, f.value, f.min, f.max);
            else if constexpr (std::is_same_v<T, ConfigBoolField>)
                ImGui::Checkbox(f.name, f.value);
        }, field);
    }
    ImGui::End();
    
    panel_cursor_y += panel_gap;
}

void ui_draw() {
    panel_cursor_y = panel_padding;
    draw_panel("Pathfinding", get_pathfinding_config().fields());
    draw_panel("Agent",       get_agent_config().fields());
    draw_panel("Steering",  get_steering_config().fields());
    draw_panel("Avoidance", get_avoidance_config().fields());
    draw_panel("Separation", get_seperation_config().fields());
    draw_panel("Arrival",   get_arrival_config().fields());
    draw_panel("Walls",     get_wall_config().fields());
}
