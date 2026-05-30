#include "ui.h"
#include "simulation/config.h"
#include "imgui.h"

static constexpr float panel_width   = 300.0f;
static constexpr float panel_padding = 20.0f;

static void draw_section(const char* title, std::vector<ConfigField> fields) {
    if (ImGui::CollapsingHeader(title, ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID(title);
        for (auto& field : fields) {
            std::visit([](auto& f) {
                using T = std::decay_t<decltype(f)>;
                if constexpr (std::is_same_v<T, ConfigFloatField>)
                    ImGui::SliderFloat(f.name, f.value, f.min, f.max);
                else if constexpr (std::is_same_v<T, ConfigBoolField>)
                    ImGui::Checkbox(f.name, f.value);
            }, field);
        }
        ImGui::PopID();
        ImGui::Spacing();
    }
}

void ui_draw() {
    const ImGuiIO& io = ImGui::GetIO();
    const float x = io.DisplaySize.x - panel_width - panel_padding;
    const float y = panel_padding;
    const float h = io.DisplaySize.y - 2.0f * panel_padding;

    ImGui::SetNextWindowPos({ x, y }, ImGuiCond_Always);
    ImGui::SetNextWindowSize({ panel_width, h }, ImGuiCond_Always);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Once);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove
                           | ImGuiWindowFlags_NoResize
                           | ImGuiWindowFlags_NoTitleBar
                           | ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::Begin("Config", nullptr, flags);
    ImGui::Text("Configuration");
    ImGui::Separator();
    ImGui::Spacing();

    draw_section("Pathfinding", get_pathfinding_config().fields());
    draw_section("Agent",       get_agent_config().fields());
    draw_section("Steering",    get_steering_config().fields());
    draw_section("Avoidance",   get_avoidance_config().fields());
    draw_section("Settling",    get_arrival_config().fields());
    draw_section("Walls",       get_wall_config().fields());

    ImGui::End();
}
