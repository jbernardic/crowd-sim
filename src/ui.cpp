#include "ui.h"
#include "simulation/config.h"
#include "imgui.h"

static constexpr float panel_width   = 300.0f;
static constexpr float panel_padding = 20.0f;

static void draw_field(ConfigField& field) {
    std::visit([](auto& f) {
        using T = std::decay_t<decltype(f)>;
        if constexpr (std::is_same_v<T, ConfigFloatField>)
            ImGui::SliderFloat(f.name, f.value, f.min, f.max);
        else if constexpr (std::is_same_v<T, ConfigBoolField>)
            ImGui::Checkbox(f.name, f.value);
    }, field);
}

static void draw_module(ConfigModule& mod) {
    if (!ImGui::CollapsingHeader(mod.name, ImGuiTreeNodeFlags_DefaultOpen))
        return;

    ImGui::PushID(mod.name);

    // On/off toggle for switchable features; tuning-only modules skip it.
    if (mod.enabled)
        ImGui::Checkbox("Enabled", mod.enabled);

    // Grey out the tunables while the feature is switched off.
    const bool off = mod.enabled && !*mod.enabled;
    if (off) ImGui::BeginDisabled();
    for (auto& field : mod.fields)
        draw_field(field);
    if (off) ImGui::EndDisabled();

    ImGui::PopID();
    ImGui::Spacing();
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

    for (auto& mod : config_modules())
        draw_module(mod);

    ImGui::End();
}
