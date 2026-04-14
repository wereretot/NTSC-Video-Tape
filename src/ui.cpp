#include "ui.h"

void styleBroadcast() {
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = s.FrameRounding = s.GrabRounding = s.PopupRounding = 0.f;
    s.ScrollbarRounding = s.TabRounding = 0.f;
    s.FrameBorderSize = s.WindowBorderSize = 1.f;
    s.ItemSpacing = {5, 3};
    s.FramePadding = {5, 2};
    s.WindowPadding = {8, 6};
    s.ScrollbarSize = 10.f;
    s.GrabMinSize = 8.f;
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg]           = {.08f, .08f, .08f, 1.f};
    c[ImGuiCol_ChildBg]            = {.06f, .06f, .06f, 1.f};
    c[ImGuiCol_PopupBg]            = {.10f, .10f, .10f, 1.f};
    c[ImGuiCol_Border]             = {.30f, .30f, .30f, 1.f};
    c[ImGuiCol_FrameBg]            = {.12f, .12f, .12f, 1.f};
    c[ImGuiCol_FrameBgHovered]     = {.18f, .18f, .18f, 1.f};
    c[ImGuiCol_FrameBgActive]      = {.08f, .08f, .08f, 1.f};
    c[ImGuiCol_TitleBg]            = {.06f, .06f, .06f, 1.f};
    c[ImGuiCol_TitleBgActive]      = {.10f, .10f, .10f, 1.f};
    c[ImGuiCol_SliderGrab]         = {1.f, .70f, .00f, 1.f};
    c[ImGuiCol_SliderGrabActive]   = {1.f, .85f, .20f, 1.f};
    c[ImGuiCol_CheckMark]          = {.00f, .90f, .46f, 1.f};
    c[ImGuiCol_Button]             = {.18f, .18f, .18f, 1.f};
    c[ImGuiCol_ButtonHovered]      = {.26f, .26f, .26f, 1.f};
    c[ImGuiCol_ButtonActive]       = {.10f, .10f, .10f, 1.f};
    c[ImGuiCol_Header]             = {.16f, .16f, .16f, 1.f};
    c[ImGuiCol_HeaderHovered]      = {.22f, .22f, .22f, 1.f};
    c[ImGuiCol_HeaderActive]       = {.10f, .10f, .10f, 1.f};
    c[ImGuiCol_ScrollbarBg]        = {.05f, .05f, .05f, 1.f};
    c[ImGuiCol_ScrollbarGrab]      = {.28f, .28f, .28f, 1.f};
    c[ImGuiCol_Tab]                = {.12f, .12f, .12f, 1.f};
    c[ImGuiCol_TabHovered]         = {.22f, .22f, .22f, 1.f};
    c[ImGuiCol_TabActive]          = {.20f, .20f, .20f, 1.f};
    c[ImGuiCol_Text]               = {.90f, .88f, .82f, 1.f};
    c[ImGuiCol_TextDisabled]       = {.38f, .38f, .38f, 1.f};
    c[ImGuiCol_Separator]          = {.30f, .30f, .30f, 1.f};
}

void sectionHdr(const char* lbl) {
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4{1.f, .70f, .00f, 1.f});
    ImGui::Separator();
    ImGui::PopStyleColor();
    ImGui::TextColored({1.f, .70f, .00f, 1.f}, "▶ %s", lbl);
    ImGui::Separator();
    ImGui::Spacing();
}

void vuBar(float lv, float w, float h, bool clip) {
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, {p.x + w, p.y + h}, IM_COL32(20, 20, 20, 255));
    float f = std::clamp(lv, 0.f, 1.f) * w;
    ImU32 col = clip ? IM_COL32(255, 30, 30, 255) : lv > .7f ? IM_COL32(255, 160, 0, 255) : IM_COL32(0, 200, 80, 255);
    if (f > .01f) dl->AddRectFilled(p, {p.x + f, p.y + h}, col);
    dl->AddRect(p, {p.x + w, p.y + h}, IM_COL32(90, 90, 90, 255));
    ImGui::Dummy({w, h});
}

void monitorFrame(SDL_Texture* tex, float w, float h, const char* label, ImU32 ledCol) {
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, {p.x + w + 8, p.y + h + 22}, IM_COL32(18, 18, 18, 255));
    dl->AddRect(p, {p.x + w + 8, p.y + h + 22}, IM_COL32(60, 60, 60, 255), 0, 0, 2.f);
    dl->AddRectFilled({p.x + 4, p.y + 4}, {p.x + w + 4, p.y + h + 4}, IM_COL32(0, 0, 0, 255));
    ImGui::SetCursorScreenPos({p.x + 4, p.y + 4});
    if (tex) ImGui::Image((ImTextureID)(intptr_t)tex, {w, h});
    else {
        ImGui::Dummy({w, h});
        dl->AddText({p.x + w * .5f - 22, p.y + h * .5f - 6}, IM_COL32(50, 50, 50, 255), "NO SIGNAL");
    }
    ImVec2 lb = {p.x, p.y + h + 6};
    dl->AddRectFilled(lb, {lb.x + w + 8, lb.y + 16}, IM_COL32(12, 12, 12, 255));
    dl->AddCircleFilled({lb.x + 8, lb.y + 8}, 4.f, ledCol);
    dl->AddText({lb.x + 16, lb.y + 3}, IM_COL32(180, 160, 60, 255), label);
    ImGui::SetCursorScreenPos(p);
    ImGui::Dummy({w + 8, h + 24});
}

std::string timecode(uint64_t s0) {
    uint64_t ms = SDL_GetTicks64() - s0;
    char b[32];
    std::snprintf(b, sizeof(b), "%02llu:%02llu:%02llu:%02llu",
        (unsigned long long)(ms / 3600000) % 24,
        (unsigned long long)(ms / 60000) % 60,
        (unsigned long long)(ms / 1000) % 60,
        (unsigned long long)(ms % 1000) / 33);
    return b;
}
