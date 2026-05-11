#include "ui.h"
#include "app_state.h"
#include "audio_capture.h"

static void drawWaveformScope(ImDrawList* dl, const cv::Mat& outF, float sx0, float sy0, float sx1, float sy1, float srcW, float srcH) {
    constexpr float kIlo = -50.f, kIhi = 120.f, kIspan = 170.f;
    auto ireY = [&](float ire) -> float {
        return sy1 - (ire - kIlo) / kIspan * srcH;
    };

    struct GL { float ire; const char* lbl; };
    static constexpr GL kG[] = {{100.f, "100"}, {75.f, "75"}, {7.5f, "7.5"}, {0.f, "0"}, {-40.f, "-40"}};
    for (auto& g : kG) {
        float gy = ireY(g.ire);
        dl->AddLine({sx0, gy}, {sx1, gy}, IM_COL32(0, 50, 16, 255), 1.f);
        dl->AddText({sx0 + 2, gy - 9}, IM_COL32(0, 100, 36, 220), g.lbl);
    }

    constexpr float kT = 63.556f;
    constexpr float fFP   = 1.500f / kT;
    constexpr float fSYE  = (1.500f + 4.700f) / kT;
    constexpr float fBRE  = (1.500f + 4.700f + 0.600f) / kT;
    constexpr float fBUE  = (1.500f + 4.700f + 0.600f + 2.500f) / kT;
    constexpr float fACT  = (1.500f + 4.700f + 0.600f + 2.500f + 1.600f) / kT;
    auto xf = [&](float f) -> float { return sx0 + f * srcW; };

    for (float fx : {fFP, fSYE, fBRE, fBUE, fACT})
        dl->AddLine({xf(fx), sy0}, {xf(fx), sy1}, IM_COL32(0, 32, 10, 200), 1.f);

    const float ly = sy0 + 1.f;
    dl->AddText({sx0 + 1, ly},      IM_COL32(0, 75, 28, 200), "FP");
    dl->AddText({xf(fFP) + 1, ly},  IM_COL32(0, 75, 28, 200), "SYNC");
    dl->AddText({xf(fBRE) + 1, ly}, IM_COL32(0, 75, 28, 200), "BST");
    dl->AddText({xf(fACT) + 1, ly}, IM_COL32(0, 75, 28, 200), "ACTIVE");

    if (!outF.empty()) {
        constexpr int N_LINES = 16, N_PX = 480;
        const int FH2 = outF.rows, FW2 = outF.cols;
        const int activePx = std::max(1, int((1.f - fACT) * N_PX));
        const float scStep = 2.f * 3.14159265f * 188.5f / float(activePx);

        const int pFP  = int(fFP * N_PX);
        const int pSYE = int(fSYE * N_PX);
        const int pBRE = int(fBRE * N_PX);
        const int pBUE = int(fBUE * N_PX);
        const int pACT = int(fACT * N_PX);

        for (int li = 0; li < N_LINES; ++li) {
            int srcY = int(float(li) / float(N_LINES) * float(FH2));
            srcY = std::clamp(srcY, 0, FH2 - 1);
            const uchar* row = outF.ptr(srcY);
            float mid = float(li) / float(N_LINES - 1) - 0.5f;
            uint8_t br = uint8_t(70 + 140 * (1.f - std::abs(mid) * 1.6f));
            ImU32 col = IM_COL32(0, br, uint8_t(br * 0.3f), 200);
            float prevX = sx0, prevY = ireY(0.f);

            for (int px = 0; px < N_PX; ++px) {
                float ire = 0.f;
                if (px < pFP) {
                    ire = 0.f;
                } else if (px < pSYE) {
                    ire = -40.f;
                } else if (px < pBRE) {
                    ire = 0.f;
                } else if (px < pBUE) {
                    float bp = float(px - pBRE) / float(pBUE - pBRE) * 2.f * 3.14159265f * 9.f;
                    ire = 20.f * std::sin(bp);
                } else if (px < pACT) {
                    ire = 0.f;
                } else {
                    float xSrc = float(px - pACT) / float(activePx) * float(FW2);
                    xSrc = std::max(0.f, std::min(float(FW2 - 1.001f), xSrc));
                    int x0 = int(xSrc);
                    float xfr = xSrc - x0;
                    int x1 = std::min(x0 + 1, FW2 - 1);
                    float r  = (row[x0 * 3 + 2] * (1 - xfr) + row[x1 * 3 + 2] * xfr) / 255.f;
                    float g2 = (row[x0 * 3 + 1] * (1 - xfr) + row[x1 * 3 + 1] * xfr) / 255.f;
                    float b2 = (row[x0 * 3 + 0] * (1 - xfr) + row[x1 * 3 + 0] * xfr) / 255.f;
                    float Y  = .2990f * r + .5870f * g2 + .1140f * b2;
                    float I  = .5957f * r - .2744f * g2 - .3213f * b2;
                    float Q2 = .2115f * r - .5227f * g2 + .3112f * b2;
                    float theta = float(px - pACT) * scStep;
                    float comp = Y + I * std::cos(theta) - Q2 * std::sin(theta);
                    ire = comp * 100.f + 7.5f;
                    ire = std::clamp(ire, -40.f, 120.f);
                }
                float nx = sx0 + float(px) / float(N_PX) * srcW;
                float ny = std::clamp(ireY(ire), sy0, sy1);
                if (px > 0) dl->AddLine({prevX, prevY}, {nx, ny}, col, 1.f);
                prevX = nx;
                prevY = ny;
            }
        }
    }

    { float gy = ireY(0.f);
      dl->AddLine({sx0, gy}, {sx1, gy}, IM_COL32(0, 80, 26, 255), 1.f); }
}

void drawUI(AppState& app, SDL_Renderer* ren) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->Pos);
    ImGui::SetNextWindowSize(vp->Size);
    ImGui::Begin("##MAIN", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar);
    const float W = vp->Size.x, H = vp->Size.y;

    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4{.04f, .04f, .04f, 1.f});
    ImGui::BeginChild("##hdr", {0, 30}, false, ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPosY(5);
    ImGui::TextColored({.20f, .90f, 1.f, 1.f}, "■ ADO-8500");
    ImGui::SameLine();
    ImGui::TextColored({1.f, .70f, 0.f, 1.f}, "BROADCAST VIDEO EFFECTS PROCESSOR");
    ImGui::SameLine();
    ImGui::TextDisabled("  |  ");
    ImGui::SameLine();
    ImGui::TextColored({1.f, .70f, 0.f, 1.f}, "%s", timecode(app.startMs).c_str());
    ImGui::SameLine();
    float pls = .5f + .5f * std::sin(SDL_GetTicks64() * .003f);
    ImGui::TextColored({0.f, pls * .9f, 0.f, 1.f}, "  ◉ PGM");
    ImGui::EndChild();
    ImGui::PopStyleColor();
    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4{.55f, .45f, .00f, 1.f});
    ImGui::Separator();
    ImGui::PopStyleColor();

    const float monH = H * .36f;
    ImGui::BeginChild("##mons", {0, monH}, false, ImGuiWindowFlags_NoScrollbar);
    {
        cv::Mat outF, rawF;
        {
            std::lock_guard<std::mutex> lk(app.frameMu);
            outF = app.lastOut;
            rawF = app.lastRaw;
        }
        if (!outF.empty()) app.outTex.update(outF);

        float pgW = W * .40f, pgH = monH - 32;
        float lpls = .4f + .6f * pls;
        monitorFrame(app.outTex.tex, pgW, pgH, "PGM  OUTPUT", IM_COL32(0, int(lpls * 220), 0, 255));
        ImGui::SameLine(0, 12);

        float srcW = pgW * .52f, srcH = srcW / ASPECT;
        if (!rawF.empty()) app.rawTex.update(rawF);
        monitorFrame(app.rawTex.tex, srcW, srcH, "SRC  INPUT", IM_COL32(50, 150, 255, 255));
        ImGui::SameLine(0, 12);

        {
            ImVec2 cp = ImGui::GetCursorScreenPos();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const float aw = srcW * 1.05f, ah = srcH;
            dl->AddRectFilled(cp, {cp.x + aw + 8, cp.y + ah + 22}, IM_COL32(14, 14, 14, 255));
            dl->AddRect(cp, {cp.x + aw + 8, cp.y + ah + 22}, IM_COL32(55, 55, 55, 255), 0, 0, 2.f);
            const float sx0 = cp.x + 4, sy0 = cp.y + 4, sx1 = sx0 + aw, sy1 = sy0 + ah;
            dl->AddRectFilled({sx0, sy0}, {sx1, sy1}, IM_COL32(0, 8, 3, 255));

            drawWaveformScope(dl, outF, sx0, sy0, sx1, sy1, aw, ah);

            ImVec2 lb = {cp.x, cp.y + ah + 6};
            dl->AddRectFilled(lb, {lb.x + aw + 8, lb.y + 16}, IM_COL32(12, 12, 12, 255));
            dl->AddCircleFilled({lb.x + 8, lb.y + 8}, 4.f, IM_COL32(0, 200, 80, 255));
            dl->AddText({lb.x + 16, lb.y + 3}, IM_COL32(180, 160, 60, 255), "WFM  NTSC COMPOSITE");
            ImGui::SetCursorScreenPos(cp);
            ImGui::Dummy({aw + 8, ah + 24});
        }
        ImGui::SameLine(0, 12);

        {
            ImVec2 cp = ImGui::GetCursorScreenPos();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            float sw2 = std::max(80.f, W - cp.x + vp->Pos.x - 24), sh2 = srcH;
            dl->AddRectFilled(cp, {cp.x + sw2 + 8, cp.y + sh2 + 22}, IM_COL32(18, 18, 18, 255));
            dl->AddRect(cp, {cp.x + sw2 + 8, cp.y + sh2 + 22}, IM_COL32(60, 60, 60, 255), 0, 0, 2.f);
            dl->AddRectFilled({cp.x + 4, cp.y + 4}, {cp.x + sw2 + 4, cp.y + sh2 + 4}, IM_COL32(0, 0, 10, 255));
            ImGui::SetCursorScreenPos({cp.x + 8, cp.y + 6});
            ImGui::BeginChild("##sysstat", {sw2 - 6, sh2 - 8}, false, ImGuiWindowFlags_NoScrollbar);
            ImGui::PushStyleColor(ImGuiCol_ChildBg, {0, 0, .04f, 1.f});
            ImGui::TextColored({1.f, .70f, 0.f, 1.f}, "── SYSTEM ──");
            ImGui::Text("CAP  %5.1f fps", (double)app.capture.fps());
            ImGui::Text("PROC %5.1f fps", (double)app.pipeline.fps());
            ImGui::Text("RAW-Q  %2zu  OUT-Q  %2zu", app.pipeline.rawQ.size(), app.pipeline.outQ.size());
            ImGui::Spacing();
            ImGui::TextColored({1.f, .70f, 0.f, 1.f}, "── A/V SYNC ──");
            int64_t ap = g_audioSamplePos.load();
            float sf = g_sourceFPS.load();
            int64_t vf = sf > 0 ? int64_t(double(ap) / (AUDIO_SR / sf)) : 0;
            ImGui::Text("APOS  %8lld smp", (long long)ap);
            ImGui::Text("VFRM  %8lld", (long long)vf);
            ImGui::Text("SFPS  %6.2f  (NTSC %.2f)", (double)sf, kNTSC_FPS);

            bool locked = false;
            {
                std::lock_guard<std::mutex> a_lk(app.audioMu);
                locked = app.tapeEngine && app.audioIO && app.audioIO->is_open();
            }

            ImGui::Spacing();
            if (locked) ImGui::TextColored({0.f, .9f, .4f, 1.f}, "● A/V LOCKED");
            else        ImGui::TextColored({.9f, .3f, .3f, 1.f}, "○ FREE-RUN");

            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, {1.f, .8f, .2f, 1.f});
            ImGui::SetNextItemWidth(sw2 - 10);
            ImGui::SliderFloat("A/V SYNC (ms)", &app.av_sync_offset_ms, 0.0f, 500.0f, "%.0f ms");
            ImGui::PopStyleColor();
            ImGui::PopStyleColor();
            ImGui::EndChild();
            ImVec2 lb = {cp.x, cp.y + sh2 + 6};
            dl->AddRectFilled(lb, {lb.x + sw2 + 8, lb.y + 16}, IM_COL32(12, 12, 12, 255));
            dl->AddCircleFilled({lb.x + 8, lb.y + 8}, 4.f, locked ? IM_COL32(0, 220, 100, 255) : IM_COL32(200, 50, 50, 255));
            dl->AddText({lb.x + 16, lb.y + 3}, IM_COL32(180, 160, 60, 255), "SYS  STATUS");
            ImGui::SetCursorScreenPos(cp);
            ImGui::Dummy({0, 0});
        }
    }
    ImGui::EndChild();

    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4{.55f, .45f, .00f, 1.f});
    ImGui::Separator();
    ImGui::PopStyleColor();

    const float ctrlH = H - monH - 72;
    ImGui::BeginChild("##ctrl", {0, ctrlH});
    const float colW = (W - 24) / 3.f;

    ImGui::BeginChild("##c1", {colW, ctrlH - 4}, true);
    sectionHdr("INPUT SOURCE");
    auto stopAudioAndClear = [&app]() {
        {
            std::lock_guard<std::mutex> lk(app.pp.mu);
            app.pp.engValid.store(false);
            app.pp.exPtr->tapeEnginePtr = nullptr;
        }
        SDL_Delay(50);
        std::lock_guard<std::mutex> lk(app.audioMu);
        std::string oldWav;
        {
            std::lock_guard<std::mutex> flk(app.fileMu);
            oldWav = app.audioWavPath;
            app.audioWavPath = "";
        }
        if (app.audioIO) {
            app.audioIO->stop();
            app.audioIO->close();
        }
        app.audioIO.reset();
        app.tapeEngine.reset();
        if (!oldWav.empty()) std::remove(oldWav.c_str());
        app.pipeline.rawQ.clear();
        app.pipeline.outQ.clear();
    };

    auto srcBtnWithStop = [&](const char* lbl, SourceType st) {
        SourceType cur = app.capture.sourceType.load();
        if (cur == st) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{0.f, .22f, .08f, 1.f});
        if (ImGui::Button(lbl, {colW - 18, 0})) {
            if (cur != st) {
                stopAudioAndClear();
                app.capture.sourceType = st;
            }
        }
        if (cur == st) ImGui::PopStyleColor();
    };

    srcBtnWithStop("◉  SMPTE TEST SIGNAL", SourceType::Demo);
    srcBtnWithStop("◉  CAMERA INPUT", SourceType::Camera);
    if (ImGui::Button("◉  OPEN VIDEO FILE ...", {colW - 18, 0})) {
        stopAudioAndClear();
        std::string cmd;
#ifdef _WIN32
        cmd = "powershell -Command \"Add-Type -AssemblyName System.Windows.Forms;$d=New-Object Windows.Forms.OpenFileDialog;$d.Filter='Video|*.mp4;*.avi;*.mkv;*.mov;*.webm|All|*.*';if($d.ShowDialog()-eq'OK'){Write-Output $d.FileName}\"";
#elif defined(__APPLE__)
        cmd = "osascript -e 'POSIX path of (choose file of type {\"mp4\",\"avi\",\"mkv\",\"mov\",\"webm\"})'";
#else
        cmd = "zenity --file-selection --title='Open Video File' --file-filter='*.mp4 *.avi *.mkv *.mov *.webm' 2>/dev/null";
#endif
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buf[512];
            if (fgets(buf, sizeof(buf), pipe)) {
                std::string path = buf;
                path.erase(path.find_last_not_of(" \n\r\t") + 1);
                if (!path.empty()) {
                    std::lock_guard<std::mutex> lk(app.fileMu);
                    app.filePath = path;
                    app.capture.sourceType = SourceType::File;
                }
            }
            pclose(pipe);
        }
    }
    sectionHdr("EFFECT SELECT");
    for (int i = 0; i < (int)Effects::Type::COUNT; ++i) {
        bool sel = app.selectedFx == i;
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{.25f, .17f, 0.f, 1.f});
        if (ImGui::Button(Effects::kNames[i], {(colW - 18) * .5f - 3, 0})) {
            app.selectedFx = i;
            std::lock_guard<std::mutex> lk(app.pp.mu);
            app.pp.fx.type = (Effects::Type)i;
            Effects::resetTrail();
        }
        if (sel) ImGui::PopStyleColor();
        if (i % 2 == 0) ImGui::SameLine();
    }
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("##c2", {colW, ctrlH - 4}, true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    sectionHdr("VIDEO PROCESSING");
    ImGui::Checkbox("NTSC/VHS SIMULATION", &app.ntscEnabled);
    {
        std::lock_guard<std::mutex> lk(app.pp.mu);
        app.pp.ntscEnabled = app.ntscEnabled;
    }
    ImGui::Text("TAPE SPEED:");
    ImGui::SameLine();
    bool spCh = false;
    if (ImGui::RadioButton("SP", &app.tapeSpeedIdx, 0)) spCh = true;
    ImGui::SameLine();
    if (ImGui::RadioButton("LP", &app.tapeSpeedIdx, 1)) spCh = true;
    ImGui::SameLine();
    if (ImGui::RadioButton("EP", &app.tapeSpeedIdx, 2)) spCh = true;
    if (spCh) {
        static const float kIPS[] = {15.f, 7.5f, 3.75f};
        app.baseParams.ips_base = kIPS[app.tapeSpeedIdx];
        std::lock_guard<std::mutex> lk(app.pp.mu);
        app.pp.spdIdx = app.tapeSpeedIdx;
    }

    // ── VCR Configuration ─────────────────────────────────────────
    static int vcrBrandIdx = 0;
    static int vcrHeadIdx = 0;
    static int vcrFormatIdx = 0;
    static const char* brandNames[] = {
        "JVC HR-S3600", "Panasonic AG-1980", "Sony SLV-1000",
        "Mitsubishi HS-HD2000U", "Sharp VC-A588U", "Toshiba M-462"
    };
    static const char* headNames[] = {
        "2-Head (Standard)", "4-Head (HQ/Slow-Mo)", "6-Head (S-VHS/Pro)"
    };
    static const char* formatNames[] = {
        "VHS", "S-VHS"
    };

    ImGui::Spacing();
    ImGui::Text("VCR BRAND:");
    ImGui::SetNextItemWidth(colW - 130);
    if (ImGui::Combo("##brand", &vcrBrandIdx, brandNames, 6)) {
        app.pipeline.ntsc_.setBrand(static_cast<VCRBrand>(vcrBrandIdx));
    }

    ImGui::Text("HEAD CONFIG:");
    ImGui::SetNextItemWidth(colW - 130);
    if (ImGui::Combo("##heads", &vcrHeadIdx, headNames, 3)) {
        app.pipeline.ntsc_.setHeadCount(static_cast<HeadCount>(2 + vcrHeadIdx * 2));
    }

    ImGui::Text("TAPE FORMAT:");
    ImGui::SetNextItemWidth(colW - 130);
    if (ImGui::Combo("##format", &vcrFormatIdx, formatNames, 2)) {
        app.pipeline.ntsc_.setTapeFormat(vcrFormatIdx == 0 ? TapeFormat::VHS : TapeFormat::SVHS);
    }

    bool hasTapeEngine = false;
    float vL = 0.f, vR = 0.f;
    {
        std::lock_guard<std::mutex> a_lk(app.audioMu);
        hasTapeEngine = app.tapeEngine != nullptr;
        if (app.audioIO) {
            vL = app.audioIO->get_level_left();
            vR = app.audioIO->get_level_right();
        }
    }

    bool tCh = false;
    auto sl3 = [&](const char* n, float* v, float lo, float hi) {
        ImGui::SetNextItemWidth(colW - 130);
        tCh |= ImGui::SliderFloat(n, v, lo, hi);
    };
    EngineParams& bp = app.baseParams;

    if (!hasTapeEngine) {
        ImGui::Spacing();
        ImGui::TextColored({.9f, .3f, .3f, 1.f}, "Load video file to enable tape DSP");
    }

    sectionHdr("TRANSPORT");
    sl3("WOW", &bp.wow_dep, 0, 5);
    sl3("FLUTTER", &bp.flutter_dep, 0, 1);
    bp.flutter_dep = std::max(bp.flutter_dep, 0.001f);
    sl3("MOTOR DEGRADE", &bp.motor_health, 0, 1);
    sl3("MOTOR DRAG", &bp.motor_drag, 0, .9f);
    sl3("DROPOUT", &bp.dropout_rate, 0, .1f);
    sectionHdr("TAPE WEAR");
    sl3("TAPE OXIDE", &bp.oxide_shedding, 0, 1);
    sl3("DEMAGNET", &bp.demagnetization, 0, 1);
    sl3("STICKY SHED", &bp.sticky_shed, 0, .2f);
    sl3("TAPE AGE", &app.videoParams.tape_age, 0, 1);
    sl3("TAPE CREASE", &app.videoParams.tape_crease, 0, 1);
    sl3("TRACKING ERR", &app.videoParams.tracking_error, 0, 1);
    sectionHdr("ELECTRONICS & NOISE");
    sl3("TAPE HISS", &bp.hiss, 0, .02f);
    sl3("HISS COLOR", &bp.hiss_color, 0, 1);
    sl3("MAINS HUM", &bp.mains_hum, 0, .05f);
    sl3("HEAD BUMP", &bp.head_bump, 0, 1);
    sl3("HF CUTOFF", &bp.cutoff_base, 1000, 20000);
    sectionHdr("HELICAL SCAN / HEAD");
    sl3("HEAD SWEEP", &app.videoParams.helical_sweep, 0, 1);
    sl3("HS JITTER", &app.videoParams.head_switch_jitter, 0, 1);
    sl3("FM CARRIER NOISE", &app.videoParams.fm_carrier_noise, 0, 1);
    sl3("CHROMA XTALK", &app.videoParams.chroma_crosstalk, 0, 1);
    sl3("FIELD PHASE ERR", &app.videoParams.inter_field_phase_error, 0, 1);
    sl3("HEAD PRE-ECHO", &app.videoParams.head_pre_echo, 0, 1);
    sl3("DRUM ECCENTRIC", &app.videoParams.drum_eccentricity, 0, 1);
    sectionHdr("HEAD ALIGNMENT");
    sl3("HEAD AZIMUTH", &app.videoParams.head_azimuth_error, 0, 1);
    sl3("TRACKING ALIGN", &app.videoParams.tracking_alignment, 0, 1);
    sl3("DRUM HEIGHT", &app.videoParams.drum_height_error, 0, 1);
    sl3("AUDIO HEAD", &app.videoParams.audio_head_alignment, 0, 1);
    {
        std::lock_guard<std::mutex> lk2(app.pp.mu);
        app.pp.epSnap = bp;
        app.pp.vpSnap = app.videoParams;
        app.pp.tapeSpd = bp.tape_speed_mult;
        app.pp.engValid.store(hasTapeEngine);
    }

    app.vuL = vL;
    app.vuR = vR;

    if (!app.audioWavPath.empty()) {
        ImGui::SetCursorPos({15, ctrlH - 55});
        std::string wavName;
        {
            std::lock_guard<std::mutex> flk(app.fileMu);
            wavName = app.audioWavPath;
        }
        ImGui::TextColored({.5f, .5f, .5f, 1.f}, "AUDIO: %s", wavName.c_str());
    }
    ImGui::EndChild();
    ImGui::SameLine();

    ImGui::BeginChild("##c3", {colW, ctrlH - 4}, true);
    sectionHdr("TAPE PRESETS");
    struct PR { const char* n; std::function<void(EngineParams&, VideoParams&)> fn; };
    static const PR prs[] = {
        {"CLEAN DIGITAL", [](EngineParams& p, VideoParams& v) { p = {}; p.ips_base = 15; p.cutoff_base = 20000; v = {}; }},
        {"VHS  SP",      [](EngineParams& p, VideoParams& v) { p = {}; p.ips_base = 15; p.wow_dep = .5f; p.flutter_dep = .15f; p.head_bump = .4f; p.cutoff_base = 12000; p.hiss = .003f; v = {}; v.helical_sweep = .5f; v.head_switch_jitter = .15f; v.fm_carrier_noise = .08f; v.chroma_crosstalk = .12f; v.inter_field_phase_error = .06f; v.head_pre_echo = .02f; }},
        {"VHS  LP",      [](EngineParams& p, VideoParams& v) { p = {}; p.ips_base = 7.5f; p.wow_dep = 1.5f; p.flutter_dep = .3f; p.head_bump = .6f; p.cutoff_base = 8000; p.hiss = .006f; p.hiss_color = .15f; v = {}; v.helical_sweep = .55f; v.head_switch_jitter = .25f; v.fm_carrier_noise = .15f; v.chroma_crosstalk = .22f; v.inter_field_phase_error = .1f; v.head_pre_echo = .04f; }},
        {"VHS  EP",      [](EngineParams& p, VideoParams& v) { p = {}; p.ips_base = 3.75f; p.wow_dep = 3.f; p.flutter_dep = .6f; p.motor_health = .6f; p.head_bump = .8f; p.cutoff_base = 5000; p.hiss = .01f; p.hiss_color = .25f; p.demagnetization = .4f; p.print_through = .2f; v = {}; v.helical_sweep = .65f; v.head_switch_jitter = .35f; v.fm_carrier_noise = .25f; v.chroma_crosstalk = .35f; v.inter_field_phase_error = .18f; v.head_pre_echo = .06f; v.drum_eccentricity = .1f; }},
        {"WORN  EP",     [](EngineParams& p, VideoParams& v) { p = {}; p.ips_base = 3.75f; p.wow_dep = 4.f; p.flutter_dep = .8f; p.motor_health = .75f; p.sticky_shed = .08f; p.dropout_rate = .05f; p.demagnetization = .7f; p.cutoff_base = 3000; p.hiss = .015f; p.hiss_color = .35f; p.mains_hum = .015f; v = {}; v.helical_sweep = .75f; v.head_switch_jitter = .5f; v.fm_carrier_noise = .35f; v.chroma_crosstalk = .45f; v.inter_field_phase_error = .25f; v.head_pre_echo = .08f; v.drum_eccentricity = .2f; }},
        {"DEGRADED",     [](EngineParams& p, VideoParams& v) { p = {}; p.ips_base = 7.5f; p.wow_dep = 4.f; p.flutter_dep = .8f; p.motor_health = .8f; p.sticky_shed = .08f; p.oxide_shedding = .3f; p.dropout_rate = .05f; p.demagnetization = .7f; p.cutoff_base = 3000; p.hiss = .015f; p.hiss_color = .3f; p.mains_hum = .025f; v = {}; v.helical_sweep = .7f; v.head_switch_jitter = .45f; v.fm_carrier_noise = .3f; v.chroma_crosstalk = .4f; v.inter_field_phase_error = .2f; v.head_pre_echo = .07f; v.drum_eccentricity = .15f; }},
    };
    for (auto& pr : prs) {
        if (ImGui::Button(pr.n, {colW - 18, 0})) {
            pr.fn(app.baseParams, app.videoParams);
            std::lock_guard<std::mutex> lk2(app.pp.mu);
            app.pp.epSnap = app.baseParams;
            app.pp.vpSnap = app.videoParams;
            app.pp.tapeSpd = app.baseParams.tape_speed_mult;
            app.pp.engValid.store(true);
        }
    }

    sectionHdr("AUDIO METERS");
    ImGui::Text("L:");
    ImGui::SameLine();
    vuBar(app.vuL, colW - 46, 10, app.vuL > .95f);
    ImGui::Text("R:");
    ImGui::SameLine();
    vuBar(app.vuR, colW - 46, 10, app.vuR > .95f);

    sectionHdr("RENDER EXPORT");
    static int fmtIdx = 0;
    const char* fmts[] = {"MP4 (H.264 Lossy)", "MKV (FFV1 Lossless)", "AVI (FFV1 Lossless)"};
    ImGui::SetNextItemWidth(colW - 18);
    ImGui::Combo("##fmt", &fmtIdx, fmts, 3);

    if (!app.exportCtx.active) {
        if (ImGui::Button("START RECORDING TO DISK", {colW - 18, 0})) {
            app.exportCtx.format = (ExportFormat)fmtIdx;
            std::string runId = std::to_string(SDL_GetTicks());
            app.exportCtx.tempVideoPath = "temp_v_" + runId + ".avi";
            app.exportCtx.tempAudioPath = "temp_a_" + runId + ".wav";

            int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

            {
                std::lock_guard<std::mutex> wlk(app.exportCtx.writerMu);
                if (app.exportCtx.writer.open(app.exportCtx.tempVideoPath, fourcc, kNTSC_FPS, {FW, FH})) {
                    app.exportCtx.active = true;
                }
            }

            if (app.exportCtx.active) {
                app.exportCtx.frameCount = 0;
                startGlobalAudioCapture(app, app.exportCtx.tempAudioPath);
                SDL_Log("Export: Started recording intermediate to %s", app.exportCtx.tempVideoPath.c_str());
            } else {
                SDL_Log("Export: Failed to open VideoWriter");
            }
        }
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, {.8f, 0, 0, 1});
        if (ImGui::Button("STOP & FINISH EXPORT", {colW - 18, 40})) {
            SDL_Log("Export: Stop button pressed");
            app.exportCtx.isClosing = true;

            while (app.exportCtx.active) {
                SDL_Delay(10);
            }

            SDL_Log("Export: Finalizing audio capture...");
            stopGlobalAudioCapture(app);

            SDL_Log("Export: Releasing VideoWriter...");
            {
                std::lock_guard<std::mutex> wlk(app.exportCtx.writerMu);
                app.exportCtx.writer.release();
            }

            SDL_Log("Export: Finalizing filenames...");
            std::string outName = "render_export_" + std::to_string(SDL_GetTicks()) +
                                  ((app.exportCtx.format == ExportFormat::H264_MP4) ? ".mp4" :
                                   (app.exportCtx.format == ExportFormat::FFV1_MKV ? ".mkv" : ".avi"));

            char muxCmd[4096];
            bool hasAudio = false;
            long audioSz = 0;
            SDL_Log("Export: Checking audio file %s", app.exportCtx.tempAudioPath.c_str());
            std::FILE* test = std::fopen(app.exportCtx.tempAudioPath.c_str(), "rb");
            if (test) {
                std::fseek(test, 0, SEEK_END);
                audioSz = std::ftell(test);
                std::fclose(test);
                if (audioSz > 4096) hasAudio = true;
            }

            SDL_Log("Export: Capture summary - Video frames: %d, Audio: %ld bytes",
                app.exportCtx.frameCount, audioSz);

            if (hasAudio) {
                const char* aCodec = (app.exportCtx.format == ExportFormat::H264_MP4) ? "aac" : "copy";
                std::snprintf(muxCmd, sizeof(muxCmd),
                    "ffmpeg -y -i \"%s\" -i \"%s\" -map 0:v:0 -map 1:a:0 -c:v copy -c:a %s -shortest -aspect 4:3 \"%s\"",
                    app.exportCtx.tempVideoPath.c_str(), app.exportCtx.tempAudioPath.c_str(), aCodec, outName.c_str());
            } else {
                std::snprintf(muxCmd, sizeof(muxCmd),
                    "ffmpeg -y -i \"%s\" -c:v copy -aspect 4:3 \"%s\"",
                    app.exportCtx.tempVideoPath.c_str(), outName.c_str());
            }

            SDL_Log("Export: Running mux command: %s", muxCmd);
            if (std::system(muxCmd) == 0) {
                SDL_Log("Export: SUCCESS -> %s", outName.c_str());
                std::remove(app.exportCtx.tempVideoPath.c_str());
                std::remove(app.exportCtx.tempAudioPath.c_str());
            } else {
                SDL_Log("Export: FAILED (system call returned non-zero)");
            }
        }
        ImGui::PopStyleColor();
        ImGui::ProgressBar(float(app.exportCtx.frameCount % 100) / 100.f, {colW - 18, 0}, "RECORDING...");
    }

    ImGui::EndChild();

    sectionHdr("SYNC STATUS");
    {
        int64_t ap = g_audioSamplePos.load();
        float sf = g_sourceFPS.load();
        int64_t vf = sf > 0 ? int64_t(double(ap) / (AUDIO_SR / sf)) : 0;
        ImGui::Text("AUDIO POS: %8lld smp", (long long)ap);
        ImGui::Text("VIDEO FRM: %8lld", (long long)vf);
        ImGui::Text("SRC FPS:   %6.2f  (NTSC %.2f)", (double)sf, kNTSC_FPS);
        ImGui::Text("PROC FPS:  %6.1f", (double)app.pipeline.fps());
        ImGui::Text("H-SYNC:    %.2f Hz", kNTSC_HSYNC);
        ImGui::Text("V-SYNC:    %.2f Hz", kNTSC_FPS * 2.f);
        bool locked = false;
        {
            std::lock_guard<std::mutex> a_lk(app.audioMu);
            locked = app.tapeEngine && app.audioIO && app.audioIO->is_open();
        }
        if (locked) ImGui::TextColored({0.f, .9f, .4f, 1.f}, "● A/V LOCKED");
        else       ImGui::TextColored({.9f, .3f, .3f, 1.f}, "○ FREE-RUN");
    }
    ImGui::EndChild();
    ImGui::End();
}
