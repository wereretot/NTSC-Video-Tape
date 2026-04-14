#include "audio_capture.h"
#include "processing_pipeline.h"
#include <cstdio>

void startGlobalAudioCapture(AppState& app, const std::string& path) {
    std::lock_guard<std::mutex> lk(app.exportCtx.writerMu);
    app.exportCtx.audioFp = std::fopen(path.c_str(), "wb");
    if (app.exportCtx.audioFp) {
        app.exportCtx.audioFramesWritten = 0;
        app.exportCtx.isClosing = false;
        writeWavHeader(app.exportCtx.audioFp, 0);
    }
}

void stopGlobalAudioCapture(AppState& app) {
    std::lock_guard<std::mutex> lk(app.exportCtx.writerMu);
    if (app.exportCtx.audioFp) {
        std::fseek(app.exportCtx.audioFp, 0, SEEK_SET);
        writeWavHeader(app.exportCtx.audioFp, app.exportCtx.audioFramesWritten);
        std::fclose(app.exportCtx.audioFp);
        app.exportCtx.audioFp = nullptr;
    }
}

void writeWavHeader(FILE* f, uint32_t total_frames) {
    uint32_t data_size = total_frames * 2 * sizeof(int16_t);
    uint32_t chunk_size = 36 + data_size;
    std::fwrite("RIFF", 1, 4, f);
    std::fwrite(&chunk_size, 4, 1, f);
    std::fwrite("WAVEfmt ", 1, 8, f);
    uint32_t subchunk1_size = 16;
    uint16_t audio_format = 1;
    uint16_t num_channels = 2;
    uint32_t sample_rate = SR;
    uint32_t byte_rate = sample_rate * num_channels * 2;
    uint16_t block_align = num_channels * 2;
    uint16_t bits_per_sample = 16;
    std::fwrite(&subchunk1_size, 4, 1, f);
    std::fwrite(&audio_format, 2, 1, f);
    std::fwrite(&num_channels, 2, 1, f);
    std::fwrite(&sample_rate, 4, 1, f);
    std::fwrite(&byte_rate, 4, 1, f);
    std::fwrite(&block_align, 2, 1, f);
    std::fwrite(&bits_per_sample, 2, 1, f);
    std::fwrite("data", 1, 4, f);
    std::fwrite(&data_size, 4, 1, f);
}
