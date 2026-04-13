cmake_minimum_required(VERSION 3.18)
project(ADO8500 VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Options ────────────────────────────────────────────────
option(ADO_OPENMP   "Enable OpenMP intra-frame parallelism" ON)
option(ADO_ASAN     "Enable AddressSanitizer (debug only)"  OFF)
option(ADO_TSAN     "Enable ThreadSanitizer  (debug only)"  OFF)

# ── Required packages ──────────────────────────────────────
find_package(SDL2  REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
    libswresample
)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
find_package(SndFile REQUIRED)

if(ADO_OPENMP)
    find_package(OpenMP)
endif()

# ── CapstanVar library ─────────────────────────────────────
set(CAPSTANVAR_ROOT "/mnt/MassArchive/Programs/Tape Warping/CapstanVar")
set(CAPSTANVAR_INCLUDE "${CAPSTANVAR_ROOT}/include")

# ── Fetch Dear ImGui ────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.90.6
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_SRC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
)

# ── Executable ──────────────────────────────────────────────
set(CAPSTANVAR_SRC
    "${CAPSTANVAR_ROOT}/src/engine.cpp"
    "${CAPSTANVAR_ROOT}/src/audio_io.cpp"
    "${CAPSTANVAR_ROOT}/src/mod_electronics.cpp"
    "${CAPSTANVAR_ROOT}/src/mod_magnetic.cpp"
    "${CAPSTANVAR_ROOT}/src/mod_transport.cpp"
)

add_executable(ado8500
    src/main.cpp
    ${IMGUI_SRC}
    ${CAPSTANVAR_SRC}
)

target_compile_definitions(ado8500 PRIVATE 
    ADO_OPENMP 
    NAGRA_AUDIO_PORTAUDIO
)

target_include_directories(ado8500 PRIVATE
    ${SDL2_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${FFMPEG_INCLUDE_DIRS}
    ${CAPSTANVAR_INCLUDE}
    ${PORTAUDIO_INCLUDE_DIRS}
)

target_link_libraries(ado8500 PRIVATE
    ${SDL2_LIBRARIES}
    ${OpenCV_LIBS}
    ${CMAKE_DL_LIBS}
    PkgConfig::FFMPEG
    ${PORTAUDIO_LIBRARIES}
    SndFile::sndfile
    pthread
)

# OpenMP intra-frame row parallelism
if(ADO_OPENMP AND OpenMP_CXX_FOUND)
    target_link_libraries(ado8500 PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(ado8500 PRIVATE ADO_OPENMP)
    message(STATUS "ADO-8500: OpenMP enabled (${OpenMP_CXX_VERSION})")
endif()

# ── Per-platform setup ──────────────────────────────────────
if(APPLE)
    target_link_libraries(ado8500 PRIVATE "-framework Cocoa")
endif()

if(WIN32)
    # Disable console window in Release
    set_target_properties(ado8500 PROPERTIES
        WIN32_EXECUTABLE $<CONFIG:Release>)
endif()

# ── Compiler flags ──────────────────────────────────────────
if(MSVC)
    target_compile_options(ado8500 PRIVATE
        $<$<CONFIG:Release>:/O2 /arch:AVX2 /fp:fast>
        $<$<CONFIG:Debug>:/Od /Zi>
        /W3
    )
else()
    target_compile_options(ado8500 PRIVATE
        $<$<CONFIG:Release>:-O3 -march=native -ffast-math -funroll-loops>
        $<$<CONFIG:Debug>:-O0 -g>
        -Wall -Wextra -Wno-unused-parameter
    )
    if(ADO_ASAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(ado8500 PRIVATE -fsanitize=address)
        target_link_options(ado8500 PRIVATE -fsanitize=address)
    endif()
    if(ADO_TSAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_options(ado8500 PRIVATE -fsanitize=thread)
        target_link_options(ado8500 PRIVATE -fsanitize=thread)
    endif()
endif()

message(STATUS "ADO-8500 configured. Build with:")
message(STATUS "  cmake --build . --config Release")
