cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 17)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project("crowd-sim")

add_subdirectory("vendor/raylib-6.0")

# ImGui
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/vendor/imgui")
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR})

# rlImGui (raylib-imgui bridge)
set(RLIMGUI_DIR "${CMAKE_SOURCE_DIR}/vendor/rlimgui")
add_library(rlimgui STATIC
    ${RLIMGUI_DIR}/rlImGui.cpp
)
target_include_directories(rlimgui PUBLIC ${RLIMGUI_DIR})
target_link_libraries(rlimgui PUBLIC imgui raylib)

add_executable(crowd-sim
    src/main.cpp
    src/game.cpp
    src/simulation/engine.cpp
        src/simulation/engine.h
        src/game.h
        src/ui.cpp
        src/ui.h
        src/simulation/config.h
        src/simulation/agent.h
        src/simulation/processors/steering.h
        src/simulation/processors/steering.cpp
        src/simulation/processors/movement.h
        src/simulation/processors/movement.cpp
        src/simulation/processors/avoidance.h
        src/simulation/processors/avoidance.cpp
        src/simulation/processors/arrival.cpp
        src/simulation/processors/arrival.h
)
target_link_libraries(crowd-sim PRIVATE raylib rlimgui imgui)
target_include_directories(crowd-sim PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/vendor/imgui"
    "${CMAKE_SOURCE_DIR}/vendor/rlimgui"
)

add_custom_target(copy_assets
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}/assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
)
add_dependencies(crowd-sim copy_assets)

if(${PLATFORM} STREQUAL "Web")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s USE_GLFW=3 -s ASSERTIONS=1 -s WASM=1 -Os -Wall -s TOTAL_MEMORY=67108864 -s FORCE_FILESYSTEM=1 --preload-file ${CMAKE_SOURCE_DIR}/assets@/assets --shell-file ${CMAKE_SOURCE_DIR}/src/minshell.html")
    set_target_properties(crowd-sim PROPERTIES SUFFIX ".html" OUTPUT_NAME "index")
endif()
