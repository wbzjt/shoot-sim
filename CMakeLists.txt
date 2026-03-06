cmake_minimum_required(VERSION 3.22)
project(ShooterSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SHOOTERSIM_BUILD_GUI "Build ShooterSim GUI application" ON)
option(SHOOTERSIM_BUILD_TESTS "Build ShooterSim tests" ON)

if(MSVC)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()

include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.3.9
)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90.7
)

FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG v0.16
)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(glfw nlohmann_json)

FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
  FetchContent_Populate(imgui)
endif()

FetchContent_GetProperties(implot)
if(NOT implot_POPULATED)
  FetchContent_Populate(implot)
endif()

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui PUBLIC glfw)

add_library(implot STATIC
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
)

target_include_directories(implot PUBLIC
  ${implot_SOURCE_DIR}
)

target_link_libraries(implot PUBLIC imgui)

find_package(OpenGL REQUIRED)

add_library(shootersim_core STATIC
  src/model/Geometry.cpp
  src/model/Ballistics.cpp
  src/model/Constraints.cpp
  src/model/Search.cpp
  src/model/Sweep.cpp
  src/model/Export.cpp
  src/util/MathUtil.cpp
  src/util/CsvUtil.cpp
  src/util/JsonUtil.cpp
)

target_include_directories(shootersim_core PUBLIC src)

target_link_libraries(shootersim_core PUBLIC nlohmann_json::nlohmann_json)

if(MSVC)
  target_compile_options(shootersim_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(shootersim_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(SHOOTERSIM_BUILD_GUI)
  add_executable(ShooterSim
    src/main.cpp
    src/app/App.cpp
    src/gui/Panels.cpp
    src/gui/PlotHelpers.cpp
  )

  target_link_libraries(ShooterSim PRIVATE
    shootersim_core
    imgui
    implot
    glfw
    OpenGL::GL
  )

  target_include_directories(ShooterSim PRIVATE
    ${implot_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
  )

  if(MSVC)
    target_compile_options(ShooterSim PRIVATE /W4 /permissive-)
  else()
    target_compile_options(ShooterSim PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

if(SHOOTERSIM_BUILD_TESTS)
  add_executable(ShooterSimTests
    tests/TestMain.cpp
    tests/test_ballistics.cpp
    tests/test_constraints.cpp
    tests/test_search.cpp
  )

  target_link_libraries(ShooterSimTests PRIVATE shootersim_core)

  if(MSVC)
    target_compile_options(ShooterSimTests PRIVATE /W4 /permissive-)
  else()
    target_compile_options(ShooterSimTests PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  enable_testing()
  add_test(NAME ShooterSimTests COMMAND ShooterSimTests)
endif()
