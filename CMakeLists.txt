cmake_minimum_required(VERSION 3.14)
project(main)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/third_party/imgui)
set(IMPLOT_DIR ${PROJECT_SOURCE_DIR}/third_party/implot)
set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/source)

# FFTW3
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F REQUIRED fftw3f)

link_directories(${FFTW3F_LIBRARY_DIRS})

find_package(SoapySDR REQUIRED)

# ImGui
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# ImPlot
set(IMPLOT_SOURCES
  ${IMPLOT_DIR}/implot.cpp
  ${IMPLOT_DIR}/implot_demo.cpp
  ${IMPLOT_DIR}/implot_items.cpp
)

set(MAIN_SOURCES
  ${SOURCE_DIR}/modulation.cpp
  ${SOURCE_DIR}/common.cpp
  ${SOURCE_DIR}/sdr.cpp
  ${SOURCE_DIR}/dsp.cpp
  ${SOURCE_DIR}/gui.cpp
  ${SOURCE_DIR}/main.cpp
)

add_executable(${PROJECT_NAME}
  ${MAIN_SOURCES}
  ${IMGUI_SOURCES}
  ${IMPLOT_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${IMGUI_DIR}
  ${IMGUI_DIR}/backends
  ${IMPLOT_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${SoapySDR_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  GL_SILENCE_DEPRECATION
)

target_link_libraries(${PROJECT_NAME}
  glfw
  OpenGL::GL
  ${FFTW3F_LIBRARIES}
  SoapySDR
)
