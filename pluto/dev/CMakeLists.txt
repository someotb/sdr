cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/third_party)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

find_package(SoapySDR REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${INCLUDE_DIR})
include_directories(${SDL2_INCLUDE_DIRS})

add_library(imgui STATIC
    ${THIRD_PARTY_DIR}/imgui/imgui.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_draw.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_demo.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_tables.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_widgets.cpp
    ${THIRD_PARTY_DIR}/imgui/backends/imgui_impl_sdl2.cpp
    ${THIRD_PARTY_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${THIRD_PARTY_DIR}/imgui)
target_compile_options(imgui PRIVATE -fPIC)
target_link_libraries(imgui PRIVATE ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

add_library(implot STATIC
    ${THIRD_PARTY_DIR}/implot/implot.cpp
    ${THIRD_PARTY_DIR}/implot/implot_demo.cpp
    ${THIRD_PARTY_DIR}/implot/implot_items.cpp
)

target_include_directories(implot PUBLIC ${THIRD_PARTY_DIR}/implot)
target_link_libraries(implot PRIVATE imgui)

set(MAIN_SOURCE_FILES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/modulation.cpp
)

add_executable(main ${MAIN_SOURCE_FILES})

target_link_libraries(main PRIVATE
    imgui
    implot
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${SoapySDR_LIBRARIES}
)
