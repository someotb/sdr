cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/third_party)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

# SDL2
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

# OpenGL/GLEW
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

include_directories(${INCLUDE_DIR})

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

add_library(implot STATIC
    ${THIRD_PARTY_DIR}/implot/implot.cpp
    ${THIRD_PARTY_DIR}/implot/implot_demo.cpp
    ${THIRD_PARTY_DIR}/implot/implot_items.cpp
)

# Указываем пути к заголовочным файлам относительно корня проекта
include_directories(${PROJECT_SOURCE_DIR}/include)

# Ищем библиотеку SoapySDR
find_package(SoapySDR REQUIRED)

# Указываем исходные файлы с правильными путями
set(MAIN_SOURCE_FILES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/modulation.cpp
    ${SRC_DIR}/sdr.cpp
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Добавляем исполняемый файл
add_executable(main ${MAIN_SOURCE_FILES})

# Линкуем библиотеки
target_link_libraries(main PRIVATE ${SoapySDR_LIBRARIES})

# Добавляем директорию include в путь поиска заголовков для индексации
target_include_directories(main PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_include_directories(implot PUBLIC ${THIRD_PARTY_DIR}/implot)
target_link_libraries(implot PRIVATE imgui)
target_compile_options(implot PRIVATE -fPIC)
target_link_libraries(main PRIVATE imgui implot ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
