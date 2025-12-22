cmake_minimum_required(VERSION 3.16)
project(PlutoSDR CXX)

set(CMAKE_BUILD_TYPE Debug)

# Указываем пути к заголовочным файлам относительно корня проекта
include_directories(${PROJECT_SOURCE_DIR}/include)

# Ищем библиотеку SoapySDR
find_package(SoapySDR REQUIRED)

# Указываем исходные файлы с правильными путями
set(MAIN_SOURCE_FILES
    main.cpp
    src/modulation.cpp
    src/sdr.cpp
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Добавляем исполняемый файл
add_executable(main ${MAIN_SOURCE_FILES})

# Линкуем библиотеки
target_link_libraries(main PRIVATE ${SoapySDR_LIBRARIES})

# Добавляем директорию include в путь поиска заголовков для индексации
target_include_directories(main PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
