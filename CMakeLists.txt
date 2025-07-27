CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
project(auto-aim)
SET(exe_name auto-aim)
set(CMAKE_CXX_STANDARD 20)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

option(USE_HIKCAM "Build with HikCAM Lib" ON)

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory(UartIMU uartimu_src)
aux_source_directory(KeyBoard key_board_src)
aux_source_directory(LcdScreen lcd_screen_src)

include_directories(${PROJECT_SOURCE_DIR}/UartIMU)
include_directories(${PROJECT_SOURCE_DIR}/KeyBoard)
include_directories(${PROJECT_SOURCE_DIR}/LcdScreen)

add_executable(${exe_name} main.cpp ${uartimu_src} ${key_board_src} ${lcd_screen_src})

include_directories(common)

add_subdirectory(common)

target_link_libraries(${exe_name} common)

target_link_libraries(${exe_name} ${OpenCV_LIBS} RMCVSerial wiringPi wiringPiDev)


