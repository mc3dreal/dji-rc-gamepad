cmake_minimum_required(VERSION 3.16)
project(dji-rc-linux C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Curses REQUIRED)

add_executable(dji-rc-joystick dji_rc_joystick.c)
target_include_directories(dji-rc-joystick PRIVATE ${CURSES_INCLUDE_DIRS})
target_link_libraries(dji-rc-joystick ${CURSES_LIBRARIES})
