cmake_minimum_required(VERSION 3.10.0)
project(pms5003 VERSION 0.1.0 LANGUAGES C)

add_library(pms5003 pms5003.c)

add_executable(pms_read pms_read.c)
target_link_libraries(pms_read PRIVATE pms5003)