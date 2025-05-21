cmake_minimum_required(VERSION 3.10.0)
project(pms5003 VERSION 0.1.0 LANGUAGES C DESCRIPTION "PMS5003 libary")

add_library(pms5003 SHARED pms5003.c)
set_target_properties(pms5003 PROPERTIES VERSION ${PROJECT_VERSION})

add_executable(pms_read pms_read.c)
target_link_libraries(pms_read PRIVATE pms5003)
