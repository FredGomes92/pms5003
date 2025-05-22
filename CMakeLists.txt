cmake_minimum_required(VERSION 3.10)
project(pms5003 VERSION 0.1.0 LANGUAGES C)

# Create the shared library from the C file
add_library(pms5003 SHARED pms5003.c)

# Set the version info
set_target_properties(pms5003 PROPERTIES
    VERSION ${PROJECT_VERSION}
    PUBLIC_HEADER pms5003.h
)

# Install the library, header, and versioned symlink
install(TARGETS pms5003
    LIBRARY DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# Create an example binary for testing
add_executable(pms_read pms_read.c)
target_link_libraries(pms_read PRIVATE pms5003)

# Optional: install the example binary
install(TARGETS pms_read
    RUNTIME DESTINATION bin
)
