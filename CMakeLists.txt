cmake_minimum_required(VERSION 3.10)
project(rts2-origin-drivers)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find RTS2
set(RTS2_INCLUDE_DIR "/home/jonathan/src/rts2/include" CACHE PATH "Path to RTS2 include directory")
set(RTS2_SRC_DIR "/home/jonathan/src/rts2" CACHE PATH "Path to RTS2 source directory")

# Link directly against RTS2 libraries in the build tree
# RTS2 telescope drivers need both librts2 and the telescope-specific libraries
set(RTS2_LIB_DIR "${RTS2_SRC_DIR}/lib")

# Find the main RTS2 library
find_library(RTS2_LIB
    NAMES rts2 librts2
    PATHS
        ${RTS2_LIB_DIR}
        ${RTS2_SRC_DIR}/.libs
        /usr/local/lib
        /usr/lib
        /usr/lib/x86_64-linux-gnu
)

if(NOT RTS2_LIB)
    message(FATAL_ERROR "RTS2 library not found! Searched in:\n  ${RTS2_LIB_DIR}\n  ${RTS2_SRC_DIR}/.libs\n  /usr/local/lib")
endif()

message(STATUS "Found RTS2 library: ${RTS2_LIB}")

# Find required libraries
find_package(OpenSSL REQUIRED)
find_package(CURL REQUIRED)

# Find libnova
find_library(NOVA_LIB
    NAMES nova
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
    REQUIRED
)

# Find cfitsio
find_library(CFITSIO_LIB
    NAMES cfitsio
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
    REQUIRED
)

# Include directories
include_directories(
    ${RTS2_INCLUDE_DIR}
    ${OPENSSL_INCLUDE_DIR}
    ${CURL_INCLUDE_DIRS}
)

# Link directories for RTS2 (important for finding all RTS2 libs)
link_directories(
    ${RTS2_LIB_DIR}
    ${RTS2_SRC_DIR}/.libs
)

# Common source files
set(WEBSOCKET_SOURCES
    origin-websocket.cpp
)

# Telescope driver
add_executable(rts2-teld-origin
    origin-teld.cpp
    ${WEBSOCKET_SOURCES}
)

# The telescope driver needs both librts2 and librts2tel
find_library(RTS2TEL_LIB
    NAMES rts2tel
    PATHS
        ${RTS2_LIB_DIR}/rts2tel/.libs
        ${RTS2_SRC_DIR}/lib/rts2tel/.libs
        /usr/local/lib
)

if(NOT RTS2TEL_LIB)
    message(WARNING "librts2tel not found, trying to link anyway...")
    set(RTS2TEL_LIB rts2tel)
endif()

message(STATUS "Found RTS2TEL library: ${RTS2TEL_LIB}")

target_link_libraries(rts2-teld-origin
    ${RTS2TEL_LIB}
    ${RTS2_LIB}
    ${OPENSSL_LIBRARIES}
    ${NOVA_LIB}
    pthread
)

# Camera driver
add_executable(rts2-camd-origin
    origin-camd.cpp
    ${WEBSOCKET_SOURCES}
)

find_package(TIFF REQUIRED)

target_link_libraries(rts2-camd-origin
    ${RTS2_LIB}
    ${OPENSSL_LIBRARIES}
    ${CURL_LIBRARIES}
    ${CFITSIO_LIB}
    ${TIFF_LIBRARIES}
    pthread
)

# Installation
install(TARGETS rts2-teld-origin rts2-camd-origin
    RUNTIME DESTINATION /usr/local/bin
)

include_directories(/home/jonathan/src/rts2/include)

add_executable(rts2-sensor-alwaysgood
    rts2-sensor-alwaysgood.cpp
)

target_link_libraries(rts2-sensor-alwaysgood
    ${RTS2_LIB}
    pthread
)

add_executable(rts2-dome-virtual rts2-dome-virtual.cpp)
target_include_directories(rts2-dome-virtual PRIVATE /home/jonathan/src/rts2/include)

# Link to the same RTS2 core libs as your other daemons.
# If your current daemons link just by librts2, reuse that.
target_link_libraries(rts2-dome-virtual PRIVATE rts2 pthread)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O0")

# Print configuration summary
message(STATUS "")
message(STATUS "==============================================")
message(STATUS "Configuration Summary for Origin Drivers")
message(STATUS "==============================================")
message(STATUS "  RTS2 Include:    ${RTS2_INCLUDE_DIR}")
message(STATUS "  RTS2 Library:    ${RTS2_LIB}")
message(STATUS "  OpenSSL:         ${OPENSSL_LIBRARIES}")
message(STATUS "  CURL:            ${CURL_LIBRARIES}")
message(STATUS "  Nova:            ${NOVA_LIB}")
message(STATUS "  CFITSIO:         ${CFITSIO_LIB}")
message(STATUS "==============================================")
message(STATUS "")

