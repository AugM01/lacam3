cmake_minimum_required(VERSION 3.16)
project(lacam-project CXX)

# Find zstd robustly. The previous hardcoded /usr/lib/x86_64-linux-gnu/libzstd.so (and the
# implicit /usr/include for zstd.h) does not exist on the LIDS cluster, where zstd is built
# from source into ~/.local. Mirror the parent project's discovery (FICO-clean/CMakeLists.txt):
# prefer ~/.local, fall back to system paths, and expose both the lib and the include dir.
find_library(ZSTD_LIBRARY
    NAMES zstd
    PATHS $ENV{HOME}/.local/lib /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
    REQUIRED
)
if(EXISTS "$ENV{HOME}/.local/include/zstd.h")
    set(ZSTD_INCLUDE_DIR "$ENV{HOME}/.local/include")
elseif(EXISTS "/usr/local/include/zstd.h")
    set(ZSTD_INCLUDE_DIR "/usr/local/include")
else()
    set(ZSTD_INCLUDE_DIR "/usr/include")
endif()
message(STATUS "zstd include: ${ZSTD_INCLUDE_DIR}")
message(STATUS "zstd library: ${ZSTD_LIBRARY}")
include_directories(${ZSTD_INCLUDE_DIR})

add_subdirectory(./lacam3)
add_subdirectory(./third_party/argparse)

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main lacam3 argparse ${ZSTD_LIBRARY})

# test
enable_testing()
file(GLOB TEST_FILES "./tests/test_*.cpp")
foreach(file ${TEST_FILES})
  string(REGEX MATCH "test\_[^\.]+" name "${file}")
  add_executable(${name} ${file})
  target_link_libraries(${name} lacam3 ${ZSTD_LIBRARY})
  add_test(${name} ${name})
endforeach()
