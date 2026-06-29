cmake_minimum_required(VERSION 3.16)
project(lacam-project CXX)

# Default to an optimized build, matching the other vendored baselines
# (PBS/EECBS/MAPF-LNS2 all carry this fallback). CMake's default RELEASE flags
# (-O3 -DNDEBUG) then apply. Without this, an unspecified CMAKE_BUILD_TYPE left the
# inner lib's $<$<CONFIG:Release>:-O3> generator expression inactive -> the baseline
# compiled at -O0 (4-10x slower) and lost the comparison unfairly to STRA's -O3 build.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "[lacam-project] CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

add_subdirectory(./lacam3)
add_subdirectory(./third_party/argparse)

add_executable(main main.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main lacam3 argparse /usr/lib/x86_64-linux-gnu/libzstd.so)

# test
enable_testing()
file(GLOB TEST_FILES "./tests/test_*.cpp")
foreach(file ${TEST_FILES})
  string(REGEX MATCH "test\_[^\.]+" name "${file}")
  add_executable(${name} ${file})
  target_link_libraries(${name} lacam3 /usr/lib/x86_64-linux-gnu/libzstd.so)
  add_test(${name} ${name})
endforeach()