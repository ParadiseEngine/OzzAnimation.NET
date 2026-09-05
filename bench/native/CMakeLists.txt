# Builds the shim the benchmark measures native ozz-animation through.
#
#   cmake -B build -DOZZ_ROOT=/path/to/ozz-animation -DOZZ_BUILD=/path/to/ozz-animation/build
#   cmake --build build --config Release
#
# then point OZZ_NATIVE_LIB at the resulting libozz_shim.dylib/.so/.dll when running the benchmark.
cmake_minimum_required(VERSION 3.16)
project(ozz_shim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED OZZ_ROOT)
  message(FATAL_ERROR "Set -DOZZ_ROOT to an ozz-animation checkout (0.17).")
endif()
if(NOT DEFINED OZZ_BUILD)
  set(OZZ_BUILD "${OZZ_ROOT}/build")
endif()

add_library(ozz_shim SHARED OzzShim.cpp)
target_include_directories(ozz_shim PRIVATE "${OZZ_ROOT}/include")
target_link_libraries(ozz_shim PRIVATE
  "${OZZ_BUILD}/src/animation/runtime/libozz_animation.a"
  "${OZZ_BUILD}/src/base/libozz_base.a")
