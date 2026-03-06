cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
project(pact_go1_deploy)
SET(CMAKE_CXX_STANDARD 17)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

# LibTorch Stuff
set(CMAKE_PREFIX_PATH /home/hcr/.local/lib/python3.10/site-packages/torch/share/cmake)  # Change the path to your LibTorch

# Unitree-Legged-SDK (Go1 version)
set(UNITREE_SDK_PATH ${CMAKE_SOURCE_DIR}/3rd_parties/unitree_legged_sdk/lib/cpp/arm64/)
message("-- UNITREE_SDK_PATH: ${UNITREE_SDK_PATH}")

find_library(UNITREE_SDK_LIB unitree_legged_sdk PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

find_package(Torch REQUIRED)

include_directories(
    include 
    ${TORCH_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/3rd-parties/unitree_legged_sdk/include/)


set(headers
  src/HardwareBridge.hpp)

set(sources
    src/HardwareBridge.cpp)

add_library(go1_system SHARED ${headers} ${sources})

add_executable(pact_deploy 
src/main.cpp)

target_link_libraries(pact_deploy
    go1_system
    ${UNITREE_SDK_LIB}
    yaml-cpp
    ${TORCH_LIBRARIES}
    )
