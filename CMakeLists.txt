message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
elseif("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(UNITREE_SDK_PATH ${CMAKE_SOURCE_DIR}/3rd_parties/unitree_legged_sdk/lib/cpp/${ARCH}/)
message("-- UNITREE_SDK_PATH: ${UNITREE_SDK_PATH}")

find_library(UNITREE_SDK_LIB unitree_legged_sdk PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif()

find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)

set(headers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HardwareBridge.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MujocoSimulationBridge.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/joystick/joystick.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PACTMJSimulationBridge.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/observers/LinearKFObserver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/observers/forward_kinematics.h
)

set(sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HardwareBridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MujocoSimulationBridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/joystick/joystick.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PACTMJSimulationBridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/observers/LinearKFObserver.cpp
)

add_library(go1_system SHARED ${headers} ${sources})

target_include_directories(go1_system
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/observers
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/3rd_parties/unitree_legged_sdk/include
        ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(go1_system
    PUBLIC
        ${TORCH_LIBRARIES}
        daros-utils
        mujoco
        glfw
)

add_executable(pact_deploy src/main.cpp)

target_link_libraries(pact_deploy
    PRIVATE
        go1_system
        ${UNITREE_SDK_LIB}
        yaml-cpp
        Threads::Threads
)

add_executable(pact_sim src/main_sim.cpp)

target_link_libraries(pact_sim
    PRIVATE
        go1_system
        ${UNITREE_SDK_LIB}
        yaml-cpp
        Threads::Threads
)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

# Optional checks; keep assertion checks enabled in Release builds as well.
add_executable(pact_policy_smoke EXCLUDE_FROM_ALL tests/pact_policy_smoke.cpp)
target_link_libraries(pact_policy_smoke PRIVATE go1_system ${UNITREE_SDK_LIB})
target_compile_options(pact_policy_smoke PRIVATE -UNDEBUG)

add_executable(position_controller_test EXCLUDE_FROM_ALL tests/position_controller_test.cpp)
target_link_libraries(position_controller_test PRIVATE go1_system ${UNITREE_SDK_LIB})
target_compile_options(position_controller_test PRIVATE -UNDEBUG -O0 -g0)
