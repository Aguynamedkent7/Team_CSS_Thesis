cmake_minimum_required(VERSION 3.24)
project(AssettoCorsa_Enhanced_SCR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This prevents OSQP's older CMakeLists from throwing policy errors in CMake 4.x
set(CMAKE_POLICY_VERSION_MINIMUM 3.24)

include(FetchContent)

# Fetch Eigen3
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

# Fetch OSQP
set(OSQP_BUILD_SHARED_LIBS OFF CACHE BOOL "Disable shared OSQP" FORCE)
set(OSQP_BUILD_TESTING OFF CACHE BOOL "Disable OSQP tests" FORCE)
FetchContent_Declare(osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG        v0.6.3
  OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(osqp)

add_library(osqp::osqp INTERFACE IMPORTED GLOBAL)
target_link_libraries(osqp::osqp INTERFACE osqpstatic)

# Trick to satisfy OsqpEigen's find_package(osqp)
file(WRITE ${CMAKE_BINARY_DIR}/osqp-config.cmake "
set(osqp_FOUND TRUE)
")
set(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR} ${CMAKE_PREFIX_PATH})

# Make sure OsqpEigen can find osqp.h
include_directories(${osqp_SOURCE_DIR}/include)
include_directories(${CMAKE_BINARY_DIR}/_deps/osqp-build/out/include)

# Fetch OsqpEigen
set(BUILD_TESTING OFF CACHE BOOL "Disable tests" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Disable shared libs" FORCE)
FetchContent_Declare(OsqpEigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  GIT_TAG        v0.8.1
  OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(OsqpEigen)

# Fetch Raylib for GUI
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.0
)
FetchContent_MakeAvailable(raylib)

add_executable(ac_agent
    main.cpp
    connector/ACSharedMemory.cpp
    connector/VJoyFeeder.cpp
    controller/SCRController.cpp
    controller/ActuatorMapper.cpp
    model/SingleTrack.cpp
    utils/CSVLoader.cpp
)

target_include_directories(ac_agent PRIVATE ${eigen_SOURCE_DIR} ${osqpeigen_SOURCE_DIR}/include)
target_link_libraries(ac_agent PRIVATE osqpstatic OsqpEigen raylib)
