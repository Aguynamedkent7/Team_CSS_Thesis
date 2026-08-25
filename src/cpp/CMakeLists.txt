cmake_minimum_required(VERSION 3.15)
project(AssettoCorsa_Enhanced_SCR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
)
FetchContent_MakeAvailable(osqp)

# Fetch OsqpEigen
set(BUILD_TESTING OFF CACHE BOOL "Disable tests" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Disable shared libs" FORCE)
FetchContent_Declare(OsqpEigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  GIT_TAG        v0.8.1
)
FetchContent_MakeAvailable(OsqpEigen)

add_executable(ac_agent
    main.cpp
    connector/ACSharedMemory.cpp
    connector/VJoyFeeder.cpp
    controller/SCRController.cpp
    controller/ActuatorMapper.cpp
    model/SingleTrack.cpp
    utils/CSVLoader.cpp
)

target_include_directories(ac_agent PRIVATE ${eigen_SOURCE_DIR})
target_link_libraries(ac_agent PRIVATE osqp::osqp OsqpEigen::OsqpEigen)
