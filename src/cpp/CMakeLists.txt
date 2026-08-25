cmake_minimum_required(VERSION 3.15)
project(AssettoCorsa_Enhanced_SCR)

set(CMAKE_CXX_STANDARD 17)

# We will need to find Eigen3, OSQP, OsqpEigen and vJoy eventually
# For now, we compile the basic skeleton.

add_executable(ac_agent
    main.cpp
    connector/ACSharedMemory.cpp
    connector/VJoyFeeder.cpp
    controller/SCRController.cpp
    controller/ActuatorMapper.cpp
)

# target_include_directories(ac_agent PRIVATE ${EIGEN3_INCLUDE_DIRS} ${vJoy_INCLUDE_DIRS})
# target_link_libraries(ac_agent PRIVATE OsqpEigen::OsqpEigen osqp::osqp ${vJoy_LIBRARIES})
