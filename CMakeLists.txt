cmake_minimum_required(VERSION 3.10)
project(ROKAE_MPC)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_PREFIX_PATH "F:/ProgramData/anaconda3/envs/mpc_rokae/Library")

# 寻找预编译好的 Pinocchio 及其依赖
find_package(pinocchio CONFIG REQUIRED)

add_executable(mpc_node 
    test/test_dynamics.cpp
)

target_link_libraries(mpc_node PRIVATE pinocchio::pinocchio)