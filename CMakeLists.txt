#cmake_minimum_required(VERSION 3.7)
cmake_minimum_required(VERSION 3.5)
project(sdc_term2_p5_model_predictive_control)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

#set(CMAKE_CXX_STANDARD 11)

# BDD-Style Tests
#set(SOURCE_FILES test_main.cpp)

#set(SOURCE_FILES main.cpp)

#add_executable(sdc_term2_p5_model_predictive_control ${SOURCE_FILES})

set(sources src/MPC.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)
