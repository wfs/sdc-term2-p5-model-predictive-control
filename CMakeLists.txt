#cmake_minimum_required(VERSION 3.7)
cmake_minimum_required(VERSION 3.5)
project(sdc_term2_p5_model_predictive_control)

set(CMAKE_CXX_STANDARD 11)

# BDD-Style Tests
set(SOURCE_FILES test_main.cpp)

#set(SOURCE_FILES main.cpp)

add_executable(sdc_term2_p5_model_predictive_control ${SOURCE_FILES})