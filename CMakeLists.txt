cmake_minimum_required(VERSION 3.10)
project(Estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(GTSAM REQUIRED)

add_executable(estimator estimator/main.cpp)
target_link_libraries(estimator gtsam)
