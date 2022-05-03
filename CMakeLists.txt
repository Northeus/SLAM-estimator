cmake_minimum_required(VERSION 3.10)
project(Estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(GTSAM REQUIRED)

add_executable(estimator
    estimator/main.cpp
    estimator/IMUMeasurement.h
    estimator/StereoMeasurement.h
    estimator/Estimator.h
    estimator/Estimator.cpp
    estimator/SnapdragonEstimator.h
    estimator/SnapdragonEstimator.cpp
    estimator/SnapdragonParser.h
    estimator/SnapdragonParser.cpp
)
target_link_libraries(estimator gtsam)

file(
    COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/imu.txt
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

file(
    COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/projections.csv
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
