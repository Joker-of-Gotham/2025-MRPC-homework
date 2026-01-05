cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_df)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

add_executable(df_quaternion src/df_quaternion.cpp)

target_link_libraries(df_quaternion
  ${catkin_LIBRARIES}
)
