cmake_minimum_required(VERSION 2.8.3)
project(quintic-polynomial-2D-trajctory)
set(CMAKE_CXX_STANDARD 14)

# for plotting

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
  ${Python3_INCLUDE_DIRS}
  include
)


find_package(Eigen3 REQUIRED)

include_directories(
  ${Python3_INCLUDE_DIRS}
  include
)

add_executable(main
  src/main.cpp
  src/class_one_dimen_poly.cpp  
  src/class_two_dimen_poly.cpp
)


target_link_libraries(main
  ${Python3_LIBRARIES}
  Eigen3::Eigen
)

