cmake_minimum_required (VERSION 3.0)
project(myproject)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (example Quintic.cpp)
add_executable (InverseKinematics Quintic.cpp)
target_link_libraries (example Eigen3::Eigen)
target_link_libraries (InverseKinematics Eigen3::Eigen)