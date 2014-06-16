cmake_minimum_required(VERSION 2.8)
project(open_karto)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    karto
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(karto SHARED src/Karto.cpp src/Mapper.cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS karto
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
