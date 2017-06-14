cmake_minimum_required(VERSION 2.8.3)
project(open_karto)

set(CMAKE_CXX_FLAGS "-ftemplate-backtrace-limit=0")
find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread serialization)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS
    include
  LIBRARIES
    karto
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(karto SHARED src/Karto.cpp src/Mapper.cpp)
target_link_libraries(karto ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS karto
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
