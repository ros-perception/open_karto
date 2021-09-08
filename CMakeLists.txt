cmake_minimum_required(VERSION 3.0.2)
project(open_karto)

find_package(catkin REQUIRED
  COMPONENTS
    sparse_bundle_adjustment
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS
    sparse_bundle_adjustment
  DEPENDS Boost
  INCLUDE_DIRS
    include
  LIBRARIES
    karto
)

if(BUILD_SHARED_LIBS)
  add_definitions(-DKARTO_DYNAMIC)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(karto SHARED src/Karto.cpp src/Mapper.cpp)
target_link_libraries(karto ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS karto
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

include(samples/CMakeLists.txt)
