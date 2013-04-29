cmake_minimum_required(VERSION 2.8)

PROJECT(open_karto)

# Export flags for dependent catkin packages
option(USE_CATKIN "Use catkin if found to build the package" ON)
if (USE_CATKIN)
  find_package(catkin)
  if(catkin_FOUND)
    message(STATUS "Using catkin.")
    catkin_package(INCLUDE_DIRS source/OpenKarto/
                  LIBRARIES OpenKarto)
  endif(catkin_FOUND)
endif(USE_CATKIN)

set(KARTO_LIB_TYPE SHARED)
add_subdirectory(source)

