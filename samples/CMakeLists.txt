#
# Copyright 2010 SRI International
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

ADD_DEFINITIONS(-DUSE_SPA)

if(WIN32)
  ADD_DEFINITIONS(-DEIGEN_DONT_ALIGN)
endif()

if(WIN32)
  # turn off various warnings
  foreach(warning 4251)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${warning}")
  endforeach(warning)
endif()

############################################################################
# tutorial 1
add_executable(
  Karto_Tutorial1
  samples/tutorial1.cpp
)

target_link_libraries(Karto_Tutorial1 karto)

if (UNIX)
  target_link_libraries(Karto_Tutorial1 "pthread")
endif()

############################################################################
# tutorial 2

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(
  Karto_Tutorial2
  samples/tutorial2.cpp
  samples/SpaSolver.h
  samples/SpaSolver.cpp
)

target_link_libraries(Karto_Tutorial2 karto ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

if(PNG_FOUND)
  target_link_libraries(Karto_Tutorial2 ${ZLIB_LIBRARIES})
  target_link_libraries(Karto_Tutorial2 ${PNG_LIBRARIES})
endif()

if (UNIX)
  target_link_libraries(Karto_Tutorial2 "pthread")
endif()
