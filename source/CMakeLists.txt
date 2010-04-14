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

CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6)
 
# set library name
SET(LIB_NAME Karto)

PROJECT(${LIB_NAME})

if(WIN32)
  # turn off various warnings
  foreach(warning 4251)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${warning}")
  endforeach(warning)
endif(WIN32)

# define public header files
SET(HEADER_FILES
  kt_Types.h
  kt_Mapper.h
)

# define source files
SET(SOURCE_FILES
  kt_Types.cpp
  kt_Mapper.cpp
)

# define 'Header Files' source group - used in Visual Studio
SOURCE_GROUP("Header Files" FILES 
  ${HEADER_FILES}
)

# define 'Source Files' source group - used in Visual Studio
SOURCE_GROUP("Source Files" FILES 
  ${SOURCE_FILES}
)

# Add platform specific definitions
IF (WIN32)
  ADD_DEFINITIONS(-D_UNICODE)
ENDIF (WIN32)

if (KARTO_STATIC)
  ADD_DEFINITIONS(-DKARTO_STATIC)
else (KARTO_STATIC)
  ADD_DEFINITIONS(-DKARTO_DYNAMIC)
endif (KARTO_STATIC)

# setup kt target
ADD_LIBRARY(${LIB_NAME}
  ${KARTO_LIB_TYPE}
  ${HEADER_FILES}
  ${SOURCE_FILES}
)
# Also build a shared version of the library
ADD_LIBRARY(${LIB_NAME}-shared SHARED
  ${KARTO_LIB_TYPE}
  ${HEADER_FILES}
  ${SOURCE_FILES}
)
# Prevent deletion of existing lib of same name
SET_TARGET_PROPERTIES(${LIB_NAME}-shared PROPERTIES CLEAN_DIRECT_OUTPUT 1)
# Set output name to be the same as shared lib (may not work on
# Windows)
SET_TARGET_PROPERTIES(${LIB_NAME}-shared PROPERTIES OUTPUT_NAME ${LIB_NAME})

# Install libraries and header(s)
INSTALL(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION "lib")
INSTALL(TARGETS ${LIB_NAME}-shared
        LIBRARY DESTINATION "lib")
INSTALL(FILES kt_Mapper.h kt_Types.h
        DESTINATION "include/karto")
