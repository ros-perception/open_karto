cmake_minimum_required(VERSION 2.8)

set(KARTO_LIB_TYPE SHARED)
add_subdirectory(source)

install(FILES package.xml DESTINATION share/open_karto)
