
cmake_minimum_required(VERSION 3.12)
project(VMTools)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX)		# options for gnu compliler
  add_compile_options(-fpermissive)
  add_compile_options(-std=c++17)
endif()
include(scripts/external.cmake)
vm_external_module(
GIT_REPOSITORY https://github.com/cad420/VMCore.git
GIT_TAG master
)

add_subdirectory(src)
