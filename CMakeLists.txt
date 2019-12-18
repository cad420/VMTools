
cmake_minimum_required(VERSION 3.12)
project(VMTools)


include(scripts/external.cmake)

vm_external_module(
GIT_REPOSITORY https://github.com/cad420/VMCore.git
GIT_TAG master
)

add_subdirectory(src)