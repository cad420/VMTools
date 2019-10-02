cmake_minimum_required(VERSION 3.12)
project(h264compression)

aux_source_directory(. SRC)
add_executable(h264compression)
target_sources(h264compression PRIVATE ${SRC})
