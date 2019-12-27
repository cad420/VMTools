
project(downsample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(downsample)
target_sources(downsample PRIVATE "downsample.cpp")
target_link_libraries(downsample vmcore)
install(TARGETS downsample LIBRARY DESTINATION "lib" RUNTIME DESTINATION "bin" ARCHIVE DESTINATION "lib")
