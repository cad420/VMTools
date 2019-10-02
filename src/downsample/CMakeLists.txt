
project(downsample)

add_executable(downsample)
target_sources(downsample PRIVATE "downsample.cpp")
target_link_libraries(downsample vmcore)
install(TARGETS downsample LIBRARY DESTINATION "lib" RUNTIME DESTINATION "bin" ARCHIVE DESTINATION "lib")
