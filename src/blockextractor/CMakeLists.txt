
add_executable(blockextractor)
target_sources(blockextractor PRIVATE "blockextractor.cpp")
target_link_libraries(blockextractor vmcore)
#target_include_directories(block2raw "../../VMCore/include")
#vm_target_dependency(block2raw vmocore PRIVATE)
install(TARGETS blockextractor LIBRARY DESTINATION "lib" RUNTIME DESTINATION "bin" ARCHIVE DESTINATION "lib")
