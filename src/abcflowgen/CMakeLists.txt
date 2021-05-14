

add_executable(abcflowgen)
target_sources(abcflowgen PRIVATE "abcflowgen.cpp" "gen.cpp")
target_link_libraries(abcflowgen vmcore)
install(TARGETS abcflowgen LIBRARY DESTINATION "lib" RUNTIME DESTINATION "bin" ARCHIVE DESTINATION "lib")