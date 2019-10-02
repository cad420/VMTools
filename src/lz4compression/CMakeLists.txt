


find_package(lz4 CONFIG REQUIRED)
if(lz4_FOUND)

add_executable(lvd2lz4encoder)
aux_source_directory(. SRC)
target_sources(lvd2lz4encoder PRIVATE ${SRC})
target_link_libraries(lvd2lz4encoder vmcore lz4::lz4)
target_include_directories(lvd2lz4encoder PRIVATE ${lz4_INCLUDE_DIRS})
install(TARGETS lvd2lz4encoder LIBRARY DESTINATION "lib" RUNTIME DESTINATION "bin" ARCHIVE DESTINATION "lib")

endif()
