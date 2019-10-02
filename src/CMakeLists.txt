

option(VM_BUILD_ABCFLOWGEN "Set ON to build the abcflow data generator" ON)
if(VM_BUILD_ABCFLOWGEN)
add_subdirectory("abcflowgen")
endif()

option(VM_BUILD_DOWNSAMPLE "Set ON to build the volume data sampler" ON)
if(VM_BUILD_DOWNSAMPLE)
add_subdirectory("downsample")
endif()

option(VM_BUILD_LVDCONVERTER "Set ON to build lvd converter" ON)
if(VM_BUILD_LVDCONVERTER)
add_subdirectory("lvdconverter")
endif()

option(VM_BUILD_LZ4COMPRESSION "Set ON to build lz4 compressor (lz4 library is required)" OFF)
if(VM_BUILD_LZ4COMPRESSION)
add_subdirectory("lz4compression")
endif()


option(VM_BUILD_H264COMPRESSION "Set ON to build h264 compressor" OFF)
if(VM_BUILD_H264COMPRESSION)
add_subdirectory("h264compression")
endif()
