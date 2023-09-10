__cc_find(stringutil)

__assert_target(stringutil)

__files_group(${CMAKE_CURRENT_SOURCE_DIR}/gcode/ SRCS)

set(LIBS trimesh2 mmesh stringutil)
		
if(CC_BC_LINUX)
	list(APPEND LIBS pthread)
endif()				
if( WIN32 AND NOT CYGWIN )
	list(APPEND LIBS crypt32 ws2_32)
endif()

__add_common_library(gcode)