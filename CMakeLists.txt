__cc_find(thumbnail)
__cc_find(stringutil)

__assert_target(thumbnail)
__assert_target(stringutil)

set(SRCS  gcode/define.h 
		 gcode/interface.h
		 gcode/sliceline.h
		 gcode/gcodedata.h
		 gcode/gcodedata.cpp
		 gcode/slicemodelbuilder.cpp
		 gcode/slicemodelbuilder.h
		 gcode/sliceresult.cpp
		 gcode/sliceresult.h
		 )

set(LIBS thumbnail trimesh2 mmesh stringutil)
		
if(CC_BC_LINUX)
	list(APPEND LIBS pthread)
endif()				
if( WIN32 AND NOT CYGWIN )
	set(LIBS ${LIBS} crypt32 ws2_32)
endif()

__add_common_library(gcode)

if( WIN32 AND NOT CYGWIN )
	target_link_libraries(gcode PRIVATE crypt32 ws2_32)
endif()