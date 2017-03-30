project(${XML2_LIBRARY} C)

set(XML2_MAJOR_VERSION 2 PARENT_SCOPE)
set(XML2_MINOR_VERSION 7 PARENT_SCOPE)
set(XML2_PATCH_VERSION 2 PARENT_SCOPE)

vp_include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB lib_srcs *.c)
file(GLOB lib_hdrs *.h include/*.h)

# does't work
#vp_list_filterout(lib_srcs test*.c)
#vp_list_filterout(lib_srcs run*.c)

add_definitions("-DHAVE_CONFIG_H")
add_library(${XML2_LIBRARY} STATIC ${lib_srcs} ${lib_hdrs})

if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(${XML2_LIBRARY} PROPERTIES FOLDER "3rdparty")
endif()

set_target_properties(${XML2_LIBRARY}
  PROPERTIES OUTPUT_NAME ${XML2_LIBRARY}
  DEBUG_POSTFIX "${VISP_DEBUG_POSTFIX}"
  COMPILE_PDB_NAME ${XML2_LIBRARY}
  COMPILE_PDB_NAME_DEBUG "${XML2_LIBRARY}${VISP_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${VISP_3P_LIBRARY_OUTPUT_PATH}
  )

if(NOT BUILD_SHARED_LIBS)
  vp_install_target(${XML2_LIBRARY} EXPORT VISPModules ARCHIVE DESTINATION ${VISP_3P_LIB_INSTALL_PATH} COMPONENT dev)
endif()
