cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-mb)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-tracker.cpp
  tutorial-mb-tracker-full.cpp
)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.mp4" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.cao" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.init" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.png" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.wrl" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.xml" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    visp_set_source_file_compile_flag(${cpp} -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-float-equal -Wno-deprecated-copy -Wno-register)
  endif()

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-mb-tracker.cpp ${data})
endforeach()
