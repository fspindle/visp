cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-blob)

find_package(VISP REQUIRED visp_core visp_blob visp_io visp_gui visp_sensor)

# set the list of source files
set(tutorial_cpp
  tutorial-blob-auto-tracker.cpp
  tutorial-blob-tracker-live.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/target.pgm" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-blob-auto-tracker.cpp ${data})
endforeach()
