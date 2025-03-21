cmake_minimum_required(VERSION 3.10)

project(tutorial-tracking-mb-aprilTag-init)

find_package(VISP REQUIRED visp_core visp_sensor visp_gui visp_vision visp_mbt visp_detection)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-apriltag-webcam.cpp
  tutorial-mb-generic-tracker-apriltag-rs2.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    # Add specific build flag to turn off warnings coming from Ogre3D 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-float-equal")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-register")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-but-set-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
  endif()
  if(VISP_HAVE_FLYCAPTURE)
    # Add specific build flag to turn off warnings coming from FlyCapture 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  endif()
  if(VISP_HAVE_REALSENSE2)
    # Add specific build flag to turn off warnings coming from librealsense 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")
  endif()

  if(CXX_FLAGS_MUTE_WARNINGS)
    # Add specific build flag to turn off warnings
    visp_set_source_file_compile_flag(${cpp} ${CXX_FLAGS_MUTE_WARNINGS})
  endif()

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
