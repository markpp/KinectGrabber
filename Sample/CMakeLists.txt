cmake_minimum_required( VERSION 2.8 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

project( sample )
add_executable( sample kinect2_grabber.h main.cpp )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( KinectSDK2 REQUIRED )
find_package( OpenCV 2.4 REQUIRED )

if( PCL_FOUND AND KinectSDK2_FOUND )
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )
  link_directories( ${OpenCV_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( sample ${PCL_LIBRARIES} )
  target_link_libraries( sample ${OpenCV_LIBRARIES} )
  target_link_libraries( sample ${KinectSDK2_LIBRARIES} )
endif()
