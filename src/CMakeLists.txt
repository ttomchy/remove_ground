add_subdirectory(communication)
add_subdirectory(ground_removal)
add_subdirectory(image_labelers)
add_subdirectory(visualization)
add_subdirectory(projections)
add_subdirectory(utils)

if(ROS_FOUND)
  add_subdirectory(ros_bridge)
endif()
