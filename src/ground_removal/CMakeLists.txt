add_library(ground_remove SHARED depth_ground_remover.cpp) # 动态链接，在运行时被加载
target_link_libraries(ground_remove
                      identifiable
                      velodyne_utils
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})

#add_executable(learn_thread thread.cpp)
#target_link_libraries(ground_remove
#      identifiable
#     velodyne_utils
#      difference
#      ${PCL_LIBRARIES}
#     ${OpenCV_LIBS})