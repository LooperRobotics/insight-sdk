add_executable(imu_pose imu_pose.cpp)
target_link_libraries(imu_pose PRIVATE LooperSDK::looper)
