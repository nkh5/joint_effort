cmake_minimum_required(VERSION 3.16)
project(amw_moveit)
find_package(ament_cmake REQUIRED)
find_package(moveit_servo REQUIRED)

# MoveIt Servo node with a clean shutdown (see README.md).
add_executable(servo_lifetime src/servo_lifetime.cpp)
target_compile_features(servo_lifetime PRIVATE cxx_std_17)
target_link_libraries(servo_lifetime moveit_servo::moveit_servo_lib_ros)
install(TARGETS servo_lifetime DESTINATION lib/${PROJECT_NAME})
ament_package()
