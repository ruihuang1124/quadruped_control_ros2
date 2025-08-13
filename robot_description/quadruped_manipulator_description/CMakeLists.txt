cmake_minimum_required(VERSION 3.8)
project(quadruped_manipulator_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY xacro launch config urdf xml assets
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()