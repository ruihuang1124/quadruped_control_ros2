cmake_minimum_required(VERSION 3.8)
project(arcdog_adjustable_leg_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes xacro launch config urdf xml assets
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()