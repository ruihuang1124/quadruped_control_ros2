cmake_minimum_required(VERSION 3.8)
project(sirius_wheel_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes simplified_meshes xacro launch config urdf xml assets new_assets
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()