cmake_minimum_required(VERSION 3.5)
project(mts_department)

find_package(gazebo_ros REQUIRED)
find_package(ament_cmake_ros REQUIRED)

install(DIRECTORY models worlds maps
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(gazebo_ros)

ament_package()

