cmake_minimum_required(VERSION 3.5)
project(qt_gui_ros2)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(nav_msgs REQUIRED)



set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  nav_msgs
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(headers_to_moc
  include/qt_gui_ros2/mainWindow.hpp
)

foreach(header "${headers_to_moc}")
  qt5_wrap_cpp(moc_files "${header}")
endforeach()

set(SRC_FILES
  src/mainWindow.cpp
  src/main.cpp
  ${moc_files}
)

add_executable(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

## Install
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
