cmake_minimum_required(VERSION 3.8)
project(ttbot_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- QT SETUP ---
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON) # Quan trọng: Để xử lý file .ui
set(CMAKE_AUTORCC ON)

include_directories(include)

# --- EXECUTABLE ---
add_executable(gui_node
  src/main.cpp
  src/main_window.cpp
  src/ros_worker.cpp
  src/mainwindow.ui            # <--- File ui
  include/ttbot_gui/main_window.hpp
  include/ttbot_gui/ros_worker.hpp
)

# --- LINK THƯ VIỆN ---
target_link_libraries(gui_node
  Qt5::Widgets Qt5::Core Qt5::Gui
)

ament_target_dependencies(gui_node
  rclcpp nav_msgs geometry_msgs
)

# --- INSTALL ---
install(TARGETS gui_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

ament_package()