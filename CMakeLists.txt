cmake_minimum_required(VERSION 3.5)
project(msp_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-address-of-packed-member )
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(msp_msgs REQUIRED)
find_package(msp_controller REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

pkg_check_modules(GST REQUIRED 
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-rtsp-server-1.0
)



add_executable(msp_vision src/msp_vision.cpp 
                          src/msp_vision_overlay.cpp
                          lib/rtsp/video.cpp
                              )
                                          
ament_target_dependencies(msp_vision msp_controller rclcpp px4_msgs msp_msgs gz-transport13 cv_bridge OpenCV )
ament_export_include_directories(include)

#target_link_libraries(msp_vision )

include_directories(
  ${GST_INCLUDE_DIRS}
  )


target_link_libraries(${PROJECT_NAME}
    ${GST_LIBRARIES}
)



ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs )



install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
  msp_vision
  DESTINATION lib/${PROJECT_NAME})

ament_package()
