cmake_minimum_required(VERSION 3.5)
project(astra_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  add_compile_options(-Wno-deprecated)
endif()

# find packages
# find_package(catkin REQUIRED camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs roscpp message_generation)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rosidl_default_generators REQUIRED)  # use to generate custom srv and msg
find_package(rosidl_cmake REQUIRED)
find_package(libuvc REQUIRED)
set(ros2_dependencies "rclcpp" "sensor_msgs" "builtin_interfaces") # store a var to tell ament

# machine config
#MACHINE = $(shell uname -m)
execute_process(COMMAND uname -m  OUTPUT_VARIABLE  MACHINES )
execute_process(COMMAND getconf LONG_BIT  OUTPUT_VARIABLE  MACHINES_BIT )
MESSAGE(STATUS "ORRBEC Machine : ${MACHINES}")
MESSAGE(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")
IF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "x64")
# for schroot enviroment
ELSEIF ( (${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "x86" )
ELSEIF ( ${MACHINES} MATCHES "x86" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i686" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "i386" )
      set(HOST_PLATFORM  "x86")
ELSEIF ( ${MACHINES} MATCHES "arm" )
      set(HOST_PLATFORM  "arm")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64")  )
      set(HOST_PLATFORM  "arm64")
ELSEIF ( (${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "32")  )
      set(HOST_PLATFORM  "arm")
ENDIF ()

message(STATUS "ORRBEC : ${HOST_PLATFORM}")
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

# generate_dynamic_reconfigure_options(cfg/Astra.cfg cfg/UVCCamera.cfg)

# custom services
set(ros_astra_SRV
  srv/GetSerial.srv
  srv/GetDeviceType.srv
  srv/GetIRGain.srv
  srv/GetCameraInfo.srv
  srv/GetUVCExposure.srv
  srv/GetIRExposure.srv
  srv/GetUVCGain.srv
  srv/GetUVCWhiteBalance.srv
  srv/SetUVCWhiteBalance.srv
  srv/SetUVCGain.srv
  srv/SetIRExposure.srv
  srv/SetIRGain.srv
  srv/SetIRFlood.srv
  srv/SetLaser.srv
  srv/SetLDP.srv
  srv/SetUVCExposure.srv
  srv/ResetIRGain.srv
  srv/ResetIRExposure.srv
  srv/SwitchIRCamera.srv
)

# ros2 custom service & messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${ros_astra_SRV}
)

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})

set(ORBBEC_OPENNI2_REDIST  "${CMAKE_CURRENT_SOURCE_DIR}/include/openni2_redist/${HOST_PLATFORM}")
# link_directories(${ORBBEC_OPENNI2_REDIST}
#                 #  ${catkin_LINK_DIRS}
#                  )

#MESSAGE(STATUS "ORRBEC : ${ORBBEC_OPENNI2_REDIST}")

include_directories(include
                    # ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/openni2
                    ${libuvc_INCLUDE_DIRS}
                    )

                    
# target astra_wapper
add_library(astra_wrapper
   src/astra_convert.cpp
   src/astra_device.cpp
   src/astra_device_info.cpp
   src/astra_timer_filter.cpp
   src/astra_frame_listener.cpp
   src/astra_device_manager.cpp
   src/astra_exception.cpp
   src/astra_video_mode.cpp
   src/astra_device_type.cpp
)

ament_target_dependencies(astra_wrapper  ${ros2_dependencies})
target_link_libraries(astra_wrapper -lOpenNI2  -L${ORBBEC_OPENNI2_REDIST}
  ${Boost_LIBRARIES} )
rosidl_target_interfaces(astra_wrapper
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target astra_driver_lib
add_library(astra_driver_lib
                src/astra_driver.cpp
                src/astra_device_type.cpp
)
ament_target_dependencies(astra_driver_lib  ${ros2_dependencies})
target_link_libraries(astra_driver_lib astra_wrapper ${Boost_LIBRARIES})
rosidl_target_interfaces(astra_driver_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target nodelet?
# add_library(astra_camera_nodelet
#    ros/astra_camera_nodelet.cpp
# )
# target_link_libraries(astra_camera_nodelet astra_driver_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
# add_dependencies(astra_camera_nodelet ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

# target astra camera node
add_executable(astra_camera_node
   ros/astra_camera_node.cpp
)
ament_target_dependencies(astra_camera_node ${ros2_dependencies})
target_link_libraries(astra_camera_node astra_driver_lib ${Boost_LIBRARIES})
rosidl_target_interfaces(astra_camera_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target list devices
add_executable(astra_list_devices
   src/list_devices.cpp
)
target_link_libraries(astra_list_devices astra_wrapper)

add_executable(astra_test_wrapper test/test_wrapper.cpp )
ament_target_dependencies(astra_test_wrapper ${ros2_dependencies})
target_link_libraries(astra_test_wrapper astra_wrapper ${Boost_LIBRARIES})
if (UNIX AND NOT APPLE)
  add_executable(astra_usb_reset src/usb_reset.c)
  set(ADDITIONAL_EXECUTABLES "astra_usb_reset")
endif()

# target uvc camera node 
add_executable(camera_node 
  src/libuvc_camera/main.cpp
  src/libuvc_camera/camera_driver.cpp
  src/astra_device_type.cpp)

ament_target_dependencies(camera_node ${ros2_dependencies})
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES})
rosidl_target_interfaces(camera_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# target uvc camera nodelet
# add_library(libuvc_camera_nodelet src/libuvc_camera/nodelet.cpp
#                                   src/libuvc_camera/camera_driver.cpp
#                                   src/astra_device_type.cpp)
# target_link_libraries(libuvc_camera_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
# add_dependencies(libuvc_camera_nodelet ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

# installation
install(TARGETS 
  astra_wrapper 
  # astra_camera_nodelet 
  astra_camera_node 
  astra_list_devices 
  astra_driver_lib 
  ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(FILES include/openni2_redist/${HOST_PLATFORM}/libOpenNI2.so
  DESTINATION lib/
)

install(DIRECTORY include/openni2_redist/${HOST_PLATFORM}/OpenNI2
  DESTINATION lib/
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION include/${PROJECT_NAME}
)

# # add xml file
# install(FILES astra_nodelets.xml
  # DESTINATION share/${PROJECT_NAME}
# )

install(FILES 56-orbbec-usb.rules
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES 56-orbbec-usb.rules
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS camera_node
#  libuvc_camera_nodelet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
