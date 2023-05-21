cmake_minimum_required(VERSION 3.0.2)
# name of folder
project(gen2bot)

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  std_msgs
  std_srvs
  actionlib_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  cv_bridge
  move_base_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

catkin_package(
	INCLUDE_DIRS include # Set include path
	CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs actionlib_msgs message_generation sensor_msgs cv_bridge actionlib move_base_msgs # Message types list
	LIBRARIES lib # Set library path
)

# Set path of where to find header files
include_directories(
include
"${CMAKE_CURRENT_LIST_DIR}/include/gen2bot"
${catkin_INCLUDE_DIRS}
)

# Makes the python script executable with "rosrun gen2bot motorTester.py" command or whichever python file it is
 catkin_install_python(PROGRAMS
  scripts/move_base_client.py
  scripts/robot_mux_control.py
  scripts/permanentGoal.py
  scripts/robot_mux_control_wired.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Link the CTRE DLL found in 'lib' folder
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/jetsontx)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/x86-64)
endif()

add_library(CTRE_Phoenix SHARED IMPORTED)
set_target_properties(CTRE_Phoenix PROPERTIES IMPORTED_LOCATION ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)

add_executable(
motorTestTrencher

src/motorTest.cpp 
)
target_link_libraries(motorTestTrencher
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Final executable to run everything at once
add_executable(
flexRobot

src/robot_mux.cpp 
src/processManagerClass.cpp 
include/gen2bot/processManagerClass.h
src/semi_auto_trencher_class.cpp 
include/gen2bot/semi_auto_trencher_class.h
src/manual_trencher_class.cpp 
include/gen2bot/manual_trencher_class.h
src/base_trencher_class.cpp 
include/gen2bot/base_trencher_class.h
include/gen2bot/wheel_trencher_class.h

)
target_link_libraries(flexRobot
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)
