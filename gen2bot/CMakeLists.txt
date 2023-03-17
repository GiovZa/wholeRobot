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

# Makes the python script executable with "rosrun motor_control_gen2bot motorTalker.py" command or whichever python file it is
 catkin_install_python(PROGRAMS
   scripts/notDTTalker.py
   scripts/move_base_client.py
   scripts/wheelTester.py
   scripts/twistConvertor.py
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

# Makes the C++ script executable with "rosrun motor_control_gen2bot falcon_tets" command
add_executable(falcon_tests src/falcon_tests.cpp)
target_link_libraries(falcon_tests
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ script executable with "rosrun motor_control_gen2bot listenerMotor" command
add_executable(listenerMotor src/listenerMotor.cpp)
target_link_libraries(listenerMotor 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ script executable with "rosrun motor_control_gen2bot listenerMotorAuto" command
add_executable(listenerMotorAuto src/listenerMotorAuto.cpp)
target_link_libraries(listenerMotorAuto
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ scripts executable with "rosrun motor_control_gen2bot notDT" command
add_executable(
notDT

src/notDT.cpp 
src/ProcessManager.cpp 
include/gen2bot/ProcessManager.h
src/NotDTClass.cpp 
include/gen2bot/NotDTClass.h
)
target_link_libraries(notDT
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ scripts executable with "rosrun motor_control_gen2bot notDT" command
add_executable(
notDTAuto

src/notDTAuto.cpp 
src/ProcessManager.cpp 
include/gen2bot/ProcessManager.h
src/NotDTClass.cpp 
include/gen2bot/NotDTClass.h
)
target_link_libraries(notDTAuto
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# node for testing wheels
add_executable(
wheelTest

src/wheelTest.cpp 
)
target_link_libraries(wheelTest
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)
