cmake_minimum_required(VERSION 3.0.2)
# name of folder
project(auger)

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
  scripts/robot_mux_control_auger.py
  scripts/autoManager.py
  scripts/robot_mux_auto.py
  scripts/permanentGoalAug.py

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

# Makes the C++ script executable with "rosrun gen2bot falcon_tests" command
add_executable(falcon_tests src/falcon_tests.cpp)
target_link_libraries(falcon_tests
  CTRE_Phoenix # Motor library
  ${catkin_LIBRARIES} 
)

add_executable(manualDrive src/manualDrive.cpp)
target_link_libraries(manualDrive 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

add_executable(autoDrive src/autoDrive.cpp)
target_link_libraries(autoDrive
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ scripts executable with "rosrun gen2bot miningOperationsAuger" command
add_executable(
miningOperationsAuger

src/robot_mux.cpp 
src/processManagerClass.cpp 
include/gen2bot/processManagerClass.h
src/augerOperationsClass.cpp 
include/gen2bot/augerOperationsClass.h
)
target_link_libraries(miningOperationsAuger
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

add_executable(
motorTestAuger

src/motorTest.cpp 
)
target_link_libraries(motorTestAuger
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)
