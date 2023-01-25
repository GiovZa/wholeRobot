cmake_minimum_required(VERSION 3.0.2)
project(motor_control_gen2bot)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  std_msgs
  std_srvs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

catkin_package(
	INCLUDE_DIRS include # Set include path
	CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs # Message types list
	LIBRARIES lib # Set library path
)
 
# Set path of where to find header files
include_directories(
include
"${CMAKE_CURRENT_LIST_DIR}/include/motor_control_gen2bot"
${catkin_INCLUDE_DIRS}
)

# Makes the python script executable with "rosrun motor_control_gen2bot motorTalker.py" command
 catkin_install_python(PROGRAMS
   scripts/motorTalker.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Link the DLL
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
  Threads::Threads
)

# Makes the C++ script executable with "rosrun motor_control_gen2bot listenerMotor" command
add_executable(listenerMotor src/listenerMotor.cpp)
target_link_libraries(listenerMotor 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
  Threads::Threads
)

# Makes the C++ script executable with "rosrun motor_control_gen2bot listenerMotorAuto" command
add_executable(listenerMotorAuto src/listenerMotorAuto.cpp)
target_link_libraries(listenerMotorAuto 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
  Threads::Threads
)

 set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -lstdc++fs")

