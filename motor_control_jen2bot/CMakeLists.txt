cmake_minimum_required(VERSION 3.0.2)
project(motor_control_jen2bot)

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
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs
	LIBRARIES lib
)
 
 include_directories(
include
"${CMAKE_CURRENT_LIST_DIR}/include/motor_control_jen2bot"
${catkin_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   scripts/motorTalker.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/jetsontx)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/x86-64)
endif()

add_library(CTRE_Phoenix SHARED IMPORTED)
set_target_properties(CTRE_Phoenix PROPERTIES IMPORTED_LOCATION ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)

add_executable(falcon_tests_old src/falcon_tests.cpp)
target_link_libraries(falcon_tests_old
  CTRE_Phoenix
  ${catkin_LIBRARIES}
  Threads::Threads
)

add_executable(listenerMotor_old src/listenerMotor.cpp)
target_link_libraries(listenerMotor_old 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
  Threads::Threads
)

add_dependencies(listenerMotor beginner_tutorials_generate_messages_cpp)

 set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -std=c++17 -lstdc++fs")

#add_executable(${motorControlJen2Bot}_node src/falcon_tests.cpp)

