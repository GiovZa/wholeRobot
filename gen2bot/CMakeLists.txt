cmake_minimum_required(VERSION 3.0.2)
project(gen2bot)

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
"${CMAKE_CURRENT_LIST_DIR}/include/gen2bot"
${catkin_INCLUDE_DIRS}
)

# Makes the python script executable with "rosrun motor_control_gen2bot motorTalker.py" command
 catkin_install_python(PROGRAMS
   scripts/motorTalker.py
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
add_executable(falcon_tests_final src/falcon_tests.cpp)
target_link_libraries(falcon_tests_final
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ script executable with "rosrun motor_control_gen2bot listenerMotor" command
add_executable(listenerMotorFinal src/listenerMotor.cpp)
target_link_libraries(listenerMotorFinal 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes the C++ script executable with "rosrun motor_control_gen2bot listenerMotorAuto" command
add_executable(listenerMotorAutoFinal src/listenerMotorAuto.cpp)
target_link_libraries(listenerMotorAutoFinal 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

# Makes C++ script that holds mining functions. minePractice.cpp is the main file called 
# upon, src/gen2botPractice.cpp holds class gen2bot functions,
# and gen2botPractice.h contains the class of gen2bot. When making an exectuable, you must list all
# files that the exectuable will rely on for code. You use an exectuable to compile code and turn
# a text file into usable c++ code. Because I need code from 3 files, I must list every file.
# The first argument is the name of what you put to call on a file to run ex.
# add_executable( <name of executable> $PATH_TO_FILE1/File1.cpp $PATH_TO_FILE2/File2.h etc...> )
# To run all 3 files in ros, do "rosrun motor_control_gen2bot minePractice"
add_executable(
minePracticeFinal
 
src/minePractice.cpp 
src/gen2botPractice.cpp 
include/gen2bot/gen2botPractice.h
)

# Now to link the DLLs with minePractice, we need CTRE DLL and default catkin DLLs:
target_link_libraries(minePracticeFinal 
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)

add_executable(
mineFinal 

src/mine.cpp 
src/gen2bot.cpp 
include/gen2bot/gen2bot.h
)

target_link_libraries(mineFinal
  CTRE_Phoenix
  ${catkin_LIBRARIES}
)
