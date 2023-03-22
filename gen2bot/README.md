Structure of Gen2bot Package:



    config: Holds parameters (basically variables in the ROS server) for launch files

    include: Holds header files for C++

    launch: Where all nodes from all packages are launched simultaneously

    lib: Where shared objects are stored, in this case it is our Phoenix Motors Library

    scripts: Where python files are

    sessions: Where our image of the qr code is stored for object recognition node

    src: Where source C++ files are 

    CMakeLists.txt: Where our files get built and compiled

    package.xml: Where our package (basically whole directory) gets manifested



Third Party ROS Packages used:



    Move_base: Our path planner for navigating to desired positions

    Robot_Localization: Sensor fuses all sensor topics so robot can have more accurate pose estimation

    Octomap_Server: Updates obstacles to a fixed frame 

    Joy: Converts linux terminal inputs from a joystick controller into ROS

    Find_Object_2d: Detects objects and gets estimated position and orientation from desired frame

    ROS_Realsense: ROS package for Intel's D455 depth and T265 tracking cameras



Motor Library Used:



    Phoenix