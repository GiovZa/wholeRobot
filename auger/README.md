Structure of Auger Package:

    config: Holds parameters (basically global variables in the ROS server) for launch files

    include: Holds header files for C++

    launch: Where all nodes from all packages are launched simultaneously

    lib: Where shared objects are stored, in this case it is our Phoenix Motors Library

    scripts: Where python files are found

    sessions: Where our image of the qr code is stored for object recognition node

    src: Where source C++ files are found

    CMakeLists.txt: Where our files get built and compiled

    package.xml: Where our package (basically whole directory) gets manifested



Motor Library Used:

    Phoenix