Launch scripts are what opens many ros nodes and files in one terminal command

autoTests.launch is a file used to test different components of autonomy, this file 
keeps changing depending on what we need

rgbd.launch is a launch file someone made to make jetsons work with realsense depth cams

wheels.launch is a luanch file to configure wheel PID and convert wheel encoders to 
odometry and twist, currently being reworked to configure PID for all motors 

brokenFullAuto.launch will eventually be the code that runs full autonomy, cycling between 
the different phases of the Alabama competition

semiAuto.launch is made for running partial autonomy in case full auto can't be achieved

proofOfLife.launch is made for proof of life video at Iowa on March 24 & 25