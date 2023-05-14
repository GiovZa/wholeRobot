# Before even going through the configurations, I suggest looking up:
# https://www.cloudbees.com/blog/yaml-tutorial-everything-you-need-get-started
# To understand markup languages and the syntax for .yaml files

Costmap files contain the parameters of the maps used to detect obstacles
common_costmap has general configurations for both global and local costmaps
Global costmap is the static frame where robot moves around (basically arena)
Local costmap is the rolling window that follows the robot to get a more detailed map 

Global costmap uses the octomap server to plan it's path
Local costmap uses the obstacle layer from common_costmap.yaml to update it's path planner

PIDTrencher.yaml has the motor configurations

robot_localization.yaml has parameters for the robot_localization ekf node which
updates the position of the robot between the local planner and robot

states.yaml holds the states of the robot so that ROS knows what state the robot is
currently in ex. digMode state is true, meaning move_base should not be sending nav_goals
