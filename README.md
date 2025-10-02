# MoveIt Python Script for Robot Arm Control
This guide explains how to use the go_to_pose.py script to control a robot arm in a ROS/MoveIt environment, specifically for your project involving the Unity simulation.

# 1.Overview
The go_to_pose.py script is your first step as the MoveIt & Robot Control Engineer. It provides the basic building blocks for commanding the robot arm to move to specific, pre-defined (hardcoded) locations and orientations in its workspace.

This script uses the moveit_commander Python library, which is the standard tool for this task.

# 2.Prerequisites
Before you can run this script, you must have the following components running and correctly configured:

1. ROS Core: A roscore must be running.

2. Unity Simulation: The Unity project with the robot arm must be running.

3. ROS-Unity Connection: The ROS TCP Connector in Unity must be successfully connected to your ROS network.

4. MoveIt move_group: You need to launch the MoveIt configuration for your robot. This is typically done with a command like:

       roslaunch <your_moveit_config_package> demo.launch
       
(Replace <your_moveit_config_package> with the actual name of your MoveIt config package, which might be inside the catkin_workspace from the provided links).

This demo.launch file starts the move_group node, which does all the hard work of motion planning, and also starts RViz for visualization.

# 3.How to run the Script
1. Place the Script: Copy the go_to_pose.py script into the scripts folder of a ROS package in your catkin workspace (e.g., a package named robot_control).

2. Make it Executable: Open a terminal, navigate to the scripts folder, and run:
   
        chmod +x go_to_pose.py

3. Run the Script: With all the prerequisites from step 2 running, open a new terminal and execute the script using rosrun:

        rosrun <your_package_name> go_to_pose.py

(Replace <your_package_name> with the name of the ROS package where you saved the script).
  
The robot arm in both Unity and RViz should now move to the target poses defined in the script each time you press Enter in the terminal.

# 4.How to Customize and Find New Poses
This script is just a starting point. You will need to modify it with your own target poses.

# A. Verify Your Planning Group Name
The script assumes the arm's planning group is named "manipulator". This is common for UR robots, but you should verify it.
  . In the running RViz window (from demo.launch), look at the "MotionPlanning" display panel. The "Planning Group" dropdown will list all available groups. Make sure you are using the correct one for the arm.
  . If it's different (e.g., "ur3_arm"), change this line in the script:
        group_name = "manipulator" # CHANGE THIS

# B. Finding New Target Poses
The easiest way to find the coordinates for a new target pose is to use RViz.
1. Move the Robot Interactively: In the RViz "MotionPlanning" panel, under the "Planning" tab, you'll see an interactive marker for the robot's end-effector. Drag this marker to the exact position and orientation you want.

2. Get the Pose Information: Open a new terminal and use rostopic to see the current pose of the end-effector link. First, find the name of your end-effector link from the script's terminal output. Let's assume it's tool0. Then run:
        rostopic echo -n 1 /tf
Look for a transform from your base_link (or similar) to your tool0 (or end-effector link). This will give you the translation (position: x, y, z) and rotation (orientation: x, y, z, w) that you need.

3. Update the Script: Copy these values into a new geometry_msgs.msg.Pose() object in the main() function of the script.

        # A new custom pose
        pose_goal_custom = geometry_msgs.msg.Pose()
        # Position from rostopic echo
        pose_goal_custom.position.x = ...
        pose_goal_custom.position.y = ...
        pose_goal_custom.position.z = ...
        # Orientation from rostopic echo
        pose_goal_custom.orientation.x = ...
        pose_goal_custom.orientation.y = ...
        pose_goal_custom.orientation.z = ...
        pose_goal_custom.orientation.w = ...

        # Then, command the robot to move there
        tutorial.go_to_pose_goal(pose_goal_custom)

This workflow is fundamental for your next tasks, which involve picking and placing objects. You will use this method to define poses for approaching, grasping, and releasing objects.
