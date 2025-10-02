#!/usr/bin/env python

# A basic MoveIt Python script for moving a robot arm to a hardcoded target pose.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        # Initialize moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        # Instantiate a RobotCommander object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object. This provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object. This object is an interface
        # to a planning group (a group of joints). In this project, the group is the set of joints in the
        # robot arm. This interface can be used to plan and execute motions:
        #
        # IMPORTANT: Change this name to match the planning group name for your robot.
        # You can find this in the MoveIt Setup Assistant or RViz. For UR robots, it's often "manipulator".
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a DisplayTrajectory ROS publisher which is used by RViz to visualize plans
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self, pose_goal):
        """
        Plans and moves the robot to a target pose.
        @param: pose_goal A geometry_msgs.msg.Pose
        """
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.set_pose_target(pose_goal)

        # Call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
        
    def go_to_joint_state(self, joint_goal):
        """
        Plans and moves the robot to a target joint state.
        @param: joint_goal A list of joint angles
        """
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


def main():
    try:
        print("----------------------------------------------------------")
        print("            MoveIt Pick and Place Python Demo             ")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        tutorial = MoveGroupPythonInterface()

        # --- Define Target Poses ---
        # A target pose is defined by a position (x, y, z) and an orientation (quaternion: x, y, z, w).
        
        # POSE 1: A pose near the center of the workspace
        pose_goal_1 = geometry_msgs.msg.Pose()
        pose_goal_1.orientation.w = 1.0
        pose_goal_1.position.x = 0.4
        pose_goal_1.position.y = 0.1
        pose_goal_1.position.z = 0.4

        # POSE 2: Another pose
        pose_goal_2 = geometry_msgs.msg.Pose()
        pose_goal_2.orientation.w = 1.0
        pose_goal_2.position.x = -0.4
        pose_goal_2.position.y = 0.2
        pose_goal_2.position.z = 0.5

        # --- Define Target Joint States ---
        # A joint state is a list of angles for each joint in the planning group.
        # The order of the joints is important.
        
        # Joint State 1: Upright position
        joint_goal_1 = tutorial.move_group.get_current_joint_values()
        joint_goal_1[0] = 0
        joint_goal_1[1] = -pi / 4
        joint_goal_1[2] = 0
        joint_goal_1[3] = -pi / 2
        joint_goal_1[4] = 0
        joint_goal_1[5] = pi / 3

        # --- Execute Movements ---
        
        print("============ Press `Enter` to move to the first JOINT state...")
        raw_input()
        tutorial.go_to_joint_state(joint_goal_1)
        
        print("============ Press `Enter` to move to the first POSE goal...")
        raw_input()
        tutorial.go_to_pose_goal(pose_goal_1)

        print("============ Press `Enter` to move to the second POSE goal...")
        raw_input()
        tutorial.go_to_pose_goal(pose_goal_2)

        print("============ Demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
