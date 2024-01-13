#!/usr/bin/env python

# Purpose to attempt to do autonomous navigation for robot by giving current and target position to move toward 

# Import the rospy library
import rospy

# Import libraries for Navigation and Path Planning - Message type for communicating location to navigate to 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Import libraries used for managing process for Navigation - Give Action, Monitor and Receive Feedback can be conducted through actionlib 
import actionlib

# Import libraries used for handing X Y Z coordinates of robot position  
from geometry_msgs.msg import Point

# Function to attempt to move robot towards goal 
def move_to_goal(current_x, current_y, goal_x, goal_y):
    
    # Starting node for conducting movement
    rospy.init_node('move_base_to_goal', anonymous=True)

    # Creating client that will communicate navigation goals using actionlib     
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for client to be properly ready to handle message - Waiting for move_base server
    client.wait_for_server()

    # Goal created with MoveBaseGoal message type to hold navigation goal
    goal = MoveBaseGoal()

    # Setting frame to Map created of assessment world
    goal.target_pose.header.frame_id = "map"

    # Allowing for X Y values to be collected while Z value stays 0 as robot does not need to upwards or downwards 
    goal.target_pose.pose.position = Point(goal_x, goal_y, 0.0)

    # Orientation not specified allowing any orientation to be used
    goal.target_pose.pose.orientation.w = 1.0

    # Sends Navigation Goal to client 
    client.send_goal(goal)

    # Waits for feedback of whether or not goal reached
    client.wait_for_result()

if __name__ == '__main__':
    try:
        # Collects current position and target position from user input
        current_x = float(input("Enter current x-coordinate: "))
        current_y = float(input("Enter current y-coordinate: "))
        goal_x = float(input("Enter goal x-coordinate: "))
        goal_y = float(input("Enter goal y-coordinate: "))

        # Ideally, the robot should be automatically be able to get its current position 
        # And target position should be already saved in robot as a Point on Map - Point can be created once the map is created, in which engineer would be place the robot in that position and save that point on the map  

        # Applies the function to the values collected 
        move_to_goal(current_x, current_y, goal_x, goal_y)
    except rospy.ROSInterruptException:
        pass

