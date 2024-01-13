#!/usr/bin/env python

# Purpose is to conduct Localization by extracting current position of robot enabling us to calculate the distance between current and target position

# Import the rospy library
import rospy

# Import the ModelStates message type - Required as that is message type in /gazebo/model_states which holds position values
from gazebo_msgs.msg import ModelStates

# Import the Pose message type - Used for knowing robot position 
from geometry_msgs.msg import Pose

# Function for reading ModelStates messages
def model_states_callback(data):
    try:
        # Takes the position data from and stores it in robot_index
        # Robot name entered same as the URDF.Xacro File 
        robot_index = data.name.index('robot')

    # Prints error message if the robot name above is not found 
    except ValueError:
        rospy.logwarn("Robot name not found in ModelStates")
        return

    # Takes the position values from robot state data 
    position = data.pose[robot_index].position

    # Prints the extracted position data 
    rospy.loginfo("Current Position (x, y, z): ({}, {}, {})".format(
        position.x, position.y, position.z))

    # Forces node to stop as position data required is only one 
    rospy.signal_shutdown("")

# Function for setting up node and subscribing to robot position to collect current location 
def model_states_listener():
    # Starts with creating required node to collect data 
    rospy.init_node('model_states_listener', anonymous=True)

     # Subscribed /gazebo/model_states topic that stores info about robot position 
     # callback function is used to break down data and extract it 
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

    # Keeps running loop
    rospy.spin()

if __name__ == '__main__':
    try:
        # Call the main function to run 
        model_states_listener()
    
    # will stop running if user or some process has indicated to stop
    except rospy.ROSInterruptException:
        pass

