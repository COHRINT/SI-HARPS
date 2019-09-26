#!/usr/bin/env python

# Example ROS node for publishing AirSim images.

# AirSim Python API
# import setup_path 
import airsim
import sys, select, termios, tty, signal

import rospy

# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped

class goalpub():
    def __init__(self):
        self.pub = rospy.Publisher("Drone1/Goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/Strategy", Int16, self.giveDroneGoal)
        rospy.init_node('droneGoalPub', anonymous=False)
        self.rate = rospy.Rate(10) # 10hz

        # self.client = airsim.MultirotorClient()
        # self.client.confirmConnection()
        # self.client.enableApiControl(True, "Drone1")
        # self.client.armDisarm(True, "Drone1")

        self.speed = 10
        
        while not rospy.is_shutdown():
            self.rate.sleep()

    def giveDroneGoal(self, msg):
        print("New strategy, Moving")
        drone_state = self.client.getMultirotorState(vehicle_name="Drone1")
        print(drone_state)
        # drone_state = self.client.simGetVehiclePose()
        pos_ned = drone_state.kinematics_estimated.position
        # orientation_ned = drone_state.kinematics_estimated.orientation

        # self.client.moveToPositionAsync(pos_ned.x_val, pos_ned.y_val+10, pos_ned.z_val, self.speed, timeout_sec=10, drivetrain=airsim.DrivetrainType.ForwardOnly, yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0))
        # if msg is 1:



if __name__ == '__main__':
    try:
        goalpub()
    except rospy.ROSInterruptException:
        pass
