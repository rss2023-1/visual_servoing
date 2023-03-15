#!/usr/bin/env python

import rospy
import numpy as np
import time

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        
        print(self.relative_x, self.relative_y)

        drive_cmd = AckermannDriveStamped()

        x = self.relative_x
        y = self.relative_y

        err = y/(x*( (y/x)**2+1)**0.5)*abs(x)/x

        err = min(max(-0.34, err), 0.34)
        
        drive_cmd.drive.steering_angle = err
        drive_cmd.drive.steering_angle_velocity = 0
        drive_cmd.drive.speed = 0.5 * abs(x-self.parking_distance)/(x-self.parking_distance)
        drive_cmd.drive.acceleration = 0
        drive_cmd.drive.jerk = 0
        

        if abs(y) < 0.03 and abs(x-self.parking_distance) < 0.03:
            drive_cmd.drive.speed = 0
            drive_cmd.drive.steering_angle = 0

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = (self.relative_x**2 + self.relative_y**2)**0.5

        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    rospy.init_node('ParkingController' )
    print('starting parking contr')
    ParkingController()
    rospy.spin()
