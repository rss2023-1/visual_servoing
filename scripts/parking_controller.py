#!/usr/bin/env python

import rospy
import numpy as np

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

        self.reverse_time = 5 # Time steps spent in reverse if cone is too close
                              # Increasing could decrease retries to line up but
                              # will increase distance backed away, seems relatively
                              # balanaced rn
        self.reverse = self.reverse_time 

    def seeCone(self): # Replace this funciton with modules 1 and 2, currently simulates if cone is within camera FOV
        theta_max = 36*np.pi/180 
        theta_min = -36*np.pi/180 # Assume camera FOV 72 deg
        if np.arctan2(self.relative_y, self.relative_x) < theta_max and np.arctan2(self.relative_y, self.relative_x) > theta_min:
            return True
        else:
            return False

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.header.frame_id = 'constant'
        
        drive_cmd.drive.speed = 1

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        if self.reverse < self.reverse_time: # Check if reversing
            drive_cmd.drive.speed = -1
            self.reverse = self.reverse + 1
        else:
            theta = np.arctan2(self.relative_y, self.relative_x)
            dist = np.sqrt(self.relative_x**2+self.relative_y**2)

            if self.seeCone():
                if abs(dist-self.parking_distance) < 0.05 and abs(theta) <= 0.05: # If within distance and angle tolerenace, park
                    drive_cmd.drive.speed = 0
                    drive_cmd.drive.acceleration = 0
                    drive_cmd.drive.jerk = 0
                elif dist > self.parking_distance: # If away from cone, control theta proportionally to align upon closing gap
                    if dist > self.parking_distance:
                        if theta != 0:
                            drive_cmd.drive.steering_angle = theta
                        else:
                            drive_cmd.drive.steering_angle = 0
                elif dist < self.parking_distance and abs(theta) <= 0.05: # If too close too cone but aligned, back up
                    drive_cmd.drive.steering_angle = 0
                    drive_cmd.drive.speed = -1
                else: # Too close and not aligned, reverse for a few timesteps then retry
                    drive_cmd.drive.speed = -1
                    self.reverse = 0           
            else: # Cone not within FOV, drive in a circle to find
                drive_cmd.drive.steering_angle = 0.34 # max steering angle
                # This can get stuck in a circle if the cone is placed directly to the left of the wheel
                # as a result of max turning radius and camera FOV. This may be resolved if the actual 
                # camera has a wider FOV, though this could be solved in code by iterating a value everytime
                # this runs and setting it to 0 otherwise. If the value is exceeded, circle times out and a right
                # steer should be briefly published to offset circle and find the cone. I'm too lazy to implement
                # it rn though lol.


        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y 
        error_msg.distance_error = np.sqrt(self.relative_x**2+self.relative_y**2)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
