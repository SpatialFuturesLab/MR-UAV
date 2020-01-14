#!/usr/bin/env python2
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Empty, Header
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix, quaternion_from_euler, quaternion_multiply
from TrajGen import TrajectoryGenerator
from math import pi, sin, cos
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
import math, tf

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose

class States:
    """States class defines the current state of the Quadcopter
    """
    def __init__(self, curr):
        self.currentState = curr
        self.stateTransition = dict()
        self.stateTransition["IDLE"] = {"TAKE_OFF": "IN_FLIGHT","LAND": "IDLE", "AUTONOMOUS_ENABLE": "IDLE", "AUTONOMOUS_DISABLE": "IDLE", "NAVIGATE": "IDLE", "HOVER": "IDLE"}
        self.stateTransition["IN_FLIGHT"] = {"TAKE_OFF": "IN_FLIGHT","LAND": "IDLE", "AUTONOMOUS_ENABLE": "HOVERING", "AUTONOMOUS_DISABLE": "IN_FLIGHT", "NAVIGATE": "IN_FLIGHT", "HOVER": "IN_FLIGHT"}
        self.stateTransition["HOVERING"] = {"TAKE_OFF": "IN_FLIGHT","LAND": "IDLE", "AUTONOMOUS_ENABLE": "HOVERING", "AUTONOMOUS_DISABLE": "IN_FLIGHT", "NAVIGATE": "NAVIGATING", "HOVER": "HOVERING"}
        self.stateTransition["NAVIGATING"] = {"TAKE_OFF": "IN_FLIGHT","LAND": "IDLE", "AUTONOMOUS_ENABLE": "NAVIGATING", "AUTONOMOUS_DISABLE": "IN_FLIGHT", "NAVIGATE": "NAVIGATING", "HOVER": "HOVERING"}
 
    def get_next_state(self, action):
        """args: action
        Given an action, this function updates the state of the quadcopter and prints the current state
        """
        self.currentState = self.stateTransition[self.currentState][action]
        print "Quad State: {0}".format(self.currentState)

class TrajectoryGenerator():
    """TrajectoryGenerator function generates a quintic spline trajectory from the start position to end position
    """
    def __init__(self, start_pos, des_pos, T, start_vel = [0,0,0], des_vel = [0,0,0], start_acc = [0,0,0], des_acc = [0,0,0]):
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.T = T

    def solve(self):
        A = np.array(
            [[0,0,0,0,0,1],
            [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
            [0,0,0,0,1,0],
            [5 * self.T**4, 4 * self.T**3, 3 * self.T**2, 2 * self.T, 1, 0],
            [0,0,0,2,0,0],
            [20 * self.T**3, 12 * self.T**2, 6 * self.T, 2, 0, 0]
            ])
        b_x = np.array(
            [[self.start_x],
            [self.des_x],
            [self.start_x_vel],
            [self.des_x_vel],
            [self.start_x_acc],
            [self.des_x_acc]
            ])

        b_y = np.array(
            [[self.start_y],
            [self.des_y],
            [self.start_y_vel],
            [self.des_y_vel],
            [self.start_y_acc],
            [self.des_y_acc]
            ])
            
        b_z = np.array(
            [[self.start_z],
            [self.des_z],
            [self.start_z_vel],
            [self.des_z_vel],
            [self.start_z_acc],
            [self.des_z_acc]
            ])
        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)
            
class PID(object):
    """PID class returns the pid error
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prevTime = None
        self.prevError = 0
        self.cumError = 0

    def calculate_pid(self, error):
        cur_time = rospy.get_time()
        if self.prevTime == None:
            self.prevTime = cur_time
            self.prevError = error
            return self.kp * error
        dt = cur_time - self.prevTime
        self.prevTime = cur_time
        self.cumError += error * dt
        pid = self.kp * error + self.kd * (error - self.prevError) / dt + min(max(-0.2, self.ki * self.cumError),0.2) 
        self.prevError = error
        return pid

class PositionController:
    """PositionController subscribes to setpoint and publishes control commands to the quadcopter
    """
    def __init__(self):
        rospy.init_node('PositionController', anonymous=False)
        
        self.quad = States("IDLE")
        
        self.capturePoseBebop = rospy.Publisher('bebop/captured_pose', PoseStamped, queue_size=10)
        self.capturePoseGeom = rospy.Publisher('GeomTool/captured_pose', PoseStamped, queue_size=10)

        rospy.Subscriber('/vrpn_client_node/bebop/pose',PoseStamped, self.pose_callback_bebop)
        rospy.Subscriber('/vrpn_client_node/GeomTool/pose',PoseStamped, self.pose_callback_geom)
        rospy.Subscriber('/bebop/joy', Joy, self.joy_callback)
        rospy.Subscriber('/bebop/setpoint', Pose, self.setpoint_callback_bebop)

        self.publishTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.publishLanding = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        self.velPub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
        
        
        self.controlX = PID(0.08, 0.001, 0.05)
        self.controlY = PID(0.08, 0.001, 0.05)
        self.controlZ = PID(0.08, 0.001, 0.05)
        self.controlW = PID(0.08, 0.001, 0.05)
        
        self.waypoints = list()
        
        self.trajectory = None
        self.orientation = list()
        
        self.position = Point()
        self.nextWaypointPosition = list()
        self.yaw = None

        self.prevWaypointPosition = [0, 0, 1]

        self.bebopOrientation = 0

        self.waypoints.append([0,0,1])
        self.orientation.append(0)

        self.moveCmd = Twist()
        self.alpha = 1.0

        self.nextWaypoint = False
        self.enableAutonomous = False
        self.exit = False

        self.iteration = 0
        self.bebopPose = PoseStamped()
        self.prevBebopPoint = None
        self.currentPose = PoseStamped()

    def calculate_position (self,c,t):
        return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]

    def calculate_velocity (self,c,t):
        return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4] 

    def calculate_acceleration (self,c,t):
        return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]  

    def get_yaw_from_quaternion(self, quaternion):
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion ([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return yaw

    def joy_callback(self, msg):
        """Uses the inputs from the joystick to update the state of quadcopter
        JoyStick A: button[0]
        JoyStick B: button[1]
        JoyStick X: button[2]
        JoyStick Y: button[3]
        """
        # Enable Autonomous Mode
        if msg.buttons[4] == 1: # RB + LB
            self.enableAutonomous = not self.enableAutonomous
            if self.enableAutonomous:
                self.quad.get_next_state("AUTONOMOUS_ENABLE")
            else:
                self.quad.get_next_state("AUTONOMOUS_DISABLE")

        # Exit Code
        elif msg.buttons[2] == 1 and msg.buttons[0] == 1:   # RB + A + X
            if self.quad.currentState != "IDLE":
                self.publishLanding.publish(Empty)
                rospy.sleep(5)
            self.exit = True

        # Take Off
        elif msg.buttons[3] == 1:   # RB + Y
            self.quad.get_next_state("TAKE_OFF")
            rospy.sleep(5)

        # Land
        elif msg.buttons[0] == 1:   # RB + A
            self.quad.get_next_state("LAND")
            rospy.sleep(5)

        # Capture point from quadcopter
        elif msg.buttons[2] == 1:
            pose = self.bebopPose
            cur_pt = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            if self.prevBebopPoint is not None:
                dist = np.linalg.norm(cur_pt -  self.prevBebopPoint)
            if self.prevBebopPoint is None or dist > 0.1: 
                self.pose_pub_bebop.publish(pose)
                self.prevBebopPoint = cur_pt

    def setpoint_callback_bebop(self, msg):
        """Callback to update setpoint message
        """
        setpt = msg.position
        if self.waypoints:
            dist = np.linalg.norm(np.array(setpt) -  np.array(self.waypoints[-1]))
            if dist > 0.1:
                self.waypoints.append([setpt.x, setpt.y, setpt.z])
                self.orientation.append(self.get_yaw_from_quaternion(msg.orientation))
        else:
            self.waypoints.append([setpt.x, setpt.y, setpt.z])

    def pose_callback_bebop(self, msg):
        """Callback to update bebop position
        """
        self.bebopPose = msg
        self.position = msg.pose.position
        self.yaw = self.get_yaw_from_quaternion(msg.pose.orientation)

    def waypoint_update(self):
        """Callback to update waypoints
        """
        if self.nextWaypoint:
            if self.waypoints:
                if len(self.nextWaypointPosition)!=0:
                    self.prevWaypointPosition = self.nextWaypointPosition
                print self.waypoints
                self.nextWaypointPosition = self.waypoints.pop(0)
                self.bebopOrientation = self.orientation.pop(0)
                self.trajectory = TrajectoryGenerator(self.prevWaypointPosition,self.nextWaypointPosition, 10)
            else:
                self.quad.get_next_state("HOVER")
            self.nextWaypoint = False
        else:
            if not self.waypoints:
                self.quad.get_next_state("HOVER")

    def move_quad(self):
        """Move Quadcopter
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.exit:
            try:
                if self.quad.currentState == "NAVIGATING" or self.quad.currentState == "HOVERING":
                    self.waypoint_update()
                if self.quad.currentState == "NAVIGATING":
                    self.iteration += 1
                    if self.iteration == 100:
                        self.iteration = 0
                        self.alpha = 1.0
                        # self.nextWaypoint = True
                        self.quad.get_next_state("HOVER")

        
                    des_pos_x = self.calculate_position (self.trajectory.x_c, 0.1 * self.iteration)
                    des_pos_y = self.calculate_position (self.trajectory.y_c, 0.1 * self.iteration)
                    des_pos_z = self.calculate_position (self.trajectory.z_c, 0.1 * self.iteration)
                
                    des_acc_x = self.calculate_acceleration (self.trajectory.x_c, 0.1 * self.iteration)
                    des_acc_y = self.calculate_acceleration (self.trajectory.y_c, 0.1 * self.iteration)
                    des_acc_z = self.calculate_acceleration (self.trajectory.z_c, 0.1 * self.iteration)

                    des_vel_x = self.calculate_velocity (self.trajectory.x_c, 0.1 * self.iteration)
                    des_vel_y = self.calculate_velocity (self.trajectory.y_c, 0.1 * self.iteration)
                    des_vel_z = self.calculate_velocity (self.trajectory.z_c, 0.1 * self.iteration)

                    erx = self.nextWaypointPosition[0] - self.position.x
                    ery = self.nextWaypointPosition[1] - self.position.y
                    
                    error_x = math.cos(self.yaw) * erx + math.sin(self.yaw) * ery
                    error_y = -math.sin(self.yaw) * erx + math.cos(self.yaw) * ery

                    des_ff_x = math.cos(self.yaw) * des_vel_x + math.sin(self.yaw) * des_vel_y
                    des_ff_y = -math.sin(self.yaw) * des_vel_x + math.cos(self.yaw) * des_vel_y

                    # PID
                    self.moveCmd.linear.x = min(max(-1, (1 - self.alpha) * self.controlX.calculate_pid(error_x) + self.alpha * des_ff_x), 1)
                    self.moveCmd.linear.y = min(max(-1, (1 - self.alpha) * self.controlY.calculate_pid(error_y) + self.alpha * des_ff_y), 1)
                    self.moveCmd.linear.z = min(max(-1, (1 - self.alpha) * self.controlZ.calculate_pid(self.nextWaypointPosition[2] - self.position.z) + self.alpha * des_vel_z), 1)
                    
                    self.moveCmd.angular.z = min(max(-1,self.controlW.calculate_pid(self.bebopOrientation - self.yaw)),1)
                    self.moveCmd.angular.z = min(max(-1,self.controlW.calculate_pid(- self.yaw)),1)

                    self.alpha *= 0.9
                    self.velPub.publish(self.moveCmd)
                    r.sleep()
                
                if self.quad.currentState == "HOVERING":
                    self.iteration += 1
                    if self.iteration == 100:
                        self.iteration = 0
                        self.quad.get_next_state("NAVIGATE")
                        self.nextWaypoint = True
                    
                    erx = self.prevWaypointPosition[0] - self.position.x
                    ery = self.prevWaypointPosition[1] - self.position.y
                    
                    error_x = math.cos(self.yaw) * erx + math.sin(self.yaw) * ery
                    error_y = -math.sin(self.yaw) * erx + math.cos(self.yaw) * ery

                    # PID
                    self.moveCmd.linear.x = min(max(-1, self.controlX.calculate_pid(error_x)), 1)
                    self.moveCmd.linear.y = min(max(-1, self.controlY.calculate_pid(error_y)), 1)
                    self.moveCmd.linear.z = min(max(-1, self.controlZ.calculate_pid(self.prevWaypointPosition[2] - self.position.z)), 1)
                    self.moveCmd.angular.z = min(max(-1, self.controlW.calculate_pid(-self.yaw)), 1)

                    self.velPub.publish(self.moveCmd)
                        
                
            except Exception as e:
                print "Exception: {0}".format(str(e))
                rospy.Publisher('bebop/land', Empty, queue_size = 10)
                
    
if __name__ == '__main__':
    try:
        P = PositionController()
        P.move_quad()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print "Exception: {0}".format(str(e))
