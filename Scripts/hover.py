#!/usr/bin/env python2
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Empty, Header
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_from_matrix, quaternion_from_euler, quaternion_multiply
from gtsam import Point3, Pose3, Rot3
from TrajGen import TrajectoryGenerator
from math import pi, sin, cos
import pickle
from nav_msgs.msg import Path


class Hover():

	
	def __init__(self):
		rospy.init_node('GoForward', anonymous=False)
		self.velPub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
		self.pathPub = rospy.Publisher('world/ref_traj', Path, queue_size=10)
		
		rospy.Subscriber('/vrpn_client_node/bebop/pose',PoseStamped, self.poseCallback)
		import time
		time.sleep(1.0)
		r = rospy.Rate(10)

		self.kp_x = 0.8
		self.kd_x = 0.005
		self.ki_x = 0.0001

		self.kp_y = 0.8
		self.kd_y = 0.005
		self.ki_y = 0.0001

		self.kp_z = 0.15
		self.kd_z = 0.03
		self.ki_z = 0.01

		self.kp_yaw = 0.1
		self.kd_yaw = 0.03
		self.ki_yaw = 0.01

		self.d_x = 0
		self.d_y = 0
		self.d_z = 0

		self.ix = 0
		self.iy = 0
		self.iz = 0
		self.iyaw = 0
		self.goalcurr = [0,0,0,0]
		self.goalcurr[0] = self.sTcurr_ros.pose.position.x 
		self.goalcurr[1] = self.sTcurr_ros.pose.position.y
		self.goalcurr[2] = self.sTcurr_ros.pose.position.z
		self.goalTarget = [1,1,1,0]
		
		self.lastT = Pose3()
		self.lastyaw = 0
		self.move_cmd = Twist()
		self.alpha = 0.9

	
		
	
	def calculate_position (self,c,t):
		return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]

	def calculate_velocity (self,c,t):
		return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4] 

	def calculate_acceleration (self,c,t):
		return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]  

	def display_trajectory(self):
		path = Path()
		point = Point()
		quat = Quaternion()
		pose_stamped = PoseStamped()
		quat.x = 0
		quat.y = 0
		quat.z = 0
		quat.w = 1
		for i in range(100):
			point.x = self.calculate_position(self.x_coeffs, 0.1 * i)
			point.y = self.calculate_position(self.y_coeffs, 0.1 * i)
			point.z = self.calculate_position(self.z_coeffs, 0.1 * i)
			pose_stamped.pose.position = point
			pose_stamped.pose.orientation = quat
			path.poses.append(pose_stamped)
			path.header.frame_id = "world"	
			self.pathPub.publish(path)

	
	def run(self):
		file = open('SetP','rb')
		
		reached = 1
		self.iteration = 0

		self.goalcurr[0] = self.sTcurr_ros.pose.position.x 
		self.goalcurr[1] = self.sTcurr_ros.pose.position.y
		self.goalcurr[2] = self.sTcurr_ros.pose.position.z 
		
		print self.goalcurr[2]
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				if reached == 1:
					
					err_x = self.sTcurr_ros.pose.position.x - self.goalcurr[0]
					err_y = self.sTcurr_ros.pose.position.y - self.goalcurr[1]
					err_z = self.sTcurr_ros.pose.position.z - self.goalcurr[2]
					err = np.array([err_x,err_y])
					error = np.linalg.norm(err)
					# print("ERROR: ", error)
					reached = 0
					

					waypoints = [self.goalcurr,self.goalTarget]
					print waypoints
					# self.goalcurr = [data.x,data.y,data.z]
					# print("Goal:", self.goalcurr )
					traj = TrajectoryGenerator(waypoints[0],waypoints[1], 10)
					traj.solve()
				
					self.x_coeffs = traj.x_c
					self.y_coeffs = traj.y_c
					self.z_coeffs = traj.z_c
					self.display_trajectory()
					print(self.goalcurr)
					
				# elif reached == 2:
				# 	waypoints = [[self.sTcurr_ros.pose.position.x,self.sTcurr_ros.pose.position.y,self.sTcurr_ros.pose.position.z],self.goalcurr]
				# 	traj = TrajectoryGenerator(waypoints[0],waypoints[1], 10)
				# 	traj.solve()
				
				# 	self.x_coeffs = traj.x_c
				# 	self.y_coeffs = traj.y_c
				# 	self.z_coeffs = traj.z_c
				# 	self.iteration = 0

				

				self.iteration += 1
				if self.iteration == 100:
					self.iteration = 0
					self.alpha = 1
					reached = 1
		
				goal = [0,0,0,0]
				goal[0] = self.calculate_position (self.x_coeffs, 0.1 * self.iteration)
				goal[1] = self.calculate_position (self.y_coeffs, 0.1 * self.iteration)
				goal[2] = self.calculate_position (self.z_coeffs, 0.1 * self.iteration)
				goal[3] = 0

				
				des_acc_x = self.calculate_acceleration (self.x_coeffs, 0.1 * self.iteration)
				des_acc_y = self.calculate_acceleration (self.y_coeffs, 0.1 * self.iteration)
				des_acc_z = self.calculate_acceleration (self.z_coeffs, 0.1 * self.iteration)

				des_vel_x = self.calculate_velocity (self.x_coeffs, 0.1 * self.iteration)
				des_vel_y = self.calculate_velocity (self.y_coeffs, 0.1 * self.iteration)
				des_vel_z = self.calculate_velocity (self.z_coeffs, 0.1 * self.iteration)


				self.sTg = Pose3(Rot3.Rz(goal[3]), Point3(goal[0], goal[1], goal[2]))
				
				q_rot = quaternion_from_euler(0,0,pi * 0.000)
				R0 = quaternion_multiply([self.sTcurr_ros.pose.orientation.x,self.sTcurr_ros.pose.orientation.y,self.sTcurr_ros.pose.orientation.z,self.sTcurr_ros.pose.orientation.w],q_rot)
				R = quaternion_matrix(R0)[0:3,0:3]
				# R = quaternion_matrix([self.sTcurr_ros.pose.orientation.x,self.sTcurr_ros.pose.orientation.y,self.sTcurr_ros.pose.orientation.z,self.sTcurr_ros.pose.orientation.w])[0:3,0:3]
				self.sTcurr = Pose3(Rot3(R), Point3(self.sTcurr_ros.pose.position.x, self.sTcurr_ros.pose.position.y, self.sTcurr_ros.pose.position.z))
				# roll, pitch, yaw = euler_from_quaternion([self.sTcurr_ros.pose.orientation.x,self.sTcurr_ros.pose.orientation.y,self.sTcurr_ros.pose.orientation.z,self.sTcurr_ros.pose.orientation.w])
				roll, pitch, yaw = euler_from_quaternion(R0)
				
				self.sTg2 = Pose3(Rot3.Rz(0), Point3(self.goalcurr[0], self.goalcurr[1], self.goalcurr[2]))
				
				currTg = self.sTcurr.between(self.sTg)
				currTg2 = self.sTcurr.between(self.sTg2)
			
				# print("YAW:", currTg2.rotation().rpy()[2])
				# print ("X Error" , currTg2.x())
				# print ("Y Error" , currTg2.y())
			
								
				

				currTg = self.sTcurr.between(self.sTg)

				currTg_yaw = currTg.rotation().rpy()[2]
				# print(currTg_yaw)
				u_pitch = 0
				u_roll = 0

				body_vx = ((currTg.x() - self.lastT.y())) / 0.1
				body_vy = ((currTg.y() - self.lastT.y())) / 0.1

				vx_error = des_vel_x * cos(currTg_yaw) - des_vel_y * sin(currTg_yaw)
				
				if (abs(currTg2.x()) <=0.01):
					u_pitch = 0
					# u_pitch =  self.kp_x * currTg2.x()
				else: 
					u_pitch =  (1 - self.alpha) * (self.kp_x * currTg2.x() + self.kd_x*(currTg2.x() - self.lastT.x()) + min(max(-0.1,self.ki_x*self.ix),0.1))
					u_pitch += self.alpha * (des_acc_x * cos(currTg_yaw) - des_acc_y * sin(currTg_yaw))  
					# u_pitch = 0

				if (abs(currTg2.y()) <=0.01):
					u_roll = 0

					# u_roll =  self.kp_y * currTg2.y()
				else: 
					# 
					u_roll =  (1 - self.alpha) * (self.kp_y * currTg2.y()  + self.kd_y*(currTg2.y() - self.lastT.y()) + min(max(-0.1,self.ki_y*self.iy),0.1))
					u_roll += self.alpha * (des_acc_x * sin(currTg_yaw) + des_acc_y * cos(currTg_yaw))

				# print("YAW:", currTg_yaw)
					# u_roll = 0
				
				# u_yaw = self.kp_yaw*currTg_yaw + self.kd_yaw*(currTg_yaw - self.lastyaw)
			
				if (abs(currTg.z()) <=0.01):
					u_z =  self.kp_z * currTg.z()
				else: 
					# u_z =  self.calculate_velocity(self.z_coeffs, 0.1 * self.iteration)

					# u_z =  self.kp_z * currTg.z() + self.calculate_velocity(self.z_coeffs, 0.1 * self.iteration)
					u_z = 0

				if (abs(currTg2.rotation().rpy()[2]) <=0.01):
					u_yaw = 0
				else: 
					# u_yaw = 0
					u_yaw =  self.alpha * sin(currTg_yaw)  +  (1 - self.alpha) * (sin(currTg2.rotation().rpy()[2])) 

				
				self.lastT = currTg2
				self.lastyaw = currTg2.rotation().rpy()[2]

				self.ix += currTg2.x()
				self.iy += currTg2.y()
				self.iz += currTg2.z()
				self.iyaw += currTg2.rotation().rpy()[2]
				

			
			
				self.move_cmd.linear.x = min(max(-1,u_pitch),1)
				# print ("X Control" , self.move_cmd.linear.x)
				# print ("X Error" , currTg.x())
			
				self.move_cmd.linear.y = min(max(-1,u_roll),1)
				# print ("Y Control" , self.move_cmd.linear.y)
				# print ("Y Error" , currTg.y())
			
				self.move_cmd.linear.z = min(max(-1,u_z),1)
				# print ("Z Control" , self.move_cmd.linear.z)
				# print ("Z Error" , currTg.z())
				
				self.move_cmd.angular.z = min(max(-1,u_yaw),1)
				# print ("Yaw Control" , self.move_cmd.angular.z)
				# print ("Yaw Error" , currTg2.rotation().rpy()[2])
				
				# print(self.sTcurr_ros.pose.position)

				if (abs(self.sTcurr_ros.pose.position.x) > 2.5 or abs(self.sTcurr_ros.pose.position.y) > 2.5 or abs(self.sTcurr_ros.pose.position.z) > 2):
					self.move_cmd.linear.x = 0
					self.move_cmd.linear.y = 0
					self.move_cmd.linear.z = 0
					self.move_cmd.angular.z = 0
					print('DANGER')
					# self.goalcurr = [0,0,0]
					# reached = 2
					# rospy.Publisher('bebop/land', Empty, queue_size = 20)
					# rospy.spin()


				# self.move_cmd.linear.z = 0


				self.velPub.publish(self.move_cmd)
				self.alpha *= 0.8
				r.sleep()
			except KeyboardInterrupt:
				rospy.Publisher('bebop/land', Empty, queue_size = 10)
				rospy.spin()

	def poseCallback(self, msg):
		self.sTcurr_ros = msg
		# print self.sTcurr_ros

	

def main():
	H = Hover()
	import time
	time.sleep(1.0)
	H.run()
	rospy.spin()


if __name__ == '__main__':
    main()
