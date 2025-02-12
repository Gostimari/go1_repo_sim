#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf import TransformListener
import csv
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from datetime import datetime
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import perf_counter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from unitree_legged_msgs.msg import MotorState, BmsState

class MetricsExtractor:

	def __init__(self):
		"""
		Function Description 
		"""
		# Temporary/auxiliary variables:
		self.odom_msg = Odometry()
		self.nav_goal_msg = MoveBaseActionGoal()
		self.bobcat_map_listener = TransformListener()
		self.vel = Twist()
		self.imu = Imu()
		self.robot_navigating = False
		self.once = False
		self.prev_trav_dist = 0
		self.prev_cumulative_height_gain = 0
		self.prev_cumulative_height_loss = 0
		self.prev_x = 0
		self.prev_y = 0
		self.prev_z = 0
		self.elapsed_time_secs = 0 # Total time elapsed moving from start->goal
		self.elapsed_time_nsecs = 0
		self.orientation = []
		self.RPY = []
		self.tic = []
		self.timeout = 350.0
		self.trigger_end = False
		self.filename_raw = datetime.now().strftime('logfile_raw-%Y-%m-%d-%H-%M')
		self.filename_metrics = datetime.now().strftime('logfile_metrics-%Y-%m-%d-%H-%M')
		self.start_time = None
		self.end_time = None
		self.goal_reached = False
		self.goal_processed = True  # New flag to track if the goal has been processed
		self.current_goal_id = None  # Track the current goal ID
		self.failed_goals_count = 0
		self.goal_counter = 0
		# Metrics:
		self.travelled_distance = 0 # Total distance travelled by the robot
		self.total_velocity = 0 # Total velocity accumulated by the robot
		self.velocity_count = 0
		self.mean_velocity = 0 # Mean velocity of the robot
		self.imu_derivation_x = 0
		self.imu_derivation_y = 0
		self.imu_derivation_z = 0
		self.instalibity_index = 0
		self.imu_append_x = []
		self.imu_append_y = []
		self.imu_append_z = []
		self.mean_torque = 0
		self.fl_calf_append = 0
		self.fl_hip_append = 0
		self.fl_thigh_append = 0
		self.fr_calf_append = 0
		self.fr_hip_append = 0
		self.fr_thigh_append = 0
		self.rl_calf_append = 0
		self.rl_hip_append = 0
		self.rl_thigh_append = 0
		self.rr_calf_append = 0
		self.rr_hip_append = 0
		self.rr_thigh_append = 0
		self.goal_counter = 0
		self.goal = True
		self.last_status = 0
		#
		self.file_raw = open(f'/home/duarte/noetic-sim/src/metrics_extractor/logfiles/{self.filename_raw}.csv', mode='a')
		self.file_metrics = open(f'/home/duarte/noetic-sim/src/metrics_extractor/logfiles/{self.filename_metrics}.csv', mode='a')

	def read_odom_and_goal(self):
		"""
		This function is responsible for receiving the navigation goal at the
		beginning of the path and periodically read the robot's
		odometry information. 
		"""
		self.odom_msg = rospy.wait_for_message("lio_odom", Odometry)
		self.status_msg = rospy.wait_for_message("move_base/status", GoalStatusArray)
		self.vel_msg = rospy.wait_for_message("cmd_vel", Twist)
		self.imu_msg = rospy.wait_for_message("trunk_imu", Imu)
  
		self.fl_calf_msg = rospy.wait_for_message("/go1_gazebo/FL_calf_controller/state", MotorState)
		self.fl_hip_msg = rospy.wait_for_message("/go1_gazebo/FL_hip_controller/state", MotorState)
		self.fl_thigh_msg = rospy.wait_for_message("/go1_gazebo/FL_thigh_controller/state", MotorState)
		self.fr_calf_msg = rospy.wait_for_message("/go1_gazebo/FR_calf_controller/state", MotorState)
		self.fr_hip_msg = rospy.wait_for_message("/go1_gazebo/FR_hip_controller/state", MotorState)
		self.fr_thigh_msg = rospy.wait_for_message("/go1_gazebo/FR_thigh_controller/state", MotorState)
		self.rl_calf_msg = rospy.wait_for_message("/go1_gazebo/RL_calf_controller/state", MotorState)
		self.rl_hip_msg = rospy.wait_for_message("/go1_gazebo/RL_hip_controller/state", MotorState)
		self.rl_thigh_msg = rospy.wait_for_message("/go1_gazebo/RL_thigh_controller/state", MotorState)
		self.rr_calf_msg = rospy.wait_for_message("/go1_gazebo/RR_calf_controller/state", MotorState)
		self.rr_hip_msg = rospy.wait_for_message("/go1_gazebo/RR_hip_controller/state", MotorState)
		self.rr_thigh_msg = rospy.wait_for_message("/go1_gazebo/RR_thigh_controller/state", MotorState)
  
		##Add energy consumption with Bms Stat -> procurar o topico correto para a sim e real

		#self.bobcat_map_listener.waitForTransform("map","base", rospy.Time(0),rospy.Duration(4.0))
		#self.orientation = self.bobcat_map_listener.lookupTransform("odom", "base", rospy.Time(0))[1]

		if self.goal == True:
			#self.nav_goal_msg = rospy.wait_for_message("move_base/goal", MoveBaseActionGoal)
			self.goal = False
			self.tic = perf_counter()

	def metrics_calculator(self):
		"""
		This function is responsible for using the odometry information to
		calculate the required metrics.
		"""
		##### Section 1 - Travelled distance calculation #####
		current_x = self.odom_msg.pose.pose.position.x
		current_y = self.odom_msg.pose.pose.position.y
		current_z = self.odom_msg.pose.pose.position.z


		if self.once == False: # The sole purpose of this block is to initialize the three variables
			self.prev_x = current_x
			self.prev_y = current_y
			self.prev_z = current_z
			self.imu_append_x = np.array([self.imu_msg.linear_acceleration.x])
			self.imu_append_y = np.array([self.imu_msg.linear_acceleration.y])
			self.imu_append_z = np.array([self.imu_msg.linear_acceleration.z])
			#self.fl_calf_append = np.array([self.fl_calf_msg.tauEst])
			#self.fl_hip_append = np.array([self.fl_hip_msg.tauEst])
			#self.fl_thigh_append = np.array([self.fl_thigh_msg.tauEst])
			#self.fr_calf_append = np.array([self.fr_calf_msg.tauEst])
			#self.fr_hip_append = np.array([self.fr_hip_msg.tauEst])
			#self.fr_thigh_append = np.array([self.fr_thigh_msg.tauEst])
			#self.rl_calf_append = np.array([self.rl_calf_msg.tauEst])
			#self.rl_hip_append = np.array([self.rl_hip_msg.tauEst])
			#self.rl_thigh_append = np.array([self.rl_thigh_msg.tauEst])
			#self.rr_calf_append = np.array([self.rr_calf_msg.tauEst])
			#self.rr_hip_append = np.array([self.rr_hip_msg.tauEst])
			#self.rr_thigh_append = np.array([self.rr_thigh_msg.tauEst])
			self.once = True

		self.travelled_distance = self.prev_trav_dist + np.sqrt(np.square(current_x - self.prev_x) + np.square(current_y - self.prev_y) + np.square(current_z - self.prev_z))
		# print(f"Travelled distance = {self.travelled_distance:.3f}")
        
        print(f"Travelled distance = {self.travelled_distance:.3f}")

		self.prev_x = current_x
		self.prev_y = current_y
		self.prev_trav_dist = self.travelled_distance
		##### End of section 1 #####

		##### Section 2 - Velocity Calculation #####
		self.total_velocity = self.vel_msg.linear.x
		#self.velocity_count += 1
		#self.mean_velocity = self.total_velocity / self.velocity_count
		self.mean_velocity = self.total_velocity
		##### End of section 2 #####
		
		print(f"Velocity = {self.mean_velocity:.3f}")
  
		##### Section 3 - Instability Index #####
		self.imu_append_x = np.append(self.imu_append_x, [self.imu_msg.linear_acceleration.x])
		self.imu_append_y = np.append(self.imu_append_y, [self.imu_msg.linear_acceleration.y])
		self.imu_append_z = np.append(self.imu_append_z, [self.imu_msg.linear_acceleration.z])
		self.imu_derivation_x = np.std(self.imu_append_x)
		self.imu_derivation_y = np.std(self.imu_append_y)
		self.imu_derivation_z = np.std(self.imu_append_z)

		self.instalibity_index = np.sqrt(np.square(self.imu_derivation_x) + np.square(self.imu_derivation_y) + np.square(self.imu_derivation_z))
		#print(f"Instability index = {self.instalibity_index:.3f}")
		##### End of section 3 #####
		
		print(f"Instability index = {self.instalibity_index:.3f}")
  
		##### Section 4 - Torque Mean #####
		self.fl_calf_append = self.fl_calf_msg.tauEst)
		self.fl_hip_append = self.fl_hip_msg.tauEst])
		self.fl_thigh_append = self.fl_thigh_msg.tauEst)
		self.fr_calf_append = self.fr_calf_msg.tauEst)
		self.fr_hip_append = self.fr_hip_msg.tauEst)
		self.fr_thigh_append = self.fr_thigh_msg.tauEst)
		self.rl_calf_append = self.rl_calf_msg.tauEst)
		self.rl_hip_append = self.rl_hip_msg.tauEst)
		self.rl_thigh_append = self.rl_thigh_msg.tauEst)
		self.rr_calf_append = self.rr_calf_msg.tauEst)
		self.rr_hip_append = self.rr_hip_msg.tauEst)
		self.rr_thigh_append = self.rr_thigh_msg.tauEst)
  
		self.mean_torque = (self.fl_calf_append + self.fl_hip_append + self.fl_thigh_append + self.fr_calf_append + self.fr_hip_append + self.fr_thigh_append + self.rl_calf_append + self.rl_hip_append + self.rl_thigh_append + self.rr_calf_append + self.rr_hip_append + self.rr_thigh_append) / 12
		##### End of section 4 #####
		
		print(f"Mean torque = {self.mean_torque:.3f}")

		#toc = perf_counter()

		#if toc - self.tic >= self.timeout:
		#	self.trigger_end = True


	def fill_logfile(self):
		"""
		Function responsible for writing the required logfiles with the data
		"""
		writer_raw = csv.writer(self.file_raw, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		writer_raw.writerow(["%.3f" % self.prev_x, "%.3f" % self.prev_y,
		 "%.6f" % self.prev_z, self.odom_msg.header.stamp.secs, self.odom_msg.header.stamp.nsecs])
		writer_metrics = csv.writer(self.file_metrics, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		writer_metrics.writerow(["%.3f" % self.travelled_distance, "%.3f" % self.mean_velocity, "%.3f" % self.instalibity_index, "%.3f" % self.mean_torque])

	def detect_end_condition(self):
		if self.status_msg.status_list:
			current_status = self.status_msg.status_list[0].status
			current_goal_id = self.status_msg.status_list[0].goal_id.id

			# Check if the goal ID has changed (new goal received)	
			if current_status != self.last_status:
				if self.last_status == 1 and current_status == 3:
					self.goal_counter += 1
					self.goal_reached = True
					self.goal_processed = False  # Reset the processed flag for the new goal
				self.last_status = current_status

			# Goal reached (status 3) and not yet processed
			if self.goal_reached and not self.goal_processed:
				writer_raw = csv.writer(self.file_raw, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_raw.writerow(["REACHED GOAL %d" % self.goal_counter])
				writer_metrics = csv.writer(self.file_metrics, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_metrics.writerow(["REACHED GOAL %d" % self.goal_counter])
				print(f"Goal {self.goal_counter} reached")
				if self.goal_counter == 3:
					self.file_raw.close()
					self.file_metrics.close()
					self.trigger_end == True
					print("All goals reached. Exiting...")
					exit()
				self.goal_reached = False
				self.goal_processed = True
			elif current_status == 4 or self.trigger_end:
				writer_raw = csv.writer(self.file_raw, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_raw.writerow(["ABORTED"])
				writer_metrics = csv.writer(self.file_metrics, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
				writer_metrics.writerow(["ABORTED"])
				print("Goal aborted")
				#exit()


	def main(self):
		"""
		Main function, responsible for calling the remaining functions.
		"""
		rate = rospy.Rate(5)
  
		while not rospy.is_shutdown():

			self.read_odom_and_goal()
			self.metrics_calculator()
			self.fill_logfile()
			self.detect_end_condition()
			rate.sleep()




if __name__ == "__main__":

	rospy.init_node('metrics_extractor')

	metrics = MetricsExtractor()
	metrics.main()
