#!/usr/bin/env python3
from __future__ import print_function, division
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
import tf
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import struct

# !!! COPYRIGHT NOTICE !!!:
# This script is based on the "nav_2d_voxels" package by Dora Lourenço
# The original package can be found here: https://gitlab.ingeniarius.pt/semfire_ing_uc/navigation/nav_2d_voxels


def extract_xyz(pc2_msg):
	"""
	Extracts the x, y, z coordinates from a PointCloud2 message.
	"""
	fmt = 'fff'  # format for x, y, z
	point_step = pc2_msg.point_step
	data = pc2_msg.data
	xyz = []

	for i in range(0, len(data), point_step):
		x, y, z = struct.unpack_from(fmt, data, offset=i)
		xyz.append([x, y, z])

	return np.array(xyz)

class LaserInterface2:
	"""
	Class that interacts with ROS and pointclouds, returns 3D numpy arrays.
	"""

	def __init__(self):
		self.listener_octomap = tf.TransformListener()
		self.listener_bobcat = tf.TransformListener()

	def listen_map_tf(self):
		"""
		Listen the tf between Odom and Map and returns the Rotation
		"""
		# listener_bobcat = tf.TransformListener()

		self.listener_bobcat.waitForTransform("map","odom", rospy.Time(0),rospy.Duration(4.0)) #map to bobcat_base

		translation, orientation = self.listener_bobcat.lookupTransform("map", "odom", rospy.Time(0)) # map to bobcat_base

		orientation_list = [orientation[0], orientation[1], orientation[2], orientation[3]]

		roll, pitch, yaw = euler_from_quaternion(orientation_list)

		yaw = 0.0

		orientation_quat = quaternion_from_euler(roll, pitch, yaw)

		quaternion = Quaternion(w=orientation_quat[3], x=orientation_quat[0], y=orientation_quat[1], z=orientation_quat[2])

		# Obtain rotation matrix from quaternion
		rotation = quaternion.rotation_matrix

		return translation, roll, pitch


	def listen_octomap_tf(self):
		"""
		Listen the yaw rotation and the translation from map to bobcat_base
		"""
		# listener_octomap = tf.TransformListener()

		time = self.listener_octomap.getLatestCommonTime("map", "odom") #map to base_footprint
		self.listener_octomap.waitForTransform("map", "odom", time, rospy.Duration(4.0)) #map to base_footprint

		translation, orientation = self.listener_octomap.lookupTransform("map", "odom", time) # map to base_footprint

		orientation_list = [orientation[0], orientation[1], orientation[2], orientation[3]]

		roll, pitch, yaw = euler_from_quaternion(orientation_list)

		roll = 0.0
		pitch = 0.0

		orientation_quat = quaternion_from_euler(roll, pitch, yaw)

		quaternion = Quaternion(w=orientation_quat[3], x=orientation_quat[0], y=orientation_quat[1], z=orientation_quat[2])

		# Obtain rotation matrix from quaternion
		rotation = quaternion.rotation_matrix

		return translation, rotation, time

	def transform_xyz_octomap(self, pc2_msg):
		"""
		Returns the xyz coordinates of Back LIDAR points in the Bobcat Base referential.
		"""
		# Listen the transform between Front LIDAR and Bobcat Base
		translation, rotation, time = self.listen_octomap_tf()
  
		# Print the fields of the PointCloud2 message for debugging
		#rospy.loginfo(f"PointCloud2 fields: {pc2_msg.fields}")
		#rospy.loginfo(f"PointCloud2 data length: {len(pc2_msg.data)}")

		# Transform the PointCloud2 message in xyz coordinates 
		#xyz_octomap = np.matrix(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg))
		xyz_octomap = extract_xyz(pc2_msg)

		# Apply the rotation matrix
		xyz_rotated = np.empty((xyz_octomap.shape[0],xyz_octomap.shape[1]))

		for point in range(xyz_octomap.shape[0]):
			point_transpose = np.transpose(xyz_octomap[point,:])

			#point_rot = rotation*point_transpose
			point_rot = np.dot(rotation, point_transpose)

			xyz_rotated[point] = np.transpose(point_rot)

		xyz_rotated = np.array(xyz_rotated)

		# Apply the translation
		xyz_transform = np.empty((xyz_octomap.shape[0],xyz_octomap.shape[1]))

		xyz_transform[:,0] = xyz_rotated[:,0] + translation[0]
		xyz_transform[:,1] = xyz_rotated[:,1] + translation[1]
		xyz_transform[:,2] = xyz_rotated[:,2] + translation[2]

		return xyz_transform, translation, time


	def get_point_cloud_octomap(self):
		"""
		Returns XYZ points that result from octomap pointcloud
		"""
		use_last_pc2 = False
		# Wait for a message from Octomap
		try:
			pc2_octomap = rospy.wait_for_message('preProcessed_pointcloud', PointCloud2, 3.0)
		except rospy.ROSException as e:
			rospy.logwarn("Timeout while waiting for a new pointcloud. Using last one.")
			use_last_pc2 = True
			return None, None, None, None, use_last_pc2

		if len(pc2_octomap.data) == 0:
			rospy.logwarn("Received an empty pointcloud. Waiting for a fresh one!")
			return None, None, None, None, use_last_pc2

		# Transform a message type PointCloud2 into xyz coordinates and Bobcat Base yaw and translation
		xyz_octomap, translation, time = self.transform_xyz_octomap(pc2_octomap)

		return xyz_octomap, pc2_octomap.width, translation, time, use_last_pc2
