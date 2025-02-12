#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing import final
import numpy as np
from statistics import mean, median, stdev, variance
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion
import csv
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import os
import random
from matplotlib import patches

class MetricsAnalyser:

	def __init__(self):
		self.x = []
		self.y = []
		self.z = []
		self.roll = []
		self.pitch = []
		self.risk = []
		self.duration = []
		self.timestamp_secs = []
		self.timestamp_nsecs = []
		self.timestamp_secs_goal1 = []
		self.timestamp_secs_goal2 = []
		self.timestamp_secs_goal3 = []
		self.timestamp_nsecs_goal1 = []
		self.timestamp_nsecs_goal2 = []
		self.timestamp_nsecs_goal3 = []
		self.fails = []
		self.files_raw = []
		self.files_metrics = []
		self.secs_goal1 = None
		self.nsecs_goal1 = None
		self.secs_goal2 = None
		self.nsecs_goal2 = None
		self.secs_goal3 = None
		self.nsecs_goal3 = None

		# Com caminho relativo:
		# script_dir = os.path.dirname(__file__)
		# directory = os.path.join(script_dir, '../logfiles/')
		# Caminho absoluto:
		directory = "/home/duarte/noetic-sim/src/metrics_extractor/logfiles/"

		self.number_of_files=0
		for root,dirs,files in os.walk(directory):
			for file in files:
				if file.startswith("logfile_raw") and root == directory: # Reads every file whose name starts by "logfile_raw" and is inside the main directory
					self.files_raw.append(open(directory+file, 'r'))
					self.number_of_files += 1
			print(f"{self.number_of_files} different runs detected.")

	def read_logfiles(self):
		for counter in range(self.number_of_files):
			data = []
			csv_reader = csv.reader(self.files_raw[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					if row[i] == "ABORTED":
						self.fails.append(counter)
					data.append(row[i])
				line_count += 1

			aux = 0
			x = []
			y = []
			z = []
			secs = []
			nsecs = []
   
			for elem in data:

				if elem == "ABORTED":
					aux += 1
					continue
 
				if elem == "REACHED GOAL 1":
					if len(secs) >= 2 and len(nsecs) >= 2:
						self.secs_goal1 = secs[-2]
						self.nsecs_goal1 = nsecs[-1]
					continue

				if elem == "REACHED GOAL 2":
					if len(secs) >= 2 and len(nsecs) >= 2:
						self.secs_goal2 = secs[-2]
						self.nsecs_goal2 = nsecs[-1]
					continue
				
				if elem == "REACHED GOAL 3":
					if len(secs) >= 2 and len(nsecs) >= 2:
						self.secs_goal3 = secs[-2]
						self.nsecs_goal3 = nsecs[-1]
					continue
					
				if(aux == 5):
					aux = 0

				if aux == 0:
					x.append(elem)
				elif aux == 1:
					y.append(elem)
				elif aux == 2:
					z.append(elem)
				elif aux == 3:
					secs.append(elem)
				elif aux == 4:
					nsecs.append(elem)
				else:
					pass
				aux += 1

			self.x.append(x)
			self.y.append(y)
			self.z.append(z)
			self.timestamp_secs.append(secs)
			self.timestamp_nsecs.append(nsecs)
   


	def statistic_curves(self):

		max_ = 0
		for counter in range(self.number_of_files):
			if len(self.x[counter]) > max_ and counter not in self.fails:
				max_ = len(self.x[counter])

		x = []
		y = []
		z = []
		secs = []
		nsecs = []
		mean_x = 0
		mean_y = 0
		mean_z = 0
		mean_time = 0
		final_x = []
		final_y = []
		final_z = []
		sigma_x = []
		sigma_y = []
		sigma_z = []
		final_sigma_x = []
		final_sigma_y = []
		final_sigma_z = []
		final_time = []

		for i in range(1,max_):
			for counter in range(self.number_of_files):
				if counter not in self.fails and i < len(self.x[counter]):
					x.append(self.x[counter][i])
					y.append(self.y[counter][i])
					z.append(self.z[counter][i])
					secs.append(self.timestamp_secs[counter][i])
					nsecs.append(self.timestamp_nsecs[counter][i])
				else:
					continue
			# print(f"{secs=}")
			# print(f"{nsecs=}")
			x_arr = np.asanyarray(x).astype('float')
			y_arr = np.asanyarray(y).astype('float')
			z_arr = np.asanyarray(z).astype('float')
			secs_arr = np.asanyarray(secs).astype('float')
			nsecs_arr = np.asanyarray(nsecs).astype('float')
			nsecs_arr *= 1E-9
			sigma_x = np.std(x_arr)
			sigma_y = np.std(y_arr)
			sigma_z = np.std(z_arr)
			final_sigma_x.append(sigma_x)
			final_sigma_y.append(sigma_y)
			final_sigma_z.append(sigma_z)
			mean_x = np.mean(x_arr)
			mean_y = np.mean(y_arr)
			mean_z = np.mean(z_arr)
			time_arr = secs_arr + nsecs_arr
			mean_time = np.mean(time_arr)
			final_x.append(mean_x)
			final_y.append(mean_y)
			final_z.append(mean_z)
			# time_nsec = np.asanyarray(nsecs).astype('float')
			# time_nsec *= 1E-9
			final_time.append(mean_time)
			x = []
			y = []
			z = []
			secs = []
			nsecs = []
		# print(final_time)

		avg_x_minus_3std = [final_x[i] - 3* final_sigma_x[i] for i in range(len(final_x))]
		avg_x_plus_3std = [final_x[i] + 3* final_sigma_x[i] for i in range(len(final_x))]
		avg_y_minus_3std = [final_y[i] - 3* final_sigma_y[i] for i in range(len(final_y))]
		avg_y_plus_3std = [final_y[i] + 3* final_sigma_y[i] for i in range(len(final_y))]
		avg_z_minus_3std = [final_z[i] - 3* final_sigma_z[i] for i in range(len(final_z))]
		avg_z_plus_3std = [final_z[i] + 3* final_sigma_z[i] for i in range(len(final_z))]

		points1 = [[x,y] for x,y in zip(avg_x_minus_3std, avg_y_minus_3std)]
		points2 = [[x,y] for x,y in zip(avg_x_plus_3std, avg_y_plus_3std)]
		points2.reverse()
		points1 = np.asanyarray(points1)
		points2 = np.asanyarray(points2)
		points = np.append(points1,points2, axis=0)
		ply = patches.Polygon(points, alpha=0.2, facecolor='orange')

		fig1 = plt.figure(figsize=[5.12, 3.84])
		# fig1 = plt.figure()
		# plt.rcParams['font.size'] = '11'
		plt.plot(final_x,final_y, label="Mean Travelled Path", lw=2)
		plt.plot(avg_x_plus_3std, avg_y_plus_3std, '--', label=r"$\mu + 3\sigma$")
		plt.plot(avg_x_minus_3std, avg_y_minus_3std, '--', label=r"$\mu - 3\sigma$")
		plt.plot(final_x[0], final_y[0], 'bX', label="Starting Point")
		plt.plot(final_x[len(x)-1], final_y[len(y)-1], 'r*', label="End Point")
		ax = plt.gca()
		ax.add_patch(ply)
		plt.xlim(-45, 40)
		plt.ylim(-30, 25)
		plt.xlabel("x (m)")
		plt.ylabel("y (m)")
		plt.legend(loc='upper center', bbox_to_anchor=(0.5,1.18), fancybox=True, ncol=3, borderpad=0.1)
		# Set tick font size
		# for label in (ax.get_xticklabels() + ax.get_yticklabels()):
		# 	label.set_fontsize(14)

		final_time = [final_time[i] - final_time[0] for i in range(len(final_time))]

		points1 = [[x,y] for x,y in zip(final_time, avg_z_minus_3std)]
		points2 = [[x,y] for x,y in zip(final_time, avg_z_plus_3std)]
		points2.reverse()
		points1 = np.asanyarray(points1)
		points2 = np.asanyarray(points2)
		points = np.append(points1,points2, axis=0)
		ply2 = patches.Polygon(points, alpha=0.2, facecolor='orange')
		
		fig2 = plt.figure(figsize=[5.12, 3.84])
		plt.plot(final_time, final_z)
		plt.plot(final_time, avg_z_plus_3std, '--', label=r"$\mu + 3\sigma$")
		plt.plot(final_time, avg_z_minus_3std, '--', label=r"$\mu - 3\sigma$")
		plt.plot(final_time[0], final_z[0], 'bX', label="Starting Point")
		plt.plot(final_time[len(final_time)-1],final_z[len(final_z)-1], 'r*', label="End Point")
		ax = plt.gca()
		ax.add_patch(ply2)
		# Scenario 1
		# plt.ylim(-3, 22)
		# Scenario 2
		# plt.ylim(-10, 17)
		# Scenario 3
		plt.ylim(2.8, 15)
		plt.xlabel("time (s)")
		plt.ylabel("z (m)")
		plt.legend(loc='upper center', bbox_to_anchor=(0.5,1.18), fancybox=True, ncol=3, borderpad=0.1)

		# Dora scenario 3 - hand picking curves
		# x0 = np.asanyarray(self.x[13][1:]).astype('float')
		# y0 = np.asanyarray(self.y[13][1:]).astype('float')
		# x1 = np.asanyarray(self.x[0][1:]).astype('float')
		# y1 = np.asanyarray(self.y[0][1:]).astype('float')
		# x2 = np.asanyarray(self.x[12][1:]).astype('float')
		# y2 = np.asanyarray(self.y[12][1:]).astype('float')
		# x3 = np.asanyarray(self.x[20][1:]).astype('float')
		# y3 = np.asanyarray(self.y[20][1:]).astype('float')

		# My scenario 3 - hand picking curves
		# x0 = np.asanyarray(self.x[0][1:]).astype('float')
		# y0 = np.asanyarray(self.y[0][1:]).astype('float')
		# x1 = np.asanyarray(self.x[3][1:]).astype('float')
		# y1 = np.asanyarray(self.y[3][1:]).astype('float')
		# x2 = np.asanyarray(self.x[25][1:]).astype('float')
		# y2 = np.asanyarray(self.y[25][1:]).astype('float')
		# x3 = np.asanyarray(self.x[46][1:]).astype('float')
		# y3 = np.asanyarray(self.y[46][1:]).astype('float')

		# Dora Scenario 1 - hand picking
		# x0 = np.asanyarray(self.x[0][1:]).astype('float')
		# y0 = np.asanyarray(self.y[0][1:]).astype('float')
		# x1 = np.asanyarray(self.x[13][1:]).astype('float')
		# y1 = np.asanyarray(self.y[13][1:]).astype('float')
		# x2 = np.asanyarray(self.x[12][1:]).astype('float')
		# y2 = np.asanyarray(self.y[12][1:]).astype('float')
		# x3 = np.asanyarray(self.x[24][1:]).astype('float')
		# y3 = np.asanyarray(self.y[24][1:]).astype('float')

		# My Scenario 1 - hand picking
		# x0 = np.asanyarray(self.x[0][1:]).astype('float')
		# y0 = np.asanyarray(self.y[0][1:]).astype('float')
		# x1 = np.asanyarray(self.x[13][1:]).astype('float')
		# y1 = np.asanyarray(self.y[13][1:]).astype('float')
		# x2 = np.asanyarray(self.x[15][1:]).astype('float')
		# y2 = np.asanyarray(self.y[15][1:]).astype('float')
		# x3 = np.asanyarray(self.x[24][1:]).astype('float')
		# y3 = np.asanyarray(self.y[24][1:]).astype('float')
  
		x0 = np.asanyarray(self.x[0][1:]).astype('float')
		y0 = np.asanyarray(self.y[0][1:]).astype('float')
		x1 = np.asanyarray(self.x[0][1:]).astype('float')
		y1 = np.asanyarray(self.y[0][1:]).astype('float')
		x2 = np.asanyarray(self.x[0][1:]).astype('float')
		y2 = np.asanyarray(self.y[0][1:]).astype('float')
		x3 = np.asanyarray(self.x[0][1:]).astype('float')
		y3 = np.asanyarray(self.y[0][1:]).astype('float')

		# Dora scenario 2 - hand picking
		# x0 = np.asanyarray(self.x[0][1:]).astype('float')
		# y0 = np.asanyarray(self.y[0][1:]).astype('float')
		# x1 = np.asanyarray(self.x[14][1:]).astype('float')
		# y1 = np.asanyarray(self.y[14][1:]).astype('float')
		# x2 = np.asanyarray(self.x[18][1:]).astype('float')
		# y2 = np.asanyarray(self.y[18][1:]).astype('float')
		# x3 = np.asanyarray(self.x[15][1:]).astype('float')
		# y3 = np.asanyarray(self.y[15][1:]).astype('float')

		# My scenario 2 - hand picking
		# x0 = np.asanyarray(self.x[0][1:]).astype('float')
		# y0 = np.asanyarray(self.y[0][1:]).astype('float')
		# x1 = np.asanyarray(self.x[22][1:]).astype('float')
		# y1 = np.asanyarray(self.y[22][1:]).astype('float')
		# x2 = np.asanyarray(self.x[24][1:]).astype('float')
		# y2 = np.asanyarray(self.y[24][1:]).astype('float')
		# x3 = np.asanyarray(self.x[25][1:]).astype('float')
		# y3 = np.asanyarray(self.y[25][1:]).astype('float')

		# xmidhalf = np.asanyarray(self.x[int(len(self.x)/4)][1:]).astype('float')
		# ymidhalf = np.asanyarray(self.y[int(len(self.x)/4)][1:]).astype('float')
		# xhalf = np.asanyarray(self.x[int(len(self.x)/2)][1:]).astype('float')
		# yhalf = np.asanyarray(self.y[int(len(self.x)/2)][1:]).astype('float')
		# xmidend = np.asanyarray(self.x[len(self.x) - int(len(self.x)/4)][1:]).astype('float')
		# ymidend = np.asanyarray(self.y[len(self.x) - int(len(self.x)/4)][1:]).astype('float')
		# xend = np.asanyarray(self.x[len(self.x)-1][1:]).astype('float')
		# yend = np.asanyarray(self.y[len(self.x)-1][1:]).astype('float')

		fig3 = plt.figure(figsize=[5.12, 3.84])
		plt.plot(x0, y0)
		plt.plot(x1, y1)
		plt.plot(x2, y2)
		plt.plot(x3, y3)
		plt.plot(x0[0], y0[0], 'bX', label="Starting Point")
		# scenario 3
		ply3 = patches.Circle([-17.4634,15.6241], 0.5, alpha=0.6, color='k', label="End Zone")
		# scenario 2
		# ply3 = patches.Circle([-13.9796,-20.1261], 0.5, alpha=0.6, color='k', label="End Zone")
		# scenario 1
		# ply3 = patches.Circle([68.7052,-70.2636], 0.5, alpha=0.6, color='k', label="End Zone")
		ax = plt.gca()
		ax.add_patch(ply3)
		plt.xlabel("x (m)")
		plt.ylabel("y (m)")
		plt.xlim(-45, 40)
		plt.ylim(-30, 25)
		plt.legend(loc='upper center', bbox_to_anchor=(0.5,1.1), fancybox=True, ncol=3, borderpad=0.1)
		# plt.plot(xhalf, yhalf)
		# plt.plot(xhalf, yhalf)
		# plt.plot(xmidend, ymidend)


		fig1.savefig('/home/duarte/noetic-sim/src/metrics_extractor/plots/trajectory y(x).pdf')
		# fig2.savefig('/home/afonso/catkin_ws/src/gradient_based_traversability_analysis/plots/trajectory z(t).pdf')
		fig3.savefig('/home/duarte/noetic-sim/src/metrics_extractor/plots/handpicked.pdf')

		plt.show()


	def plots(self):
		
		once=True

		fig1 = plt.figure()
		ax1 = fig1.add_subplot(projection='3d')
		fig2 = plt.figure()
		ax2 = fig2.add_subplot()
		fig3 = plt.figure()
		ax3 = fig3.add_subplot()

		for counter in range(self.number_of_files):
			if counter not in self.fails:
				x = np.asanyarray(self.x[counter][1:]).astype('float')
				y = np.asanyarray(self.y[counter][1:]).astype('float')
				z = np.asanyarray(self.z[counter][1:]).astype('float')
				roll = np.asanyarray(self.roll[counter][1:]).astype('float')
				pitch = np.asanyarray(self.pitch[counter][1:]).astype('float')

				risk = []
				# rand = random.uniform(0, 1)
				# print(rand)
				for i in range(roll.shape[0]):
					if np.abs(np.rad2deg(roll[i])) > 35 or np.abs(np.rad2deg(pitch[i])) > 45:
						risk.append(i)

				time_nsec = np.asanyarray(self.timestamp_nsecs[counter]).astype('float')[1:]
				time_nsec *= 1E-9
				time = np.asanyarray(self.timestamp_secs[counter]).astype('float')[1:]
				time -= time[0]
				time += time_nsec
				self.duration.append(time[len(time)-1])

				p = ax1.scatter(x, y, z, c=time, label='Travelled Path (m)', s=1.0)
				ax1.set_xlabel('x')
				ax1.set_ylabel('y')
				ax1.set_zlabel('z')
				ax1.scatter(x[0], y[0], z[0], marker='X', s=100.0, label="Starting Point (m)")
				ax1.scatter(x[len(x)-1], y[len(y)-1], z[len(z)-1], marker='*', s=100.0, label="End Point (m)")
				# ax1.title.set_text(f"Path took {time[len(time)-1]} seconds to execute")
				# ax1.legend()
				if once:
					cbar=fig1.colorbar(p)
					cbar.set_label("Travel Time")
					ax1.legend()


				ax2.title.set_text("Robot's Trajectory Projected on X-Y plane")
				ax2.scatter(x, y, label='Travelled Path (m)', s=5.0)
				ax2.plot(x[risk], y[risk], 'k.', label="Risky Portion (x,y)")
				ax2.set_xlabel('x')
				ax2.set_ylabel('y')
				ax2.plot(x[0], y[0], 'bX', label="Starting Point (m)")
				ax2.plot(x[len(x)-1], y[len(y)-1], 'r*', label="End Point (m)")
				# ax2.legend()
				if once:
					ax2.legend()

				ax3.title.set_text("Robot's Trajectory - Z function of Time")
				ax3.scatter(time, z, label='Travelled Path (m)', s=5.0)
				ax3.plot(time[risk], z[risk], 'k.', label="Risky Portion (x,y)")
				ax3.set_xlabel('t')
				ax3.set_ylabel('z')
				ax3.plot(time[0], z[0], 'bX', label="Starting Point (m)")
				ax3.plot(time[len(time)-1], z[len(z)-1], 'r*', label="End Point (m)")
				# ax3.legend()
				if once:
					ax3.legend()
					once = False

		# fig1.savefig('/home/afonso/catkin_ws/src/gradient_based_traversability_analysis/plots/trajectory 3D.pdf')
		# fig2.savefig('/home/afonso/catkin_ws/src/gradient_based_traversability_analysis/plots/trajectory y(x).pdf')
		# fig3.savefig('/home/afonso/catkin_ws/src/gradient_based_traversability_analysis/plots/trajectory z(t).pdf')
		plt.show()


	def main(self):

		self.read_logfiles()
		# self.plots()
		self.statistic_curves()


if __name__ == "__main__":

	analyser = MetricsAnalyser()
	analyser.main()
