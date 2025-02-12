#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from statistics import mean, stdev
import csv
import os

class TableGenerator:

	def __init__(self):
		self.timestamp_secs = []
		self.timestamp_nsecs = []
		self.travelled_distance = []
		self.up_effort = []
		self.down_effort = []
		self.duration = []
		self.path_riskiness = []
		self.roll = []
		self.pitch = []
		self.fails = []
		self.file_table = open(f'/home/duarte/noetic-sim/src/metrics_extractor/logfiles/table.tex', mode='w')
		self.files_metrics = []
		self.files_raw = []
		self.dirs = []

		directory = "/home/duarte/noetic-sim/src/metrics_extractor/logfiles/"

		self.number_of_files=0
		self.number_of_experiments=0

		exclude = set(['misc'])
		for root,dirs,files in os.walk(directory):
			dirs[:] = [d for d in dirs if d not in exclude]
			dirs.sort()
			if len(dirs) != 0:
				self.number_of_experiments = len(dirs)
				self.dirs = dirs
			for file in files:
				if file.startswith("logfile_metrics"):
					self.files_metrics.append(open(root+'/'+file, 'r'))
					self.number_of_files += 1
				elif file.startswith("logfile_raw"):
					self.files_raw.append(open(root+'/'+file, 'r'))
					

	def read_logfiles(self):
		for counter in range(self.number_of_files):
			# print(self.number_of_files)
			data = []
			csv_reader = csv.reader(self.files_raw[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					data.append(row[i])
				line_count += 1

			aux = 0
			secs = []
			nsecs = []
			roll = []
			pitch = []
			for elem in data:

				if(aux == 8):
					aux = 0

				if aux == 0:
					pass
				elif aux == 1:
					pass
				elif aux == 2:
					pass
				elif aux == 3:
					secs.append(elem)
				elif aux == 4:
					nsecs.append(elem)
				elif aux == 5:
					roll.append(elem)
				elif aux == 6:
					pitch.append(elem)
				else:
					pass
				aux += 1

			self.timestamp_secs.append(secs)
			self.timestamp_nsecs.append(nsecs)
			self.roll.append(roll)
			self.pitch.append(pitch)

			data = []
			csv_reader = csv.reader(self.files_metrics[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					if row[i] == "ABORTED":
						self.fails.append(counter)
					data.append(row[i])
				line_count += 1

			aux = 0
			travelled_distance = []
			up_effort = []
			down_effort = []
			path_riskiness = []
			for elem in data[:-1]:

				if(aux == 4):
					aux = 0

				if aux == 0:
					travelled_distance.append(elem)
				elif aux == 1:
					up_effort.append(elem)
				elif aux == 2:
					down_effort.append(elem)
				else:
					path_riskiness.append(elem)
				aux += 1

			self.travelled_distance.append(travelled_distance)
			self.up_effort.append(up_effort)
			self.down_effort.append(down_effort)
			
			path_riskiness = int(path_riskiness[-1])/len(path_riskiness)*100

			self.path_riskiness.append(path_riskiness)

			time_nsec = np.asanyarray(self.timestamp_nsecs[counter]).astype('float')[1:]
			time_nsec *= 1E-9
			time = np.asanyarray(self.timestamp_secs[counter]).astype('float')[1:]
			time -= time[0]
			time += time_nsec
			self.duration.append(time[len(time)-1])


	def table_generator(self):

		travelled_distance = []
		up_effort = []
		down_effort = []
		path_riskiness = []
		all_travelled_distance = []
		all_up_effort = []
		all_down_effort = []
		all_path_riskiness = []
		duration = []
		all_duration = []
		failure_rate = []

		begin_tabular = []
		line1 = []
		line2 = []
		line3 = []
		line4 = []
		line5 = []
		line6 = []
		line7 = []
		end_tabular = []

		runs = int(self.number_of_files/self.number_of_experiments)

		columns = "l"
		for j in range(self.number_of_experiments):
			columns += "r"

		begin_tabular.append(f"\\begin{{tabular}}{{@{{}}{columns}@{{}}}}\n")
		line1.append(f"\\toprule Metrics")
		line2.append(f"Travelled Distance (m)")
		line3.append(f"Up Effort (m)")
		line4.append(f"Down Effort (m)")
		line5.append(f"Elapsed Time (s)")
		line6.append(f"Path Riskiness Index (\\%)")
		line7.append(f"Failure Rate (\\%)")
		end_tabular.append(f"\\end{{tabular}}")

		iterator = 0
		for counter in range(self.number_of_experiments):
			for i in range(runs):
				if iterator not in self.fails:
					travelled_distance.append(float(np.asanyarray(self.travelled_distance[iterator]).astype('float')[-1:]))
					up_effort.append(float(np.asanyarray(self.up_effort[iterator]).astype('float')[-1:]))
					down_effort.append(float(np.asanyarray(self.down_effort[iterator]).astype('float')[-1:]))
					path_riskiness.append(self.path_riskiness[iterator])
					duration.append(self.duration[iterator])
					iterator += 1
				else:
					iterator +=1 
					continue
			all_travelled_distance.append(travelled_distance)
			all_up_effort.append(up_effort)
			all_down_effort.append(down_effort)
			all_path_riskiness.append(path_riskiness)
			all_duration.append(duration)
			travelled_distance = []
			up_effort = []
			down_effort = []
			path_riskiness = []
			duration = []

		aux = 0
		for iter in range(self.number_of_experiments):
			for elem in self.fails:
				if elem >= iter*runs and elem < (iter+1)*runs:
					aux += 1
			failure_rate.append(aux)
			aux = 0

		for counter in range(self.number_of_experiments):
			mean_travelled_distance = mean(all_travelled_distance[counter])
			stdev_travelled_distance = stdev(all_travelled_distance[counter])
			mean_up_effort = mean(all_up_effort[counter])
			stdev_up_effort = stdev(all_up_effort[counter])
			mean_down_effort = mean(all_down_effort[counter])
			stdev_down_effort = stdev(all_down_effort[counter])
			mean_duration = mean(all_duration[counter])
			stdev_duration = stdev(all_duration[counter])
			mean_risk = mean(all_path_riskiness[counter])
			stdev_risk = stdev(all_path_riskiness[counter])
			failure_rate_percentage = failure_rate[counter]/runs*100


			line1.append(f" & \\thead{{{self.dirs[counter]}}}")
			line2.append(f" & ${mean_travelled_distance:.3f} \\pm {stdev_travelled_distance:.3f}$")
			line3.append(f" & ${mean_up_effort:.3f} \\pm {stdev_up_effort:.3f}$")
			line4.append(f" & ${mean_down_effort:.3f} \\pm {stdev_down_effort:.3f}$")
			line5.append(f" & ${mean_duration:.3f} \\pm {stdev_duration:.3f}$")
			line6.append(f" & ${mean_risk:.3f} \\pm {stdev_risk:.3f}$")
			line7.append(f" & ${failure_rate_percentage:.3f}$")


		line1.append(" \\\\ \\midrule\n")
		line2.append(" \\\\\n")
		line3.append(" \\\\\n")
		line4.append(" \\\\\n")
		line5.append(" \\\\\n")
		line6.append(" \\\\\n")
		line7.append(" \\\\ \\bottomrule\n")

		self.file_table.writelines(begin_tabular)
		self.file_table.writelines(line1)
		self.file_table.writelines(line2)
		self.file_table.writelines(line3)
		self.file_table.writelines(line4)
		self.file_table.writelines(line5)
		self.file_table.writelines(line6)
		self.file_table.writelines(line7)
		self.file_table.writelines(end_tabular)

	# def post_processing_effort(self):
	# 	aux_mean_ = []
	# 	for counter in range(self.number_of_files):
	# 		if counter not in self.fails:
	# 			pitch = np.abs(np.asanyarray(self.pitch[counter][1:]).astype('float'))
	# 			aux_mean_.append(np.mean(pitch))
	# 	mean_ = np.rad2deg(np.mean(aux_mean_))
	# 	stddev = np.rad2deg(np.std(aux_mean_))
	# 	print(f"{mean_=:0.3f} and {stddev=:0.3f}")


	# def post_processing_risks(self):
		
	# 	sigmoid_pitch = lambda x: 1 / (1 + np.exp(-0.25*x + 6))
	# 	sigmoid_roll = lambda x: 1 / (1 + np.exp(-0.20*x + 3))
		
	# 	aux_roll = []
	# 	aux_pitch = []
	# 	for counter in range(self.number_of_files):
	# 		if counter not in self.fails:
	# 			roll = np.rad2deg(np.abs(np.#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from statistics import mean, stdev
import csv
import os

class TableGenerator:

	def __init__(self):
		self.timestamp_secs = []
		self.timestamp_nsecs = []
		self.travelled_distance = []
		self.up_effort = []
		self.down_effort = []
		self.duration = []
		self.path_riskiness = []
		self.roll = []
		self.pitch = []
		self.fails = []
		self.file_table = open(f'/home/duarte/noetic-sim/src/metrics_extractor/logfiles/table.tex', mode='w')
		self.files_metrics = []
		self.files_raw = []
		self.dirs = []

		directory = "/home/duarte/noetic-sim/src/metrics_extractor/logfiles/"

		self.number_of_files=0
		self.number_of_experiments=0

		exclude = set(['misc'])
		for root,dirs,files in os.walk(directory):
			dirs[:] = [d for d in dirs if d not in exclude]
			dirs.sort()
			if len(dirs) != 0:
				self.number_of_experiments = len(dirs)
				self.dirs = dirs
			for file in files:
				if file.startswith("logfile_metrics"):
					self.files_metrics.append(open(root+'/'+file, 'r'))
					self.number_of_files += 1
				elif file.startswith("logfile_raw"):
					self.files_raw.append(open(root+'/'+file, 'r'))
					

	def read_logfiles(self):
		for counter in range(self.number_of_files):
			# print(self.number_of_files)
			data = []
			csv_reader = csv.reader(self.files_raw[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					data.append(row[i])
				line_count += 1

			aux = 0
			secs = []
			nsecs = []
			roll = []
			pitch = []
			for elem in data:

				if(aux == 8):
					aux = 0

				if aux == 0:
					pass
				elif aux == 1:
					pass
				elif aux == 2:
					pass
				elif aux == 3:
					secs.append(elem)
				elif aux == 4:
					nsecs.append(elem)
				elif aux == 5:
					roll.append(elem)
				elif aux == 6:
					pitch.append(elem)
				else:
					pass
				aux += 1

			self.timestamp_secs.append(secs)
			self.timestamp_nsecs.append(nsecs)
			self.roll.append(roll)
			self.pitch.append(pitch)

			data = []
			csv_reader = csv.reader(self.files_metrics[counter], delimiter=',')
			line_count = 0
			for row in csv_reader:
				for i in range(len(row)):
					if row[i] == "ABORTED":
						self.fails.append(counter)
					data.append(row[i])
				line_count += 1

			aux = 0
			travelled_distance = []
			up_effort = []
			down_effort = []
			path_riskiness = []
			for elem in data[:-1]:

				if(aux == 4):
					aux = 0

				if aux == 0:
					travelled_distance.append(elem)
				elif aux == 1:
					up_effort.append(elem)
				elif aux == 2:
					down_effort.append(elem)
				else:
					path_riskiness.append(elem)
				aux += 1

			self.travelled_distance.append(travelled_distance)
			self.up_effort.append(up_effort)
			self.down_effort.append(down_effort)
			
			path_riskiness = int(path_riskiness[-1])/len(path_riskiness)*100

			self.path_riskiness.append(path_riskiness)

			time_nsec = np.asanyarray(self.timestamp_nsecs[counter]).astype('float')[1:]
			time_nsec *= 1E-9
			time = np.asanyarray(self.timestamp_secs[counter]).astype('float')[1:]
			time -= time[0]
			time += time_nsec
			self.duration.append(time[len(time)-1])


	def table_generator(self):

		travelled_distance = []
		up_effort = []
		down_effort = []
		path_riskiness = []
		all_travelled_distance = []
		all_up_effort = []
		all_down_effort = []
		all_path_riskiness = []
		duration = []
		all_duration = []
		failure_rate = []

		begin_tabular = []
		line1 = []
		line2 = []
		line3 = []
		line4 = []
		line5 = []
		line6 = []
		line7 = []
		end_tabular = []

		runs = int(self.number_of_files/self.number_of_experiments)

		columns = "l"
		for j in range(self.number_of_experiments):
			columns += "r"

		begin_tabular.append(f"\\begin{{tabular}}{{@{{}}{columns}@{{}}}}\n")
		line1.append(f"\\toprule Metrics")
		line2.append(f"Travelled Distance (m)")
		line3.append(f"Up Effort (m)")
		line4.append(f"Down Effort (m)")
		line5.append(f"Elapsed Time (s)")
		line6.append(f"Path Riskiness Index (\\%)")
		line7.append(f"Failure Rate (\\%)")
		end_tabular.append(f"\\end{{tabular}}")

		iterator = 0
		for counter in range(self.number_of_experiments):
			for i in range(runs):
				if iterator not in self.fails:
					travelled_distance.append(float(np.asanyarray(self.travelled_distance[iterator]).astype('float')[-1:]))
					up_effort.append(float(np.asanyarray(self.up_effort[iterator]).astype('float')[-1:]))
					down_effort.append(float(np.asanyarray(self.down_effort[iterator]).astype('float')[-1:]))
					path_riskiness.append(self.path_riskiness[iterator])
					duration.append(self.duration[iterator])
					iterator += 1
				else:
					iterator +=1 
					continue
			all_travelled_distance.append(travelled_distance)
			all_up_effort.append(up_effort)
			all_down_effort.append(down_effort)
			all_path_riskiness.append(path_riskiness)
			all_duration.append(duration)
			travelled_distance = []
			up_effort = []
			down_effort = []
			path_riskiness = []
			duration = []

		aux = 0
		for iter in range(self.number_of_experiments):
			for elem in self.fails:
				if elem >= iter*runs and elem < (iter+1)*runs:
					aux += 1
			failure_rate.append(aux)
			aux = 0

		for counter in range(self.number_of_experiments):
			mean_travelled_distance = mean(all_travelled_distance[counter])
			stdev_travelled_distance = stdev(all_travelled_distance[counter])
			mean_up_effort = mean(all_up_effort[counter])
			stdev_up_effort = stdev(all_up_effort[counter])
			mean_down_effort = mean(all_down_effort[counter])
			stdev_down_effort = stdev(all_down_effort[counter])
			mean_duration = mean(all_duration[counter])
			stdev_duration = stdev(all_duration[counter])
			mean_risk = mean(all_path_riskiness[counter])
			stdev_risk = stdev(all_path_riskiness[counter])
			failure_rate_percentage = failure_rate[counter]/runs*100


			line1.append(f" & \\thead{{{self.dirs[counter]}}}")
			line2.append(f" & ${mean_travelled_distance:.3f} \\pm {stdev_travelled_distance:.3f}$")
			line3.append(f" & ${mean_up_effort:.3f} \\pm {stdev_up_effort:.3f}$")
			line4.append(f" & ${mean_down_effort:.3f} \\pm {stdev_down_effort:.3f}$")
			line5.append(f" & ${mean_duration:.3f} \\pm {stdev_duration:.3f}$")
			line6.append(f" & ${mean_risk:.3f} \\pm {stdev_risk:.3f}$")
			line7.append(f" & ${failure_rate_percentage:.3f}$")


		line1.append(" \\\\ \\midrule\n")
		line2.append(" \\\\\n")
		line3.append(" \\\\\n")
		line4.append(" \\\\\n")
		line5.append(" \\\\\n")
		line6.append(" \\\\\n")
		line7.append(" \\\\ \\bottomrule\n")

		self.file_table.writelines(begin_tabular)
		self.file_table.writelines(line1)
		self.file_table.writelines(line2)
		self.file_table.writelines(line3)
		self.file_table.writelines(line4)
		self.file_table.writelines(line5)
		self.file_table.writelines(line6)
		self.file_table.writelines(line7)
		self.file_table.writelines(end_tabular)

	# def post_processing_effort(self):
	# 	aux_mean_ = []
	# 	for counter in range(self.number_of_files):
	# 		if counter not in self.fails:
	# 			pitch = np.abs(np.asanyarray(self.pitch[counter][1:]).astype('float'))
	# 			aux_mean_.append(np.mean(pitch))
	# 	mean_ = np.rad2deg(np.mean(aux_mean_))
	# 	stddev = np.rad2deg(np.std(aux_mean_))
	# 	print(f"{mean_=:0.3f} and {stddev=:0.3f}")


	# def post_processing_risks(self):
		
	# 	sigmoid_pitch = lambda x: 1 / (1 + np.exp(-0.25*x + 6))
	# 	sigmoid_roll = lambda x: 1 / (1 + np.exp(-0.20*x + 3))
		
	# 	aux_roll = []
	# 	aux_pitch = []
	# 	for counter in range(self.number_of_files):
	# 		if counter not in self.fails:
	# 			roll = np.rad2deg(np.abs(np.asanyarray(self.roll[counter][1:]).astype('float')))
	# 			pitch = np.rad2deg(np.abs(np.asanyarray(self.pitch[counter][1:]).astype('float')))
	# 			aux_roll.append(np.mean(sigmoid_roll(roll)))
	# 			aux_pitch.append(np.mean(sigmoid_pitch(pitch)))

	# 	mean_roll_danger = np.mean(aux_roll)
	# 	stddev_roll_danger = np.std(aux_roll)
	# 	mean_pitch_danger = np.mean(aux_pitch)
	# 	stddev_pitch_danger = np.std(aux_pitch)
	# 	print(f"{mean_roll_danger=:0.3f} and {stddev_roll_danger=:0.3f} and {mean_pitch_danger=:0.3f} and {stddev_pitch_danger=:0.3f}")	


	def main(self):

		self.read_logfiles()
		# self.post_processing_effort()
		# self.post_processing_risks()
		self.table_generator()
			




if __name__ == "__main__":

	table_generator = TableGenerator()
	table_generator.main()

	# 	mean_roll_danger = np.mean(aux_roll)
	# 	stddev_roll_danger = np.std(aux_roll)
	# 	mean_pitch_danger = np.mean(aux_pitch)
	# 	stddev_pitch_danger = np.std(aux_pitch)
	# 	print(f"{mean_roll_danger=:0.3f} and {stddev_roll_danger=:0.3f} and {mean_pitch_danger=:0.3f} and {stddev_pitch_danger=:0.3f}")	


	def main(self):

		self.read_logfiles()
		# self.post_processing_effort()
		# self.post_processing_risks()
		self.table_generator()
			




if __name__ == "__main__":

	table_generator = TableGenerator()
	table_generator.main()
