# Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)

import os
import csv
import numpy as np


# This function computes the Average Packet Size (CPM) for each simulation
def compute_average_cpm_packet_size(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    # Iterate over scenarios
    for i in numberOfUEs:
        for j in redundancy:
            for k in penetrationRate:
                for z in cpm_gen_period:

                    os.chdir(results_path + '/Traces')
                    os.chdir(str(i) + '/' + j + '/' + str(k) + '/' + str(z))

                    # Read and store the trace files
                    files = os.listdir()

                    # Initialize the average packet size
                    avg_size_array = np.array([])

                    # Iterate over UEs trace files
                    for f in files:
                        with open(f, 'r') as csvfile:
                            lines = csv.reader(csvfile, delimiter=',')
                            next(lines)
                            for row in lines:
                                # Filter only CPM
                                if int(row[0]) == 14:
                                    avg_size_array = np.append(avg_size_array, float(row[3]))

                    # Compute the Average Packet Size for this simulation
                    avg_size = avg_size_array.mean()
                    print(f"Average Packet Size (CPM): {avg_size} bytes")

                    # Store the Average EAR for each simulation in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('Average_PacketSizeCPM.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_size])


# This function computes the Average Packet Size (CAM) for each simulation
def compute_average_cam_packet_size(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    # Iterate over scenarios
    for i in numberOfUEs:
        for j in redundancy:
            for k in penetrationRate:
                for z in cpm_gen_period:

                    os.chdir(results_path + '/Traces')
                    os.chdir(str(i) + '/' + j + '/' + str(k) + '/' + str(z))

                    # Read and store the trace files
                    files = os.listdir()

                    # Initialize the average packet size
                    avg_size_array = np.array([])

                    # Iterate over UEs trace files
                    for f in files:
                        with open(f, 'r') as csvfile:
                            lines = csv.reader(csvfile, delimiter=',')
                            next(lines)
                            for row in lines:
                                # Filter only CAM
                                if int(row[0]) == 2:
                                    avg_size_array = np.append(avg_size_array, float(row[3]))

                    # Compute the Average Packet Size for this simulation
                    avg_size = avg_size_array.mean()
                    print(f"Average Packet Size (CAM): {avg_size} bytes")

                    # Store the Average EAR for each simulation in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('Average_PacketSizeCAM.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_size])


# This function computes the average number of Transmitted CPMs/vehicle/second for each simulation
def compute_average_transmitted_cpm_vehicle_second(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period, simTime):

    # Iterate over scenarios
    for i in numberOfUEs:
        for j in redundancy:
            for k in penetrationRate:
                for z in cpm_gen_period:

                    os.chdir(results_path + '/Traces')
                    os.chdir(str(i) + '/' + j + '/' + str(k) + '/' + str(z))

                    # Read and store the trace files
                    files = os.listdir()

                    # Initialize the average transmitted CPMs
                    avg_tx_cpm = 0

                    # Iterate over UEs trace files
                    for f in files:
                        with open(f, 'r') as csvfile:
                            lines = csv.reader(csvfile, delimiter=',')
                            next(lines)
                            for row in lines:
                                # Filter only CPM
                                if int(row[0]) == 14:
                                    avg_tx_cpm += 1

                    # Compute the Average Transmitted CPMs/vehicle/second for this simulation
                    avg_tx_cpm = avg_tx_cpm / len(files) / simTime
                    print(f"Average Transmitted CPMs/vehicle/second: {avg_tx_cpm}")

                    # Store the Average Transmitted CPMs/vehicle/second for each simulation in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('Average_TransmittedCPMs.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_tx_cpm])


# This function computes the Overall number of Transmitted CPMs for each simulation
def compute_transmitted_cpm(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    # Iterate over scenarios
    for i in numberOfUEs:
        for j in redundancy:
            for k in penetrationRate:
                for z in cpm_gen_period:

                    os.chdir(results_path + '/Traces')
                    os.chdir(str(i) + '/' + j + '/' + str(k) + '/' + str(z))

                    # Read and store the trace files
                    files = os.listdir()

                    # Initialize the average transmitted CPMs
                    avg_tx_cpm = 0

                    # Iterate over UEs trace files
                    for f in files:
                        with open(f, 'r') as csvfile:
                            lines = csv.reader(csvfile, delimiter=',')
                            next(lines)
                            for row in lines:
                                # Filter only CPM
                                if int(row[0]) == 14:
                                    avg_tx_cpm += 1

                    print(f"Transmitted CPMs: {avg_tx_cpm}")

                    # Store the Overall Transmitted CPMs for each simulation in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('TransmittedCPMs.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_tx_cpm])


# This function computes the Overall number of Transmitted CAMs for each simulation
def compute_transmitted_cam(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    # Iterate over scenarios
    for i in numberOfUEs:
        for j in redundancy:
            for k in penetrationRate:
                for z in cpm_gen_period:

                    os.chdir(results_path + '/Traces')
                    os.chdir(str(i) + '/' + j + '/' + str(k) + '/' + str(z))

                    # Read and store the trace files
                    files = os.listdir()

                    # Initialize the average transmitted CPMs
                    avg_tx_cam = 0

                    # Iterate over UEs trace files
                    for f in files:
                        with open(f, 'r') as csvfile:
                            lines = csv.reader(csvfile, delimiter=',')
                            next(lines)
                            for row in lines:
                                # Filter only CAM
                                if int(row[0]) == 2:
                                    avg_tx_cam += 1

                    print(f"Transmitted CAMs: {avg_tx_cam}")

                    # Store the Overall Transmitted CAMs for each simulation in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('TransmittedCAMs.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_tx_cam])


# This function clears the CSV files
def clear_csv(results_path):

    os.chdir(results_path)

    for i in os.listdir():
        if os.path.isfile(i):
            with open(i, 'r+') as f:
                f.readline()  # read one line
                f.truncate(f.tell())  # terminate the file here


# Post-processing (insert desired directory with results to be processed)
# results_path = os.path.expanduser(
#     '/home/ready-lab/CLionProjects/ms-van3t/ns-3-dev/Simulation_EXTENDED_2024-06-03 18:11:38.523070/Seed_1')
#
# # Define Toolchain parameters
# simTime = 30                                    # Simulation Time
# s = [50, 75, 100]                               # Simulated scenarios (number of ITS-Ss)
# r = ['ETSI']                                    # Simulated Value of Information computation method
# penetration_rate = [0.2, 0.5, 0.8]                # Market Penetration Rate (MPR)
# T_CpmGen_ms = [100, 1000]                       # CPM Generation Period
#
# # Clear CSV files (clear ONLY Traces CSVs)
# clear_csv(results_path + '/Traces')
# # Compute metrics
# compute_average_cpm_packet_size(results_path, s, r, penetration_rate, T_CpmGen_ms)
# compute_average_cam_packet_size(results_path, s, r, penetration_rate, T_CpmGen_ms)
# compute_average_transmitted_cpm_vehicle_second(results_path, s, r, penetration_rate, T_CpmGen_ms, simTime)
# compute_transmitted_cpm(results_path, s, r, penetration_rate, T_CpmGen_ms)
# compute_transmitted_cam(results_path, s, r, penetration_rate, T_CpmGen_ms)
