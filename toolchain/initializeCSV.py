# Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)

import os
import shutil
import csv
import numpy as np

def csv_paths_initialization(ns3_path, results_path, seed):

    os.chdir(ns3_path)

    # Create 'Results' directory if it does not exist
    os.makedirs("Results", exist_ok=True)

    os.chdir(results_path)

    if seed == 0:
        # Remove the previous CSV files, if they exist
        answer = input("Continue? This will remove any previous CSV file[y/n]")

        if answer.lower() in ["y", "yes"]:
            for filename in os.listdir(results_path):
                file_path = os.path.join(results_path, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)
                except Exception as e:
                    print('Failed to delete %s. Reason: %s' % (file_path, e))
        else:
            exit()

    # Generate the subdirectories for Results
    os.makedirs("Traces")
    os.makedirs("PRR")
    os.makedirs("AoR")
    os.makedirs("EAR")
    os.makedirs("CBR")
    os.makedirs("RDM")


def csv_ear_initialization(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    os.chdir(results_path + "/EAR")

    # Move CSV files into proper directory
    os.makedirs(str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' + str(cpm_gen_period))
    for i in range(1, numberOfUEs + 1):
        filename = 'EAR-veh' + str(i) + '.csv'
        if os.path.exists(filename):
            os.rename(filename, str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' +
                      str(cpm_gen_period) + '/' + filename)


def csv_traces_initialization(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    os.chdir(results_path + "/Traces")

    # Generate a new CSV to store the Average Packet Size for each scenario
    try:
        file = open("Average_PacketSize.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Average CPM Packet Size (bytes)\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Average Transmitted CPMs/vehicle/second for each scenario
    try:
        file = open("Average_TransmittedCPMs.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Average_Transmitted_CPMs_vehicle_second\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Transmitted CPMs for each scenario
    try:
        file = open("TransmittedCPMs.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Transmitted_CPMs\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Transmitted CAMs for each scenario
    try:
        file = open("TransmittedCAMs.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Transmitted_CAMs\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Move CSV files into proper directory
    os.makedirs(str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' + str(cpm_gen_period))
    for i in range(1, numberOfUEs + 1):
        filename = 'Trace-veh' + str(i) + '.csv'
        if os.path.exists(filename):
            os.rename(filename, str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' +
                      str(cpm_gen_period) + '/' + filename)


# This function compute the Average Packet Size (CPM) for each simulation
def compute_average_packet_size(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

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

                    # Compute the Average Packet Size for this scenario
                    avg_size = avg_size_array.mean()
                    print(f"Average Packet Size (CPM): {avg_size} bytes")

                    # Store the Average EAR for each scenario in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('Average_PacketSize.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_size])


# This function compute the number of Transmitted CPMs/vehicle/second for each simulation
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
                    avg_tx_cpm = avg_tx_cpm / len(files) /  simTime
                    print(f"Average Transmitted CPMs/vehicle/second: {avg_tx_cpm}")

                    # Store the Average Transmitted CPMs/vehicle/second for each simulation in a CSV
                    os.chdir(results_path + '/Traces')
                    with open('Average_TransmittedCPMs.csv', 'a', newline='') as csvfile:
                        lines = csv.writer(csvfile)
                        lines.writerow([i, j, k, z, avg_tx_cpm])


# This function compute the Overall number of Transmitted CPMs for each simulation
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


# This function compute the Overall number of Transmitted CAMs for each simulation
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