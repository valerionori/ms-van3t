# Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)

import os
import shutil


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
    files = os.listdir()
    for i in files:
        os.rename(i, str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' +
                  str(cpm_gen_period) + '/' + i)


def csv_traces_initialization(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    os.chdir(results_path + "/Traces")

    # Generate a new CSV to store the Average Packet Size (CPM) for each simulation
    try:
        file = open("Average_PacketSizeCPM.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Average_CPM_PacketSize_bytes\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Average Packet Size (CAM) for each simulation
    try:
        file = open("Average_PacketSizeCAM.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Average_CAM_PacketSize_bytes\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Average Transmitted CPMs/vehicle/second for each simulation
    try:
        file = open("Average_TransmittedCPMs.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Average_Transmitted_CPMs_vehicle_second\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Transmitted CPMs for each simulation
    try:
        file = open("TransmittedCPMs.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Transmitted_CPMs\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Generate a new CSV to store the Transmitted CAMs for each simulation
    try:
        file = open("TransmittedCAMs.csv", "x")
        file.write("numberOfUEs,RMR,MPR,T_Cpm_Gen,Transmitted_CAMs\n")
        file.close()
    except FileExistsError:
        print("CSV Update.")

    # Move CSV files into proper directory
    os.makedirs(str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' + str(cpm_gen_period))
    files = os.listdir()
    for i in files:
        os.rename(i, str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' +
                  str(cpm_gen_period) + '/' + i)
