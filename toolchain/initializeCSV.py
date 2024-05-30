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
    for i in range(1, numberOfUEs + 1):
        filename = 'EAR-veh' + str(i) + '.csv'
        if os.path.exists(filename):
            os.rename(filename, str(numberOfUEs) + '/' + str(redundancy) + '/' + str(penetrationRate) + '/' +
                      str(cpm_gen_period) + '/' + filename)


def csv_packet_size_initialization(results_path, numberOfUEs, redundancy, penetrationRate, cpm_gen_period):

    os.chdir(results_path + "/Traces")

    # Generate a new CSV to store the Average for each scenario
    try:
        file = open("Average_PacketSize.csv", "x")
        file.write("Scenario,Redundancy Mitigation,AveragePacketSize (bytes)\n")
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
