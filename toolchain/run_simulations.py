# Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)

"""
This script will run the "v2v-cooperativePerception-nrv2x" example
with different parameters.

Different scenarios may be evaluated, depending on the number of UEs to be included
and on the Redundancy Mitigation Rule to be applied.

Results are stored in CSV files inside 'ms-van3t/ns-3-dev/Results/'
Plots are stored in inside 'ms-van3t/ns-3-dev/Plots/'
"""

import os
import initializeCSV
import vehicleFlow
from datetime import datetime


def run_simulation(ns3_path, results_path, maps_path, simTime,
                   AoR, channel_bandwidth, penetration_rate, T_CpmGen_ms,
                   scenarios, redundancy, seed, seed_counter):

    application = "v2v-extendedCPM-nrv2x"
    counter = 0

    for i in scenarios:

        # Generate a rou.xml file with a specific number of vehicles
        vehicleFlow.fixedFlow(i, maps_path)

        for j in redundancy:
            for k in penetration_rate:
                for z in T_CpmGen_ms:

                    # Set Simulation Parameters
                    parameters = (
                        f"{application} "
                        f"--simTime={simTime} "
                        f"--AreaOfRelevance={AoR} "
                        f"--VoIComputationMethod={j} "
                        f"--bandwidthBandSl={channel_bandwidth*10} "
                        f"--penetrationRate={k} "
                        f"--CPMGenerationPeriod={z} "
                        f"--sumo-gui={False} "
                        f"--SUMOSeed={seed}"
                    )

                    # Update Simulation Number
                    counter = counter + 1
                    numberOfSimulations = len(scenarios)*len(penetration_rate)*len(T_CpmGen_ms)*len(redundancy)
                    print(f'Simulation n. {counter}/{numberOfSimulations}, Seed: {seed_counter + 1}/5')

                    # Run the Simulation
                    os.chdir(ns3_path)
                    os.system('./ns3 run ' + '"' + parameters + '"')

                    # Initialize the CSV files
                    initializeCSV.csv_ear_initialization(results_path, i, j, k, z)
                    initializeCSV.csv_traces_initialization(results_path, i, j, k, z)


# Define Toolchain parameters
simTime = 30                                    # Simulation Time
AoR = 250                                       # EAR Measurements Start Time
s = [50, 75, 100]                               # Simulated scenarios (number of ITS-Ss)
r = ['ETSI']                                    # Simulated Value of Information computation method
channel_bandwidth = 10                          # SL Channel Bandwidth in MHz
penetration_rate = [0.2, 0.5, 0.8]                # Market Penetration Rate (MPR)
T_CpmGen_ms = [100, 1000]                       # CPM Generation Period
seeds = [2785, 7277]          # Simulations seed

# Take the Start Time of the Toolchain
toolchain_start = datetime.now()

# Define paths
ns3_path = os.path.expanduser('~/CLionProjects/ms-van3t/ns-3-dev/')
results_path = os.path.expanduser('~/CLionProjects/ms-van3t/ns-3-dev/Results/')
maps_path = os.path.expanduser(
    '~/CLionProjects/ms-van3t/ns-3-dev/src/automotive/examples/sumo_files_v2v_map_congestion/')

# Run simulations for each parameter combination
seed_counter = 0
for i in seeds:

    # Initialize all CSV files
    initializeCSV.csv_paths_initialization(ns3_path, results_path, seed_counter)

    # Run the simulations for different combination of parameters
    run_simulation(ns3_path, results_path, maps_path, simTime, AoR,
                   channel_bandwidth, penetration_rate, T_CpmGen_ms, s, r, i, seed_counter)

    # Compute metrics
    initializeCSV.compute_average_packet_size(results_path, s, r, penetration_rate, T_CpmGen_ms)
    initializeCSV.compute_average_transmitted_cpm_vehicle_second(results_path, s, r, penetration_rate, T_CpmGen_ms, simTime)
    initializeCSV.compute_transmitted_cpm(results_path, s, r, penetration_rate, T_CpmGen_ms)
    initializeCSV.compute_transmitted_cam(results_path, s, r, penetration_rate, T_CpmGen_ms)

    os.chdir(ns3_path)
    os.makedirs("Simulation_EXTENDED_" + str(toolchain_start), exist_ok=True)
    os.rename("Results/", "Simulation_EXTENDED_" + str(toolchain_start) + "/Seed_" + str(seed_counter) + "/")
    seed_counter += 1

# Take the End Time of the Toolchain
toolchain_end = datetime.now()

# Show simulation time overall
print('Simulation Started on day ' + toolchain_start.strftime("%d-%m-%Y at %H:%M:%S"))
print('Simulation Ended on day ' + toolchain_end.strftime("%d-%m-%Y at %H:%M:%S"))

print('Results generated in ' + ns3_path)
