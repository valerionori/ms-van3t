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
                   scenarios, redundancy, seed):

    application = "v2v-CPM+CAM-nrv2x"
    counter = 0

    for i in scenarios:

        # Generate a rou.xml file with a specific number of vehicles
        vehicleFlow.randomFlow(i, maps_path)

        for j in redundancy:
            for k in penetration_rate:
                for z in T_CpmGen_ms:

                    # Set Simulation Parameters
                    parameters = (
                        f"{application} "
                        f"--simTime={simTime} "
                        f"--AreaOfRelevance={AoR} "
                        f"--RedundancyMitigation={j} "
                        f"--bandwidthBandSl={channel_bandwidth*10} "
                        f"--penetrationRate={k} "
                        f"--CPMGenerationPeriod={z} "
                        f"--sumo-gui={True}"
                    )

                    # Update Simulation Number
                    counter = counter + 1
                    numberOfSimulations = len(scenarios)*len(penetration_rate)*len(T_CpmGen_ms)*len(redundancy)
                    print('Simulation n. ' + str(counter) + '/' + str(numberOfSimulations) + ', Seed:' + str(seed))

                    # Run the Simulation
                    os.chdir(ns3_path)
                    os.system('./ns3 run ' + '"' + parameters + '"')

                    # Initialize the CSV files
                    initializeCSV.csv_ear_initialization(results_path, i, j, k, z)
                    # initializeCSV.csv_cbr_initialization(results_path, i, j, k, z)
                    initializeCSV.csv_packet_size_initialization(results_path, i, j, k, z)


# Define Toolchain parameters
simTime = 10                                    # Simulation Time
AoR = 250                                       # EAR Measurements Start Time
s = [10, 12]                               # Simulated scenarios (number of ITS-Ss)
r = [True]                                      # Simulated Redundancy Mitigation Rules
channel_bandwidth = 10                          # SL Channel Bandwidth in MHz
penetration_rate = [0.5, 1]                # Market Penetration Rate (MPR)
T_CpmGen_ms = [100, 1000]                       # CPM Generation Period
seeds = 1                                       # number of simulations for each point

# Take the Start Time of the Toolchain
toolchain_start = datetime.now()

# Define paths
ns3_path = os.path.expanduser('~/CLionProjects/ms-van3t/ns-3-dev/')
results_path = os.path.expanduser('~/CLionProjects/ms-van3t/ns-3-dev/Results/')
maps_path = os.path.expanduser(
    '~/CLionProjects/ms-van3t/ns-3-dev/src/automotive/examples/sumo_files_v2v_map_congestion/')

# Define Plots path
plot_dir = toolchain_start.strftime("%d-%m-%Y_%H:%M:%S")
plots_path = os.path.expanduser('~/CLionProjects/ms-van3t/ns-3-dev/Plots/' + plot_dir + '/')


# Run simulations for each parameter combination
for i in range(seeds):

    # Initialize all CSV files
    initializeCSV.csv_paths_initialization(ns3_path, results_path, i)

    run_simulation(ns3_path, results_path, maps_path, simTime, AoR,
                   channel_bandwidth, penetration_rate, T_CpmGen_ms, s, r, i)


    os.chdir(ns3_path)
    os.rename("Results/", "Seed_" + str(i) + "/")

# Take the End Time of the Toolchain
toolchain_end = datetime.now()

# Show simulation time overall
print('Simulation Started on day ' + toolchain_start.strftime("%d-%m-%Y at %H:%M:%S"))
print('Simulation Ended on day ' + toolchain_end.strftime("%d-%m-%Y at %H:%M:%S"))


print('Results generated in ' + results_path)
print('Plots generated in ' + plots_path)
