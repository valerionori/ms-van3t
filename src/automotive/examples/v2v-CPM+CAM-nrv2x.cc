/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"


#include <unistd.h>
#include "ns3/core-module.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("v2v-nrv2x");

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
void
GetSlBitmapFromString (const std::string& slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
          {
                  { "0", 0 },
                  { "1", 1 },
          };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
  {
    extracted.push_back (token);
  }

  for (const auto & v : extracted)
  {
    if (lookupTable.find (v) == lookupTable.end ())
    {
      NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
    }
    slBitMapVector.emplace_back(lookupTable[v] & 0x01);
  }
}

/**
 * @VALERIO
 *
 * @brief This function detects the number of connected vehicles out of the total number of vehicles in the simulation.
 *
 * @param sumoClient  the pointer to the TraCI client
 */
std::vector<std::string> numberOfUEsEnabled;
void
detectConnectedVehicles(Ptr<TraciClient> sumoClient)
{
  std::vector<std::string> numberOfUEs;

  numberOfUEs = sumoClient->vehicle.getIDList(); // Get all vehicle IDs

  // Define the same color of connected UEs
  libsumo::TraCIColor connected;
  connected.r=0;connected.g=225;connected.b=255;connected.a=255;

  // Fill the vector with connected UEs only
  for (const auto& j:numberOfUEs)
  {
    auto color = sumoClient->TraCIAPI::vehicle.getColor(j);
    if (color.r == connected.r && color.g == connected.g && color.b == connected.b && color.a == connected.a)
      numberOfUEsEnabled.push_back(j);
  }

  std::cout << "Detected " << numberOfUEsEnabled.size() << " connected vehicles out of " << numberOfUEs.size() << std::endl;
}

/**
 * @VALERIO
 *
 * @brief This function schedules the measurement of the Environmental Awareness Ratio (EAR).
 *
 * @param AoR_radius the radius of the Area od Relevance (AoR) in meters
 * @param sumoClient the pointer to the TraCI client
 *
 * EAR is computed as the ratio between 'known' objects and actual objects within the AoR of the egoVehicle.
 *      - 'known' objects are retrieved from a CSV file which is updated every 100 ms with every object in the LDM
 *         of the egoVehicle within its AoR;
 *      - actual objects within the AoR of the egoVehicle are retrieved from TraCI API.
 *
 * The two vectors are compared to see how many Objects do match (matching_object).
 */
std::vector<double> Average_EAR_Overall = {};
void
computeAwarenessRatio (Ptr<TraciClient> sumoClient, double AoR_radius)
{
  std::vector<std::string> AoR;     // Vector to store current Objects within the AoR

  for(const auto& i:numberOfUEsEnabled)
  {
    /* Compute the position of the egoVehicle */
    libsumo::TraCIPosition egoPosXY = sumoClient->TraCIAPI::vehicle.getPosition(i);

    /* Get all IDs in the simulation and compare their distance from the egoVehicle*/
    std::vector<std::string> allIDs;
    allIDs = sumoClient->vehicle.getIDList ();  // Get all IDs in the simulation
    allIDs.erase(std::remove(allIDs.begin(), allIDs.end(), i), allIDs.end()); // Exclude this UE

    /* Clear AoR vector */
    AoR.clear();

    for(const auto & allID : allIDs)
    {
      // Compute the Object position
      libsumo::TraCIPosition geoPosXY = sumoClient->TraCIAPI::vehicle.getPosition(allID);

      // Check if the Object is within the AoR
      if(sqrt(pow((egoPosXY.x-geoPosXY.x),2) + pow((egoPosXY.y-geoPosXY.y),2)) <= AoR_radius)
        AoR.insert(AoR.end(), allID);  // Add the Object to the vector
    }


    /* Read the CSV file with the "known" Objects within the AoR and save the last result into a vector */
    std::vector<std::string> AoR_K;

    std::string aor_csv_name = "Results/AoR/AoR-" + i + ".csv";
    std::ifstream aor_csv(aor_csv_name);
    std::string line;

    while (std::getline(aor_csv, line)) {
      std::stringstream ss(line);
      std::string value;

      while (std::getline(ss, value, ',')) {
        AoR_K.push_back(value);
      }
    }

    /* Compare the two arrays on the terminal */
//    std::sort(AoR.begin (), AoR.end ());
//    std::sort(AoR_K.begin (), AoR_K.end ());
//
//    std::cout << i << " AoR Evaluation:" << std::endl;
//
//    std::cout << "TRACI: ";
//
//    for (const auto& element : AoR) {
//      std::cout << element << " ";
//    }
//
//    std::cout << std::endl;
//
//    std::cout << "KNOWN: ";
//
//    for (const auto& element_known : AoR_K) {
//      std::cout << element_known << " ";
//    }
//
//    std::cout << std::endl;

    /* Initialize the variables */
    double EAR;
    double matching_objects = 0;

    /* Compare the two vectors */
    for (const auto& element : AoR) {
      for (const auto& element_known : AoR_K) {
        if(element == element_known)
          matching_objects++;
      }
    }

    //std::cout << i << " -> matching objects: " << matching_objects << std::endl;

    /* Compute the Awareness Ratio */
    if(AoR.empty() && AoR_K.empty())
      EAR = 1;
    else if(AoR.size () >= AoR_K.size())
      EAR = matching_objects/double(AoR.size ());
    else
      EAR = matching_objects/double(AoR_K.size ());

    //std::cout << i << "-> EAR: " << EAR << std::endl;

    /* Store computed EAR in a CSV file */
    std::string ear_csv_name = "Results/EAR/EAR-" + i + ".csv";
    std::ofstream ear_csv;
    ear_csv.open(ear_csv_name, std::ios::app);
    ear_csv << Simulator::Now().GetSeconds() << ","
            << EAR << std::endl;

    /* Add the computed EAR to the vector for Average computation */
    Average_EAR_Overall.push_back(EAR);
  }

  // Schedule the next measurement
  Simulator::Schedule(Seconds(0.2), &computeAwarenessRatio, sumoClient, AoR_radius);
}


int
main (int argc, char *argv[])
{

  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map_congestion/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map_congestion/map.sumo.cfg";

  /*** 0.a App Options ***/
  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  bool vehicle_vis = false;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  double penetrationRate = 0.7;

  xmlDocPtr rou_xml_file;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = true;


  // Simulation parameters.
  double simTime = 100.0;
  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (2.0);

  // @VALERIO -> Area of Relevance (AoR) radius
  double AoR_radius = 250;

  /**
   * @VALERIO
   *
   * VoIcomputationMethod:  Value of Information computation methods as defined in ETSI TS 103 324 V2.1.1 (2023-06) Annex E.
   * Values:
   *    - "ETSI" (ETSI TS 103 324 V2.1.1 Section 6.1.2.3, default)
   *    - "frequency-based"
   *    - "dynamics-based"
   *    - "distance-based"
   *    - "object self-announcement"
   */
  bool redundancyMitigation = true; // ObjectInclusionConfig as defined in ETSI TS 103 324 V2.1.1 (2023-06)
  std::string VoIcomputationMethod = "ETSI";

  int16_t T_GenCpm = 100; // @VALERIO, @MATTIA -> set the CPM Generation Period to be applied

  uint16_t sumo_seed = 10; // @VALERIO, @MATTIA -> seed for TraCI client


  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  uint16_t bandwidthBandSl = 400;
  double txPower = 23; //dBm
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

  /*
   * From here, we instruct the ns3::CommandLine class of all the input parameters
   * that we may accept as input, as well as their description, and the storage
   * variable.
   */
  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("penetrationRate", "Rate of vehicles equipped with wireless communication devices", penetrationRate);

  cmd.AddValue ("simTime",
                "Simulation time in seconds",
                simTime);
  cmd.AddValue ("slBearerActivationTime",
                "Sidelik bearer activation time in seconds",
                slBearersActivationTime);
  cmd.AddValue ("centralFrequencyBandSl",
                "The central frequency to be used for Sidelink band/channel",
                centralFrequencyBandSl);
  cmd.AddValue ("bandwidthBandSl",
                "The system bandwidth to be used for Sidelink",
                bandwidthBandSl);
  cmd.AddValue ("txPower",
                "total tx power in dBm",
                txPower);
  cmd.AddValue ("tddPattern",
                "The TDD pattern string",
                tddPattern);
  cmd.AddValue ("slBitMap",
                "The Sidelink bitmap string",
                slBitMap);
  cmd.AddValue ("numerologyBwpSl",
                "The numerology to be used in Sidelink bandwidth part",
                numerologyBwpSl);
  cmd.AddValue ("slSensingWindow",
                "The Sidelink sensing window length in ms",
                slSensingWindow);
  cmd.AddValue ("slSelectionWindow",
                "The parameter which decides the minimum Sidelink selection "
                "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                slSelectionWindow);
  cmd.AddValue ("slSubchannelSize",
                "The Sidelink subchannel size in RBs",
                slSubchannelSize);
  cmd.AddValue ("slMaxNumPerReserve",
                "The parameter which indicates the maximum number of reserved "
                "PSCCH/PSSCH resources that can be indicated by an SCI.",
                slMaxNumPerReserve);
  cmd.AddValue ("slProbResourceKeep",
                "The parameter which indicates the probability with which the "
                "UE keeps the current resource when the resource reselection"
                "counter reaches zero.",
                slProbResourceKeep);
  cmd.AddValue ("slMaxTxTransNumPssch",
                "The parameter which indicates the maximum transmission number "
                "(including new transmission and retransmission) for PSSCH.",
                slMaxTxTransNumPssch);
  cmd.AddValue ("ReservationPeriod",
                "The resource reservation period in ms",
                reservationPeriod);
  cmd.AddValue ("enableSensing",
                "If true, it enables the sensing based resource selection for "
                "SL, otherwise, no sensing is applied",
                enableSensing);
  cmd.AddValue ("t1",
                "The start of the selection window in physical slots, "
                "accounting for physical layer processing delay",
                t1);
  cmd.AddValue ("t2",
                "The end of the selection window in physical slots",
                t2);
  cmd.AddValue ("slThresPsschRsrp",
                "A threshold in dBm used for sensing based UE autonomous resource selection",
                slThresPsschRsrp);
  cmd.AddValue ("enableChannelRandomness",
                "Enable shadowing and channel updates",
                enableChannelRandomness);
  cmd.AddValue ("channelUpdatePeriod",
                "The channel update period in ms",
                channelUpdatePeriod);
  cmd.AddValue ("mcs",
                "The MCS to used for sidelink",
                mcs);

  /* @VALERIO, @MATTIA -> Added command line values for new parameters */
  cmd.AddValue ("AreaOfRelevance",
                "Set the radius of the Area of Relevance (AoR) in meters",
                AoR_radius);
  cmd.AddValue ("RedundancyMitigation",
                "Enable the Redundancy Mitigation Rules defined by ETSI for CPMs",
                redundancyMitigation);
  cmd.AddValue ("VoIComputationMethod",
                "Set the method to compute Value of Information for Perceived Objects (CPM)",
                VoIcomputationMethod);
  cmd.AddValue ("CPMGenerationPeriod",
                "Set the CPM Generation Period",
                T_GenCpm);
  cmd.AddValue ("SUMOSeed",
                "Set the seed for TraCI client",
                sumo_seed);


  // Parse the command line
  cmd.Parse (argc, argv);

  if (verbose)
  {
    LogComponentEnable ("v2v-nrv2x", LOG_LEVEL_INFO);
    LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
    LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
  }

  /*
   * Check if the frequency is in the allowed range.
   * If you need to add other checks, here is the best position to put them.
   */
  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /***  Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == nullptr)
  {
    NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
  }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
  {
    NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
  }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");
  /*
   * Create a NodeContainer for all the UEs
   */
  NodeContainer allSlUesContainer;
  allSlUesContainer.Create(numberOfNodes);

  /*
   * Assign mobility to the UEs.
   */
  MobilityHelper mobility;
  mobility.Install (allSlUesContainer);

  /*
   * Setup the NR module. We create the various helpers needed for the
   * NR simulation:
   * - EpcHelper, which will setup the core network
   * - NrHelper, which takes care of creating and connecting the various
   * part of the NR stack
   */
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  /*
   * Spectrum division. We create one operational band, containing
   * one component carrier, and a single bandwidth part
   * centered at the frequency specified by the input parameters.
   * We will use the StreetCanyon channel modeling.
   */
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
   * creates a single BWP per CC
   */
  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  //CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::CV2X_UrbanMicrocell);

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  /*
   * The configured spectrum division is:
   * ------------Band1--------------
   * ------------CC1----------------
   * ------------BwpSl--------------
   */
  if (enableChannelRandomness)
  {
    Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
    nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
    nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
  }
  else
  {
    Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
    nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
    nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
  }

  /*
   * Initialize channel and pathloss, plus other things inside bandSl. If needed,
   * the band configuration can be done manually, but we leave it for more
   * sophisticated examples. For the moment, this method will take care
   * of all the spectrum initialization needs.
   */
  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});


  /*
   * Now, we can set up the attributes. We can have three kind of attributes:
   */

  /*
   * Antennas for all the UEs
   * We are not using beam forming in SL, rather we are using
   * quasi-omnidirectional transmission and reception, which is the default
   * configuration of the beams.
   *
   * Following attribute would be common for all the UEs
   */
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  /*
   * We have configured the attributes we needed. Now, install and get the pointers
   * to the NetDevices, which contains all the NR stack:
   */
  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (allSlUesContainer, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
  {
    DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
  }

  /*
   * Configure Sidelink. We create the following helpers needed for the
   * NR Sidelink, i.e., V2X simulation:
   * - NrSlHelper, which will configure the UEs protocol stack to be ready to
   *   perform Sidelink related procedures.
   * - EpcHelper, which takes care of triggering the call to EpcUeNas class
   *   to establish the NR Sidelink bearer(s). We note that, at this stage
   *   just communicate the pointer of already instantiated EpcHelper object,
   *   which is the same pointer communicated to the NrHelper above.
   */
  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  /*
   * Set the SL error model and AMC
   * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
   *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
   * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
   */
  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  /*
   * Set the SL scheduler attributes
   * In this example we use NrSlUeMacSchedulerSimple scheduler, which uses
   * fix MCS value
   */
  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  /*
   * Very important method to configure UE protocol stack, i.e., it would
   * configure all the SAPs among the layers, setup callbacks, configure
   * error model, configure AMC, and configure ChunkProcessor in Interference
   * API.
   */
  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);


  /*
   * Start preparing for all the sub Structs/RRC Information Element (IEs)
   * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
   * hold all the pre-configuration related to Sidelink.
   */

  //SlResourcePoolNr IE
  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();
  /*
   * Above pool factory is created to help the users of the simulator to create
   * a pool with valid default configuration. Please have a look at the
   * constructor of NrSlCommPreconfigResourcePoolFactory class.
   *
   * In the following, we show how one could change those default pool parameter
   * values as per the need.
   */
  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  //Configure the BWP IE
  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
  {
    // it is the BWP id
    slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
  }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams{};
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList{};
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  /****************************** End SL Configuration ***********************/

  /*
   * Fix the random streams
   */
  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  /*
   * if enableOneTxPerLane is true:
   *
   * Divide the UEs in transmitting UEs and receiving UEs. Each lane can
   * have only odd number of UEs, and on each lane middle UE would
   * be the transmitter.
   *
   * else:
   *
   * All the UEs can transmit and receive
   */
  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (allSlUesContainer);
  rxSlUes.Add (allSlUesContainer);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  /*
   * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
   * configured time.
   *
   * This example supports IPV4 and IPV6
   */

  InternetStackHelper internet;
  internet.Install (allSlUesContainer);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < allSlUesContainer.GetN (); ++u)
  {
    Ptr<Node> ueNode = allSlUesContainer.Get (u);
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
  }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  // enable log component
  //LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (penetrationRate));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (sumo_seed));

  std::string sumo_additional_options = "--verbose true";

  if(!sumo_netstate_file_name.empty())
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
    vehicleVis->startServer();
    vehicleVis->connectToServer ();
    sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = nullptr;
  MetricSupervisor metSupObj((int) m_baseline_prr);
  if(m_metric_sup)
  {
    metSup = &metSupObj;
    metSup->setTraCIClient(sumoClient);
    metSup->disablePRRVerboseOnStdout();
    metSup->setChannelTechnology("Nr");
    metSup->enableCBRVerboseOnStdout();
    metSup->enableCBRWriteToFile();
    metSup->setCBRWindowValue(100);
    metSup->setCBRAlphaValue(0.5);
    metSup->setSimulationTimeValue((float) simTime);
    //metSup->startCheckCBR();
  }

  /*** 7. Setup interface and application for dynamic nodes ***/
  emergencyVehicleAlertHelper EmergencyVehicleAlertHelper;
  EmergencyVehicleAlertHelper.SetAttribute ("Client", PointerValue (sumoClient));
  EmergencyVehicleAlertHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  EmergencyVehicleAlertHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  EmergencyVehicleAlertHelper.SetAttribute ("CSV", StringValue(csv_name));
  EmergencyVehicleAlertHelper.SetAttribute ("Model", StringValue ("nrv2x"));
  EmergencyVehicleAlertHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  EmergencyVehicleAlertHelper.SetAttribute ("SendCPM", BooleanValue(true));
  EmergencyVehicleAlertHelper.SetAttribute ("SendCAM", BooleanValue(true));
  /* @VALERIO, @MATTIA -> Added Attributes for new parameters */
  EmergencyVehicleAlertHelper.SetAttribute ("AreaOfRelevance", DoubleValue(AoR_radius));
  EmergencyVehicleAlertHelper.SetAttribute ("RedundancyMitigation", BooleanValue(redundancyMitigation));
  EmergencyVehicleAlertHelper.SetAttribute ("VoIComputationMethod", StringValue(VoIcomputationMethod));
  EmergencyVehicleAlertHelper.SetAttribute ("CPMGenerationPeriod", IntegerValue(T_GenCpm));

  /* callback function for node creation */
  int i=0;
  STARTUP_FCN setupNewWifiNode = [&] (const std::string& vehicleID) -> Ptr<Node>
  {
      if (nodeCounter >= allSlUesContainer.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = allSlUesContainer.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      EmergencyVehicleAlertHelper.SetAttribute ("IpAddr", Ipv4AddressValue(groupAddress4));
      i++;

      //ApplicationContainer CAMSenderApp = CamSenderHelper.Install (includedNode);
      ApplicationContainer AppSample = EmergencyVehicleAlertHelper.Install (includedNode);

      //AppSample.Start (Seconds (0.0));
      AppSample.Start (Seconds (1.5 + double(rand())/RAND_MAX));
      AppSample.Stop (Seconds(simTime) - Simulator::Now () - Seconds (0.1));

      return includedNode;
  };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
  {
      /* stop all applications */
      Ptr<emergencyVehicleAlert> appSample_ = exNode->GetApplication(0)->GetObject<emergencyVehicleAlert>();

      if(appSample_)
        appSample_->StopApplicationNow();

      /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
  };

  /* start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /* @VALERIO -> detect the number of connected vehicles */
  Simulator::Schedule(Seconds(3.0), &detectConnectedVehicles, sumoClient);

  /* @VALERIO -> compute the Environmental Awareness Ratio (EAR) */
  if(m_metric_sup)
  {
    Simulator::Schedule(Seconds(4.0), &computeAwarenessRatio, sumoClient, AoR_radius);
    Simulator::Schedule(Seconds(4.0), std::bind(&MetricSupervisor::startCheckCBR, metSup));
  }


  /*** 8. Start Simulation ***/
  Simulator::Stop (Seconds(simTime));

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
  {
    /** @VALERIO PRR data processing */
    std::ofstream csv_ofstream_prr;
    std::string csv_name_prr = "Results/PRR/Average_PRR.csv";

    if(access(csv_name_prr.c_str(),F_OK)!=-1)
    {
      // The file already exists
      csv_ofstream_prr.open(csv_name_prr,std::ofstream::out | std::ofstream::app);
    }
    else
    {
      // The file does not exist yet
      csv_ofstream_prr.open(csv_name_prr);
      csv_ofstream_prr << "numberOfUEs,RMR,MPR,T_Cpm_Gen,Average PRR,Average Latency" << std::endl;
    }

    // Fill the CVS file
    csv_ofstream_prr << numberOfNodes << "," << VoIcomputationMethod << "," << penetrationRate << "," << T_GenCpm << "," << metSup->getAveragePRR_overall() << "," << metSup->getAverageLatency_overall () << std::endl;

    std::cout << "Average PRR: " << metSup->getAveragePRR_overall() << std::endl;
    std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;


    /** @VALERIO CBR data processing */
    std::ofstream csv_ofstream_cbr;
    std::string csv_name_cbr = "Results/CBR/Average_CBR.csv";

    if(access(csv_name_cbr.c_str(),F_OK)!=-1)
    {
      // The file already exists
      csv_ofstream_cbr.open(csv_name_cbr,std::ofstream::out | std::ofstream::app);
    }
    else
    {
      // The file does not exist yet
      csv_ofstream_cbr.open(csv_name_cbr);
      csv_ofstream_cbr << "numberOfUEs,RMR,MPR,T_Cpm_Gen,Average CBR" << std::endl;
    }

    // Fill the CVS file
    csv_ofstream_cbr << numberOfNodes << "," << VoIcomputationMethod << "," << penetrationRate << "," << T_GenCpm << "," << metSup->getAverageCBROverall() << std::endl;

    std::cout << "Average CBR: " << metSup->getAverageCBROverall() << std::endl;

    /** @VALERIO EAR data processing */
    std::ofstream csv_ofstream_ear;
    std::string csv_name_ear = "Results/EAR/Average_EAR.csv";

    if(access(csv_name_ear.c_str(),F_OK)!=-1)
    {
      // The file already exists
      csv_ofstream_ear.open(csv_name_ear,std::ofstream::out | std::ofstream::app);
    }
    else
    {
      // The file does not exist yet
      csv_ofstream_ear.open(csv_name_ear);
      csv_ofstream_ear << "numberOfUEs,RMR,MPR,T_Cpm_Gen,Average EAR" << std::endl;
    }

    // Compute the Average EAR
    double Average_EAR = 0;
    for(auto it:Average_EAR_Overall)
    {
      Average_EAR += it;
    }
    if(!Average_EAR_Overall.empty())
      Average_EAR /= (double) Average_EAR_Overall.size();

    // Fill the CVS file
    csv_ofstream_ear << numberOfNodes << "," << VoIcomputationMethod << "," << penetrationRate << "," << T_GenCpm << "," << Average_EAR << std::endl;

    std::cout << "Average EAR: " << Average_EAR << std::endl;
  }

  return 0;
}


