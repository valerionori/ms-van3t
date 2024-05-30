/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
 * Modified by:
 *  Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)
 *  Mattia Andreani, Università di Modena e Reggio Emilia (mattia.andreani@unimore.it)
*/

#include "emergencyVehicleAlert.h"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("emergencyVehicleAlert");

  NS_OBJECT_ENSURE_REGISTERED(emergencyVehicleAlert);

  // Function to compute the distance between two objects, given their Lon/Lat
  double appUtil_haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
      // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
      return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
  }

  // Function to compute the absolute difference between two angles (angles must be between -180 and 180)
  double appUtil_angDiff(double ang1, double ang2) {
      double angDiff;
      angDiff=ang1-ang2;

      if(angDiff>180)
      {
        angDiff-=360;
      }
      else if(angDiff<-180)
      {
        angDiff+=360;
      }
      return std::abs(angDiff);
  }

  TypeId
  emergencyVehicleAlert::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::emergencyVehicleAlert")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<emergencyVehicleAlert> ()
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&emergencyVehicleAlert::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&emergencyVehicleAlert::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&emergencyVehicleAlert::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&emergencyVehicleAlert::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Model",
            "Physical and MAC layer communication model",
            StringValue (""),
            MakeStringAccessor (&emergencyVehicleAlert::m_model),
            MakeStringChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&emergencyVehicleAlert::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&emergencyVehicleAlert::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
            "To enable/disable the transmission of CAM messages",
            BooleanValue(true),
            MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cam),
            MakeBooleanChecker ())
        .AddAttribute ("SendCPM",
           "To enable/disable the transmission of CPM messages",
           BooleanValue(true),
           MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cpm),
           MakeBooleanChecker ())

        /* @VALERIO, @MATTIA -> Added Attributes for new parameters */
        .AddAttribute ("RedundancyMitigation",
                       "To enable the Redundancy Mitigation Rules defined by ETSI fro CPMs",
                       StringValue (),
                       MakeBooleanAccessor (&emergencyVehicleAlert::m_redundancy_mitigation),
                       MakeBooleanChecker ())
        .AddAttribute ("AreaOfRelevance",
                       "To set Area of Relevance (AoR) radius",
                       DoubleValue (),
                       MakeDoubleAccessor ( &emergencyVehicleAlert::m_AoR_radius),
                       MakeDoubleChecker<double>())
        .AddAttribute("CPMGenerationPeriod",
                      "To set the CPM Generation Period",
                      IntegerValue(),
                      MakeIntegerAccessor(&emergencyVehicleAlert::m_T_GenCpm),
                      MakeIntegerChecker<int16_t>());
        return tid;
  }

  emergencyVehicleAlert::emergencyVehicleAlert ()
  {
    NS_LOG_FUNCTION(this);
    m_client = nullptr;
    m_print_summary = true;
    m_already_print = false;
    m_send_cam = true;

    m_denm_sent = 0;
    m_cam_received = 0;
    m_cpm_received = 0;
    m_denm_received = 0;
    m_denm_intertime = 0;

    m_distance_threshold = 75; // Distance used in GeoNet to determine the radius of the circumference around the emergency vehicle where the DENMs are valid
    m_heading_threshold = 45; // Max heading angle difference between the normal vehicles and the emergency vehicle, that triggers a reaction in the normal vehicles

    m_redundancy_mitigation = true; // @VALERIO
    m_T_GenCpm = 100; // @VALERIO, @MATTIA
  }

  emergencyVehicleAlert::~emergencyVehicleAlert ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  emergencyVehicleAlert::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  emergencyVehicleAlert::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    /*
     * In this example, the vehicle can be either of type "passenger" or of type "emergency" (see cars.rou.xml in SUMO folder inside examples/sumo_files_v2v_map)
     * All the vehicles broadcast CAM messages. When a "passenger" car receives a CAM from an "emergency" vehicle, it checks the distance between them and
     * the difference in heading, and if it considers it to be close, it takes proper actions to facilitate the takeover maneuver.
     */

    /* Save the vehicles informations */
    m_id = m_client->GetVehicleId (this->GetNode ());
    m_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
    m_max_speed = m_client->TraCIAPI::vehicle.getMaxSpeed (m_id);

    VDP* traci_vdp = new VDPTraCI(m_client,m_id);

    //Create LDM and sensor object
    m_LDM = CreateObject<LDM>();
    m_LDM->setStationID(m_id);
    m_LDM->setTraCIclient(m_client);
    m_LDM->setVDP(traci_vdp);
    m_LDM->setAreaOfRelevance(m_AoR_radius); // @VALERIO

    m_sensor = CreateObject<SUMOSensor>();
    m_sensor->setStationID(m_id);
    m_sensor->setTraCIclient(m_client);
    m_sensor->setVDP(traci_vdp);
    m_sensor->setLDM (m_LDM);
    m_sensor->setSensorRange (50); // @VALERIO

    // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();

    if(m_metric_supervisor!=nullptr)
    {
      m_geoNet->setMetricSupervisor(m_metric_supervisor);
    }

    m_btp->setGeoNet(m_geoNet);
    m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);
    m_cpService.setBTP(m_btp);
    m_caService.setLDM(m_LDM);
    m_cpService.setLDM(m_LDM);

    /* Create the Sockets for TX and RX */
    TypeId tid;
    if(m_model=="80211p")
      tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    else if(m_model=="cv2x" || m_model=="nrv2x")
      tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    else
      NS_FATAL_ERROR ("No communication model set - check simulation script - valid models: '80211p' or 'lte'");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    if(m_model=="80211p")
    {
        /* Bind the socket to local address */
        PacketSocketAddress local = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetAddress () );
        if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
        // Set the socketAddress for broadcast
        PacketSocketAddress remote = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetBroadcast () );
        m_socket->Connect (remote);
    }
    else // m_model=="cv2x"
    {
        /* The C-V2X model requires the socket to be bind to "any" IPv4 address, and to be connected to the
         * IP address of the transmitting node. Then, the model will take care of broadcasting the packets.
        */
        if (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for C-V2X");
        }
        m_socket->Connect (InetSocketAddress(m_ipAddress,19));
    }

    /* Set Station Type in DENBasicService */
    StationType_t stationtype;
    if (m_type=="passenger")
      stationtype = StationType_passengerCar;
    else if (m_type=="emergency"){
      stationtype = StationType_specialVehicle;
      m_LDM->enablePolygons (); // Uncomment to enable detected object polygon visualization for this specific vehicle
      }
    else
      stationtype = StationType_unknown;

    libsumo::TraCIColor connected;
    connected.r=0;connected.g=225;connected.b=255;connected.a=255;
    m_client->TraCIAPI::vehicle.setColor (m_id, connected);

    /* Set sockets, callback and station properties in DENBasicService */
    m_denService.setSocketTx (m_socket);
    m_denService.setSocketRx (m_socket);
    m_denService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    //m_denService.addDENRxCallback ([this](auto && PH1, auto && PH2) { receiveDENM(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });
    m_denService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_caService.addCARxCallback ([this](auto && PH1, auto && PH2) { receiveCAM(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });
    m_caService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties and TraCI VDP in CPBasicService */
    m_cpService.setSocketTx (m_socket);
    m_cpService.setSocketRx (m_socket);
    m_cpService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_cpService.addCPRxCallback ([this](auto && PH1, auto && PH2) { receiveCPM(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });
    m_cpService.setRealTime (m_real_time);
    m_cpService.setTraCIclient (m_client);
    m_cpService.enableRedundancyMitigation(true); // @VALERIO
    m_cpService.changeTGenCpm(m_T_GenCpm); // @VALERIO

    /* Set TraCI VDP for GeoNet object */
    m_caService.setVDP(traci_vdp);
    m_denService.setVDP(traci_vdp);
    m_cpService.setVDP(traci_vdp);

    /* Schedule CAM dissemination */
    if(m_send_cam)
    {
      // Old desync code kept just for reference
      // It may lead to nodes not being desynchronized properly in specific situations in which
      // Simulator::Now().GetNanoSeconds () returns the same seed for multiple nodes
      // std::srand(Simulator::Now().GetNanoSeconds ());
      // double desync = ((double)std::rand()/RAND_MAX);

      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue ();

      m_caService.startCamDissemination(desync);
    }

    /* Schedule CPM dissemination */
    if(m_send_cpm)
    {
      m_cpService.startCpmDissemination ();
    }

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name+"-"+m_id+"-CAM.csv",std::ofstream::trunc);
      m_csv_ofstream_cam << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
    }
  }

  void
  emergencyVehicleAlert::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_speed_ev);
    Simulator::Cancel(m_send_cam_ev);
    Simulator::Cancel(m_update_denm_ev);

    uint64_t cam_sent, cpm_sent;

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.close ();
    }

    cam_sent = m_caService.terminateDissemination ();
    cpm_sent = m_cpService.terminateDissemination ();
    m_denService.cleanup();
    m_LDM->cleanup();
    m_sensor->cleanup();

    if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id
                << ",CAM-SENT:" << cam_sent
                << ",CAM-RECEIVED:" << m_cam_received
                << ",CPM-SENT: " << cpm_sent
                << ",CPM-RECEIVED: " << m_cpm_received
                << std::endl;
      m_already_print=true;
    }
  }

  void
  emergencyVehicleAlert::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  emergencyVehicleAlert::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
   /* Implement CAM strategy here */
   m_cam_received++;
  }

  void
  emergencyVehicleAlert::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
  {
    /* Implement CPM strategy here */
    m_cpm_received++;
    (void) from;
    //For every PO inside the CPM, if any
    bool POs_ok;
    //auto wrappedContainer = asn1cpp::makeSeq(WrappedCpmContainer);
    int wrappedContainer_size = asn1cpp::sequenceof::getSize(cpm->payload.cpmContainers);
    for (int i=0; i<wrappedContainer_size; i++)
      {
        auto wrappedContainer = asn1cpp::sequenceof::getSeq(cpm->payload.cpmContainers,WrappedCpmContainer,i);
        WrappedCpmContainer__containerData_PR present = asn1cpp::getField(wrappedContainer->containerData.present,WrappedCpmContainer__containerData_PR);
        if(present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
        {
          auto POcontainer = asn1cpp::getSeq(wrappedContainer->containerData.choice.PerceivedObjectContainer,PerceivedObjectContainer);
          int PObjects_size = asn1cpp::sequenceof::getSize(POcontainer->perceivedObjects);
          std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CPMv2 from " << asn1cpp::getField(cpm->header.stationId,long) << " with " << PObjects_size << " perceived objects." << std::endl;
          for(int j=0; j<PObjects_size;j++)
              {
               LDM::returnedVehicleData_t PO_data;
               auto PO_seq = asn1cpp::makeSeq(PerceivedObject);
               PO_seq = asn1cpp::sequenceof::getSeq(POcontainer->perceivedObjects,PerceivedObject,j);
               //If PO is already in local copy of vLDM
               if(m_LDM->lookup(asn1cpp::getField(PO_seq->objectId,long),PO_data) == LDM::LDM_OK)
                    {
                      //Add the new perception to the LDM
                      std::vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
                      if(std::find(associatedCVs.begin(), associatedCVs.end (), asn1cpp::getField(cpm->header.stationId,long)) == associatedCVs.end ())
                        associatedCVs.push_back (asn1cpp::getField(cpm->header.stationId,long));
                      PO_data.vehData.associatedCVs = OptionalDataItem<std::vector<long>>(associatedCVs);
                      m_LDM->insert (PO_data.vehData);
                    }
               else
                    {
                      //Translate CPM data to LDM format
                      m_LDM->insert(translateCPMdata(cpm,PO_seq,j));
                    }
              }
        }
      }
  }

  vehicleData_t
  emergencyVehicleAlert::translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                           asn1cpp::Seq<PerceivedObject> object, int objectIndex)
  {
    vehicleData_t retval;
    retval.detected = true;
    retval.stationID = asn1cpp::getField(object->objectId,long);
    retval.ID = std::to_string(retval.stationID);
    retval.vehicleLength = asn1cpp::getField(object->objectDimensionX->value,long);
    retval.vehicleWidth = asn1cpp::getField(object->objectDimensionY->value,long);
    retval.heading = asn1cpp::getField(object->angles->zAngle.value,double) / DECI;
    retval.xSpeedAbs.setData (asn1cpp::getField(object->velocity->choice.cartesianVelocity.xVelocity.value,long));
    retval.xSpeedAbs.setData (asn1cpp::getField(object->velocity->choice.cartesianVelocity.yVelocity.value,long));
    retval.speed_ms = (sqrt (pow(retval.xSpeedAbs.getData(),2) +
                             pow(retval.ySpeedAbs.getData(),2)))/CENTI;

    libsumo::TraCIPosition fromPosition = m_client->TraCIAPI::simulation.convertLonLattoXY (asn1cpp::getField(cpm->payload.managementContainer.referencePosition.longitude,double)/DOT_ONE_MICRO,
                                                                                           asn1cpp::getField(cpm->payload.managementContainer.referencePosition.latitude,double)/DOT_ONE_MICRO);
    libsumo::TraCIPosition objectPosition = fromPosition;
    objectPosition.x += asn1cpp::getField(object->position.xCoordinate.value,long)/CENTI;
    objectPosition.y += asn1cpp::getField(object->position.yCoordinate.value,long)/CENTI;
    objectPosition = m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x,objectPosition.y);
    retval.lon = objectPosition.x;
    retval.lat = objectPosition.y;

    retval.camTimestamp = asn1cpp::getField(cpm->payload.managementContainer.referenceTime,long);
    retval.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(object->measurementDeltaTime,long)*1000);
    retval.stationType = StationType_passengerCar;
    retval.perceivedBy.setData(asn1cpp::getField(cpm->header.stationId,long));

        return retval;
  }


  }





