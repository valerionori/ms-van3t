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
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
 * Modified by:
 *  Valerio Nori, Università di Modena e Reggio Emilia (valerio.nori@hotmail.com)
 *  Mattia Andreani, Università di Modena e Reggio Emilia (mattia.andreani@unimore.it)
*/
#include "LDM.h"
#include <cmath>
#include <iostream>

#define DEG_2_RAD(val) ((val)*M_PI/180.0)

#define VEHICLE_AREA 9
#define LOG_FREQ 100

namespace ns3 {

  // Function to compute the distance between two objects, given their Lon/Lat
  double compute_dist(double lat_a, double lon_a, double lat_b, double lon_b)
  {
      // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
      return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
  }
  const point_type frontLeftPoint(0.0, 0.5);
  const point_type frontRightPoint(0.0, -0.5);
  const point_type backRightPoint(-1.0, -0.5);
  const point_type backLeftPoint(-1.0, 0.5);
  const point_type boxCenterPoint(-0.5, 0.0);
  uint64_t get_timestamp_us(void)
  {
    time_t seconds;
    uint64_t microseconds;
    struct timespec now;

    if(clock_gettime(CLOCK_REALTIME, &now) == -1) {
            perror("Cannot get the current microseconds UTC timestamp");
            return -1;
    }

    seconds=now.tv_sec;
    microseconds=round(now.tv_nsec/1e3);

    // milliseconds, due to the rounding operation, shall not exceed 999999
    if(microseconds > 999999) {
            seconds++;
            microseconds=0;
    }

    return seconds*1000000+microseconds;
  }

  LDM::LDM()
  {
    m_card = 0;
    m_count = 0;
    m_stationID = 0;
    m_polygons=false;
    m_client=NULL;

    m_LDM = std::unordered_map<uint64_t,returnedVehicleData_t> ();

    m_event_deleteOlderThan = Simulator::Schedule(Seconds(DB_CLEANER_INTERVAL_SECONDS),&LDM::deleteOlderThan,this);

    std::srand(Simulator::Now().GetNanoSeconds ());
    double desync = ((double)std::rand()/RAND_MAX);
    m_event_writeContents = Simulator::Schedule(MilliSeconds(LOG_FREQ+(desync*100)),&LDM::writeAllContents,this);

    /* @VALERIO -> Schedule AoR check */
    m_event_checkAreaOfRelevance = Simulator::Schedule(MilliSeconds(LOG_FREQ+(desync*100)),&LDM::checkAreaOfRelevance,this);
    m_event_checkAgeOfInformation = Simulator::Schedule(MilliSeconds(LOG_FREQ+(desync*100)),&LDM::checkAgeOfInformation,this);

    /* @VALERIO -> AoR Initialization */
    AoR_radius = 250;
    m_AoR = std::vector<std::string> ();

    /* @VALERIO -> Redundancy Mitigation Map Initialization */
    m_enableRDM = false;
    m_RDM = std::unordered_map<uint64_t,redundancyMitigation_t> ();
  }

  LDM::~LDM() {
      Simulator::Cancel(m_event_deleteOlderThan);
      Simulator::Cancel(m_event_writeContents);
      Simulator::Cancel (m_event_checkAreaOfRelevance); // @VALERIO
      Simulator::Cancel(m_event_checkAgeOfInformation); // @VALERIO
      clear();
  }

  LDM::LDM_error_t
  LDM::insert(vehicleData_t newVehicleData)
  {
    LDM_error_t retval;

    if(m_card==UINT64_MAX) {
            return LDM_MAP_FULL;
    }

    auto it = m_LDM.find(newVehicleData.stationID);

    if (it == m_LDM.end()) {
        newVehicleData.age_us = Simulator::Now().GetMicroSeconds ();
        m_LDM[newVehicleData.stationID].vehData = newVehicleData;
        m_LDM[newVehicleData.stationID].phData = PHpoints();
        m_LDM[newVehicleData.stationID].phData.insert (newVehicleData,m_stationID);
        m_card++;
        retval = LDM_OK;
    } else {
        newVehicleData.age_us = it->second.vehData.age_us;
        it->second.vehData = newVehicleData;
        it->second.phData.insert (newVehicleData,m_stationID);
        retval = LDM_UPDATED;
    }

 /**
 * @VALERIO
 *
 * The following lines will properly fill the RDM (Redundancy Dynamic Map) which will be used to implement
 * Value of Information (VoI) computation methods as specified in ETSI TS 103 324 V2.1.1 (2023-06) Annex E
 */
    if (m_enableRDM)
    {
      /* Check if the Object is perceived from a received CPM (not by local sensors) */
      if(newVehicleData.perceivedBy.getData() != long(m_stationID))
      {
        /* If the Object is not in the RDM, insert the Object with initialized values */
        auto iterator = m_RDM.find(newVehicleData.stationID);
        if(iterator == m_RDM.end ())
        {
          /* Information about the perceived Object */
          redundancyMitigation_t redundancyMitigationObject;
          redundancyMitigationObject.counter = 1;
          redundancyMitigationObject.timestamp = Simulator::Now ().GetMicroSeconds ();
          redundancyMitigationObject.latitude = newVehicleData.lat;
          redundancyMitigationObject.longitude = newVehicleData.lon;
          redundancyMitigationObject.speed = newVehicleData.speed_ms;

          if (newVehicleData.perceivedBy.isAvailable())
          {
            /* Extract the positions of the current ITS-S and of the ITS-S which sent the CPM from TraCI Client */
            libsumo::TraCIPosition egoPosXY = m_client->TraCIAPI::vehicle.getPosition(m_id);
            std::string pID = "veh" + std::to_string(newVehicleData.perceivedBy.getData());
            libsumo::TraCIPosition PosXY = m_client->TraCIAPI::vehicle.getPosition(pID);
            redundancyMitigationObject.distance = sqrt(pow((egoPosXY.x-PosXY.x),2) + pow((egoPosXY.y-PosXY.y),2));
          }
          else
            redundancyMitigationObject.distance = 0;

          redundancyMitigationObject.CPM_sent = 0;

          m_RDM.insert(std::make_pair(newVehicleData.stationID, redundancyMitigationObject));

        }
        else
        {
          /* If the Object is already in the RDM, find iterator to update its values */
          for(iterator = m_RDM.begin (); iterator != m_RDM.end (); ++iterator)
          {
            if(iterator->first == newVehicleData.stationID) // Find the Object in the RDM
            {
              /* If the Object AoI is greater than W_InclusionRateControl, initialize all the values of the Object */
              if((Simulator::Now ().GetMicroSeconds () - iterator->second.timestamp) >= (W_InclusionRateControl * 1000))
              {
                m_RDM[newVehicleData.stationID].counter = 1;
                m_RDM[newVehicleData.stationID].timestamp = Simulator::Now ().GetMicroSeconds ();
                m_RDM[newVehicleData.stationID].latitude = newVehicleData.lat;
                m_RDM[newVehicleData.stationID].longitude = newVehicleData.lon;
                m_RDM[newVehicleData.stationID].speed = newVehicleData.speed_ms;
                if (newVehicleData.perceivedBy.isAvailable())
                {
                  /* Extract the positions of the current ITS-S and of the ITS-S which sent the CPM from TraCI Client */
                  libsumo::TraCIPosition egoPosXY = m_client->TraCIAPI::vehicle.getPosition(m_id);
                  std::string pID = "veh" + std::to_string(newVehicleData.perceivedBy.getData());
                  libsumo::TraCIPosition PosXY = m_client->TraCIAPI::vehicle.getPosition(pID);
                  m_RDM[newVehicleData.stationID].distance = sqrt(pow((egoPosXY.x-PosXY.x),2) + pow((egoPosXY.y-PosXY.y),2));
                }
                else
                  m_RDM[newVehicleData.stationID].distance = 0;

                m_RDM[newVehicleData.stationID].CPM_sent = 0;
              }
              else
              {
                // Do not update the timestamp
                // Update the counter
                m_RDM[newVehicleData.stationID].counter++;
                m_RDM[newVehicleData.stationID].latitude = newVehicleData.lat;
                m_RDM[newVehicleData.stationID].longitude = newVehicleData.lon;
                m_RDM[newVehicleData.stationID].speed = newVehicleData.speed_ms;
                /* If the Object is perceived by a farther ITS-S, update the distance */
                if (newVehicleData.perceivedBy.isAvailable())
                {
                  /* Extract the positions of the current ITS-S and of the ITS-S which sent the CPM from TraCI Client */
                  libsumo::TraCIPosition egoPosXY = m_client->TraCIAPI::vehicle.getPosition(m_id);
                  std::string pID = "veh" + std::to_string(newVehicleData.perceivedBy.getData());
                  libsumo::TraCIPosition PosXY = m_client->TraCIAPI::vehicle.getPosition(pID);
                  if(m_RDM[newVehicleData.stationID].distance <= sqrt(pow((egoPosXY.x-PosXY.x),2) + pow((egoPosXY.y-PosXY.y),2)))
                    m_RDM[newVehicleData.stationID].distance = sqrt(pow((egoPosXY.x-PosXY.x),2) + pow((egoPosXY.y-PosXY.y),2));
                }
              }
            }
          }
        }

        /* If data about the ITS-S which perceived the Object is not in the RDM, insert it */
        auto ir = m_RDM.find(uint64_t(newVehicleData.perceivedBy.getData()));
        if(ir == m_RDM.end ())
        {
          /* Information about the ITS-S which perceived the Object */
          redundancyMitigation_t redundancyMitigationSender;
          redundancyMitigationSender.counter = 0;
          redundancyMitigationSender.timestamp = 0;
          redundancyMitigationSender.distance = 0;
          redundancyMitigationSender.latitude = 0;
          redundancyMitigationSender.longitude = 0;
          redundancyMitigationSender.speed = 0;
          redundancyMitigationSender.CPM_sent = 1;

          m_RDM.insert(std::make_pair(uint64_t(newVehicleData.perceivedBy.getData()), redundancyMitigationSender));
        }
        else
        {
          /* If data about the ITS-S which perceived the Object are already in the RDM, update its CPM counter */
          m_RDM[newVehicleData.perceivedBy.getData()].CPM_sent++;
        }

        /* Fill the CSV file */
        m_csv_name_rdm = "Results/RDM/RDM-" + m_id + ".csv";
        std::ifstream m_csv_ifstream_rdm(m_csv_name_rdm);
        m_csv_ofstream_rdm.open(m_csv_name_rdm, std::ofstream::app);
        if (!m_csv_ifstream_rdm.is_open())
          m_csv_ofstream_rdm << "ObjectID,timestamp,first_detection,PerceivedBy,Counter,Latitude,Longitude,Speed,Sender ITS-S Distance,CPM Sent" << std::endl;
        if(newVehicleData.perceivedBy.isAvailable())
        {
          m_csv_ofstream_rdm.open(m_csv_name_rdm, std::ofstream::app);
          m_csv_ofstream_rdm << newVehicleData.stationID << ","
                             << Simulator::Now ().GetMicroSeconds () << ","
                             << m_RDM[newVehicleData.stationID].timestamp << ","
                             << newVehicleData.perceivedBy.getData() << ","
                             << m_RDM[newVehicleData.stationID].counter << ","
                             << m_RDM[newVehicleData.stationID].latitude << ","
                             << m_RDM[newVehicleData.stationID].longitude << ","
                             << m_RDM[newVehicleData.stationID].speed << ","
                             << m_RDM[newVehicleData.stationID].distance << ","
                             << m_RDM[newVehicleData.stationID].CPM_sent << ","
                             << std::endl;
        }
      }
    }
    return retval;
  }

  LDM::LDM_error_t
  LDM::remove(uint64_t stationID)
  {

    auto it = m_LDM.find(stationID);

    if (it == m_LDM.end()){
        return LDM_ITEM_NOT_FOUND;
      }
    else{
        m_LDM.erase (it);
        m_card--;
      }
    return LDM_OK;
  }

  LDM::LDM_error_t
  LDM::lookup(uint64_t stationID,returnedVehicleData_t &retVehicleData)
  {

    auto it = m_LDM.find(stationID);

    if (it == m_LDM.end()){
        return LDM_ITEM_NOT_FOUND;
      }
    else{
        retVehicleData = it->second;
      }
    return LDM_OK;
  }

  LDM::LDM_error_t
  LDM::updateCPMincluded(uint64_t stationID,uint64_t timestamp)
  {
    auto it = m_LDM.find(stationID);

    if (it == m_LDM.end()){
        return LDM_ITEM_NOT_FOUND;
      }
    else{
        it->second.vehData.lastCPMincluded = timestamp;
        it->second.phData.setCPMincluded ();
      }
    return LDM_OK;
  }

  LDM::LDM_error_t
  LDM::rangeSelect(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

        if(haversineDist(lat,lon,it->second.vehData.lat,it->second.vehData.lon)<=range_m) {
                selectedVehicles.push_back(it->second);
        }
    }

    return LDM_OK;
  }

  bool
  LDM::getAllPOs (std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    bool retval = false;

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

	if(it->second.vehData.detected) {
		selectedVehicles.push_back(it->second);
		retval = true;
	}
    }

    return retval;
  }

  bool
  LDM::getAllIDs (std::set<int> &IDs)
  {
    bool retval = false;

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
        IDs.insert (it->first);
        retval = true;
    }

    return retval;

  }

  bool
  LDM::getAllCVs(std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    bool retval = false;

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

	if(!it->second.vehData.detected) {
		selectedVehicles.push_back(it->second);
		retval = true;
	}
    }

    return retval;
  }

  LDM::LDM_error_t
  LDM::rangeSelect(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles)
  {
    returnedVehicleData_t retData;

    // Get the latitude and longitude of the speficied vehicle
    if(lookup(stationID,retData)!=LDM_OK) {
            return LDM_ITEM_NOT_FOUND;
    }

    // Perform a rangeSelect() centered on that latitude and longitude values
    return rangeSelect(range_m,retData.vehData.lat,retData.vehData.lon,selectedVehicles);
  }

  void
  LDM::deleteOlderThan()
  {
    uint64_t now = Simulator::Now ().GetMicroSeconds ();
    double curr_dwell = 0.0;

    for (auto it = m_LDM.cbegin(); it != m_LDM.cend();) {
        std::string id = std::to_string(it->second.vehData.stationID);
        if(m_polygons && m_client!=NULL)
          {
            std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
            if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
               !it->second.vehData.detected)
              {
                m_client->TraCIAPI::polygon.remove(id,5);
              }
          }
        if(((double)(now-it->second.vehData.timestamp_us))/1000.0 > DB_DELETE_OLDER_THAN_SECONDS*1000) {
            if(it->second.vehData.detected)
              {
                long age = it->second.vehData.age_us;
                curr_dwell = now - age; //Dwelling time on database
                m_dwell_count ++;
                m_avg_dwell += (curr_dwell-m_avg_dwell)/m_dwell_count;

                if(m_client!=NULL)
                  {
                    std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
                    if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
                     m_polygons)
                      m_client->TraCIAPI::polygon.remove(id,5);
                  }
              }

              it = m_LDM.erase(it);
              m_card--;
            } else {
              ++it;
            }
    }
    m_count++;
    //writeAllContents();
    m_event_deleteOlderThan = Simulator::Schedule(Seconds(DB_CLEANER_INTERVAL_SECONDS),&LDM::deleteOlderThan,this);
  }


  void
  LDM::deleteOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args)
  {
    uint64_t now = get_timestamp_us();
    double curr_dwell = 0.0;

    for (auto it = m_LDM.cbegin(); it != m_LDM.cend();) {
        std::string id = std::to_string(it->second.vehData.stationID);
        if(m_polygons && m_client!=NULL)
          {
            std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
            if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
               !it->second.vehData.detected)
              {
                m_client->TraCIAPI::polygon.remove(id,5);
              }
          }
        if(((double)(now-it->second.vehData.timestamp_us))/1000.0 > DB_DELETE_OLDER_THAN_SECONDS*1000) {
            if(it->second.vehData.detected)
              {
                long age = it->second.vehData.age_us;
                curr_dwell = now - age; //Dwelling time on database
                m_dwell_count ++;
                m_avg_dwell += (curr_dwell-m_avg_dwell)/m_dwell_count;

                if(m_client!=NULL)
                  {
                    std::vector<std::string> polygonList = m_client->TraCIAPI::polygon.getIDList ();
                    if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end () &&
                     m_polygons)
                      m_client->TraCIAPI::polygon.remove(id,5);
                  }
              }
              oper_fcn(it->second.vehData.stationID,additional_args);
              it = m_LDM.erase(it);
              m_card--;
            } else {
              ++it;
            }
    }
  }

  void
  LDM::clear() {

    m_LDM.clear();
    // Set the cardinality of the map to 0 again
    m_card = 0;
  }

  void
  LDM::writeAllContents()
  {
    if (m_client == NULL)
      return;
    if (m_station_type == StationType_pedestrian)
      return;

    libsumo::TraCIPosition egoPosXY=m_client->TraCIAPI::vehicle.getPosition(m_id);

    std::vector<uint64_t> POs,CVs;
    double conf = 0.0;
    double age = 0.0;
    double assoc = 0.0;
    double dist = 0.0;
    double maxDist = 0.0;
    returnedVehicleData_t vehdata = {0};

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {

        if(it->second.vehData.detected)
          POs.push_back (it->second.vehData.stationID);
        else if(!it->second.vehData.detected)
          CVs.push_back (it->second.vehData.stationID);
    }

    for (auto it = POs.begin(); it != POs.end(); it++)
      {
        lookup(*it,vehdata);
        //OptionalDataItem<long> respPMID = vehdata.vehData.respPMID;
        std::vector<long> assocCVIDs = vehdata.vehData.associatedCVs.getData ();

        conf += vehdata.vehData.confidence.getData();
        age += (Simulator::Now ().GetMicroSeconds () - (double) vehdata.vehData.timestamp_us)/1000;


        std::string sID = "veh" + std::to_string(*it);
        libsumo::TraCIPosition PosXY=m_client->TraCIAPI::vehicle.getPosition(sID);
        double distance = sqrt(pow((egoPosXY.x-PosXY.x),2)+pow((egoPosXY.y-PosXY.y),2));
        dist += distance;
        if(distance > maxDist)
          maxDist = distance;

        if(!assocCVIDs.empty ())
          {
            std::vector<uint64_t> assocPMs;
            for(auto it = assocCVIDs.begin();it !=assocCVIDs.end ();it++)
              {
                if(std::find(assocPMs.begin (),assocPMs.end(),*it) == assocPMs.end())
                  assocPMs.push_back (*it);
              }
            assoc += assocPMs.size ();
          }
      }

    if(!POs.empty ())
      {
        conf = conf / POs.size();
        age = age / POs.size();
        assoc = assoc / POs.size();
        dist = dist / POs.size();
      }

    m_csv_file << Simulator::Now ().GetSeconds () << ","
               << m_card << ","
               << POs.size() << ","
               << conf << ","
               << age << ","
               << m_avg_dwell/1000 << ","
               << assoc << ","
               << CVs.size () << ","
               << dist << ","
               << maxDist << ","
               << std::endl;
    m_event_writeContents = Simulator::Schedule(MilliSeconds(LOG_FREQ),&LDM::writeAllContents,this);
  }

  void
  LDM::cleanup()
  {
    Simulator::Cancel(m_event_writeContents);
  }

  void
  LDM::executeOnAllContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args)
  {

    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
        oper_fcn(it->second.vehData,additional_args);
    }
  }

  void
  LDM::updatePolygons()
  {
    if(m_client == NULL)
      return;
    for (auto it = m_LDM.begin(); it != m_LDM.end(); ++it) {
        if (m_polygons)
        {
            std::string veh = std::to_string(it->second.vehData.stationID);
            //if(it->second.vehData.detected)
            if(it->second.vehData.stationID != m_stationID)
              drawPolygon(it->second.vehData);
        }
    }
    m_event_updatePolygons = Simulator::Schedule(MilliSeconds (50),&LDM::updatePolygons,this); // 20fps
  }

  void
  LDM::drawPolygon(vehicleData_t data)
  {
    if(m_client == NULL)
      return;
    using namespace boost::geometry::strategy::transform;
    libsumo::TraCIPosition SPos;
    double angle = 0.0;
    vehiclePoints_t Spoints;
    std::string id = std::to_string(data.stationID);

    //Get sensed points
    // Scale with vehicle size
    scale_transformer<double, 2, 2> scaleS((double) data.vehicleLength.getData ()/10,(double) data.vehicleWidth.getData()/10);
    boost::geometry::transform(boxCenterPoint, Spoints.center, scaleS);
    boost::geometry::transform(frontLeftPoint, Spoints.front_left,scaleS);
    boost::geometry::transform(frontRightPoint, Spoints.front_right,scaleS);
    boost::geometry::transform(backLeftPoint, Spoints.back_left, scaleS);
    boost::geometry::transform(backRightPoint, Spoints.back_right,scaleS);


    // Rotate
    angle = -1.0 * (90-data.heading);
    rotate_transformer<boost::geometry::degree, double, 2, 2> rotateS(angle);
    boost::geometry::transform(Spoints.center, Spoints.center, rotateS);
    boost::geometry::transform(Spoints.front_left, Spoints.front_left, rotateS);
    boost::geometry::transform(Spoints.front_right, Spoints.front_right, rotateS);
    boost::geometry::transform(Spoints.back_left, Spoints.back_left, rotateS);
    boost::geometry::transform(Spoints.back_right, Spoints.back_right, rotateS);

    //Translate to actual front bumper position
    SPos = m_client->TraCIAPI::simulation.convertLonLattoXY (data.lon,data.lat);
    translate_transformer<double, 2, 2> translateS(SPos.x,SPos.y);
    boost::geometry::transform(Spoints.center, Spoints.center, translateS);
    boost::geometry::transform(Spoints.front_left, Spoints.front_left, translateS);
    boost::geometry::transform(Spoints.front_right, Spoints.front_right, translateS);
    boost::geometry::transform(Spoints.back_left, Spoints.back_left, translateS);
    boost::geometry::transform(Spoints.back_right, Spoints.back_right, translateS);

    libsumo::TraCIPositionVector SUMOPolygon;
    libsumo::TraCIColor magenta;
    magenta.r=255;magenta.g=0;magenta.b=255;magenta.a=255;
    libsumo::TraCIColor magenta_cpm;
    magenta_cpm.r=0;magenta_cpm.g=214;magenta_cpm.b=0;magenta_cpm.a=225;
    libsumo::TraCIColor cian_connected;
    cian_connected.r=189;cian_connected.g=238;cian_connected.b=245;cian_connected.a=255;
    SUMOPolygon.push_back(boost2TraciPos (Spoints.front_left));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.back_left));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.back_right));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.front_right));
    SUMOPolygon.push_back(boost2TraciPos (Spoints.front_left));


    std::vector<std::string> polygonList;
    polygonList = m_client->TraCIAPI::polygon.getIDList ();
    if(std::find(polygonList.begin(), polygonList.end (), id) != polygonList.end ())
      {
        m_client->TraCIAPI::polygon.setShape (id,SUMOPolygon);
        if (data.detected)
          {
            long who = data.perceivedBy.getData();
            if (who == m_stationID)
              m_client->TraCIAPI::polygon.setColor (id,magenta);
            else
              m_client->TraCIAPI::polygon.setColor (id,magenta_cpm);
          }
      }
    else
      {
        if(data.detected)
          {
            long who = data.perceivedBy.getData();
            if (who == m_stationID)
              m_client->TraCIAPI::polygon.add(id,SUMOPolygon,magenta,1,"building.yes",5);
            else
              m_client->TraCIAPI::polygon.add(id,SUMOPolygon,magenta_cpm,1,"building.yes",5);
          }
      }
  }

  libsumo::TraCIPosition
  LDM::boost2TraciPos(point_type point_type)
  {
    libsumo::TraCIPosition retPos;
    retPos.x = boost::geometry::get<0>(point_type);
    retPos.y = boost::geometry::get<1>(point_type);
    retPos.z = 1.0;
    return retPos;
  }

  /* @VALERIO */
  void
  LDM::checkAreaOfRelevance ()
  {
    if (m_client == nullptr)
      return;

    m_AoR.clear();

    /* Compute egoVehicle position */
    libsumo::TraCIPosition egoPosXY = m_client->TraCIAPI::vehicle.getPosition(m_id);

    /* Scan the LDM to check which UEs are within the AoR */
    for (auto & it : m_LDM)
    {
      if(it.second.vehData.stationID != m_stationID) // Exclude this UE
      {
        std::string sID = "veh" + std::to_string(it.second.vehData.stationID);
        libsumo::TraCIPosition objPosXY = m_client->TraCIAPI::vehicle.getPosition(sID);

        if(sqrt(pow((egoPosXY.x-objPosXY.x),2) + pow((egoPosXY.y-objPosXY.y),2)) <= AoR_radius)
        {
          m_AoR.insert (m_AoR.end(), sID);
        }
      }
    }

    /* Fill the CSV file */
    m_csv_name_aor = "Results/AoR/AoR-" + m_id + ".csv";
    m_csv_ofstream_aor.open (m_csv_name_aor, std::ios::out);
    for (const auto& element : m_AoR) {
      m_csv_ofstream_aor << element << ",";
    }

    m_csv_ofstream_aor << std::endl;
    m_csv_ofstream_aor.close ();

    m_event_checkAreaOfRelevance = Simulator::Schedule(MilliSeconds(LOG_FREQ),&LDM::checkAreaOfRelevance,this);
  }

  /* @VALERIO */
  void
  LDM::checkAgeOfInformation ()
  {
    if (m_client == nullptr)
      return;

    /* Fill the CSV file */
    m_csv_name_aoi = "Results/AoI/AoI-" + m_id + ".csv";
    m_csv_ofstream_aoi.open (m_csv_name_aoi, std::ios::out);
    for (const auto& it : m_LDM) {
      m_csv_ofstream_aoi << it.second.vehData.timestamp_us << ",";
    }

    m_csv_ofstream_aoi << std::endl;
    m_csv_ofstream_aoi.close ();

    m_event_checkAgeOfInformation = Simulator::Schedule(MilliSeconds(LOG_FREQ),&LDM::checkAgeOfInformation,this);
  }
}
