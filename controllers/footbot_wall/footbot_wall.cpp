/* Include the controller definition */
#include "footbot_wall.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

#include <algorithm>



CFootBotWall::CFootBotWall() : // initializer list
   m_pcWheels(NULL),
   m_pcDistanceS(NULL),
   m_pcDistanceA(NULL),
   m_pcProximity(NULL),
   m_pcRangeAndBearingS(NULL),
   m_pcPositioning(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f) {}


void CFootBotWall::Init(TConfigurationNode& t_node) {

   
   //Sensors & Actuators
   m_pcWheels     = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   
   m_pcDistanceA  = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");
   m_pcDistanceA->SetRPM(30.0);
   
   m_pcDistanceS  = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
   m_pcProximity  = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
   m_pcRangeAndBearingS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
   m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");

   
   //Parse the configuration file
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   
   R.angle_interval = {-CRadians::PI, CRadians::ZERO};
   F.angle_interval = {-CRadians::PI_OVER_SIX/2.0, CRadians::PI_OVER_SIX/2.0};
   H.angle_interval = {-CRadians::PI, CRadians::PI};
   

   sectorLbl_to_sectorData = {{'R', R},
                              {'F', F},
                              {'H', H}};

   std::vector<std::pair<CRadians,Real>> local_min_readings;
}


bool isOpen(Real distance){
   if(distance >= 140.0)
      return true;
   else
      return false;
}


void CFootBotWall::getLocalMinMaxReadings(){
   
   // find local mins
   Real l ,c, r, ll, rr, lll, rrr; // sliding window |l|c|r|

   for(int i = 0;i<pr.size()-4;i++){
      ll = pr[i].distance;
      l = pr[i+1].distance;
      c = pr[i+2].distance;
      r = pr[i+3].distance;
      rr = pr[i+4].distance;

      if(ll>l && l>c && c<r && r<rr)
         lmr_new.push_back(pr[i+2]);
   }

   //find local maxs (dalle aperture)
   CRadians start, end; 
   int start_index, end_index, first_start_index, first_end_index;

   CRadians current_angle;
   Real current_distance;

   bool started = false;

   for(int i=0;i<pr.size();i++){
      current_angle = pr[i].angle;
      current_distance = pr[i].distance;

      if (started == false){ 
         if (isOpen(current_distance)){
            start = current_angle;
            start_index = i;
            end = start;
            end_index = start_index;
            started = true;
         }
      }

      if (started == true){
         if (isOpen(current_distance)){
            end = current_angle;
            end_index = i;
         }
         else{
            if (start_index == 0){
               first_start_index = start_index;
               first_end_index = end_index; //copy the first end index
            }

            lMr.push_back(pr[int(start_index + (end_index-start_index)/2.0)]);
            started = false;
         }
      }
   }

   if (started == true)
      if (first_start_index != 0)
         lMr.push_back(pr[int(start_index + (end_index-start_index)/2.0)]);
      else{
         lMr.erase(lMr.begin());
         lMr.push_back(pr[int(start_index + (end_index+first_end_index-start_index)/2.0) % pr.size()]);
      }
}



void CFootBotWall::processReadings(char sector_lbl){
   
   pr = sectorLbl_to_sectorData[sector_lbl].readings;

   Real avg;
   int window_len = 5;
   int readings_len = sectorLbl_to_sectorData[sector_lbl].readings.size();
   int j = 0;

   for(int i = 0;i<readings_len;i++){
      avg = 0.0f;
      j = 0;
      for(j;j<window_len;j++)
         avg = avg + sectorLbl_to_sectorData[sector_lbl].readings[(i+j) % readings_len].distance;
   
      avg = avg / window_len;

      pr[(i+int((window_len-1)/2)) % readings_len].distance = avg;
   }
}



std::pair<CRadians,Real> CFootBotWall::getMinReading(char sector_lbl){
   Real distance;
   std::pair <CRadians, Real> min_reading (0,150);
   
   for(int i = 0;i< sectorLbl_to_sectorData[sector_lbl].readings.size();i++){
      if(sectorLbl_to_sectorData[sector_lbl].readings[i].distance < min_reading.second){
         min_reading.second = sectorLbl_to_sectorData[sector_lbl].readings[i].distance;
         min_reading.first = sectorLbl_to_sectorData[sector_lbl].readings[i].angle;
      }
   }

   return min_reading;
}


void CFootBotWall::getZoneLabel(CVector3 position){
   int min_i = 0;
   int max_i = 0;
   
   zone_data current_zone = {position,""};

   if(lmr_new.size() > 0 and lMr.size() > 0){
      for (int i = 0; i < lmr_new.size() + lMr.size(); i++){
         if (min_i < lmr_new.size() and max_i < lMr.size()){
            if(lmr_new[min_i].angle < lMr[max_i].angle){
               current_zone.label.append("m");
               min_i++;
            }
            else if(lmr_new[min_i].angle > lMr[max_i].angle){
               current_zone.label.append("M");
               max_i++;
            }
         }
         else if(min_i == lmr_new.size()){
            current_zone.label.append("M");
            max_i++;
         }
         else{
            current_zone.label.append("m");
            min_i++;
         }
      }
   }

   zone_trajectory.push_back(current_zone);
}





void CFootBotWall::ControlStep() {

   // reset sectors_data for the next control step
   for (const auto& [sectorLbl, sectorData] : sectorLbl_to_sectorData)
      sectorLbl_to_sectorData[sectorLbl].readings.clear();
   

   // get the current step readings
   const CCI_FootBotDistanceScannerSensor::TReadingsMap& long_readings = m_pcDistanceS->GetLongReadingsMap();
   const CCI_RangeAndBearingSensor::TReadings& rab_readings = m_pcRangeAndBearingS->GetReadings();
   const CCI_PositioningSensor::SReading& robot_state = m_pcPositioning->GetReading();

   
   // add the current step readings in the map world_model_long (it starts empty then it grows then it stops)
   Real mod_distance;
   for (const auto& [angle, distance] : long_readings){
      mod_distance = distance;
      if (mod_distance == -1) mod_distance = 15.0f;
      if (mod_distance == -2) mod_distance = 150.0f;
      angle_data a = {angle,mod_distance,tic};
      world_model_long[angle] = a;
   }



   // add the readings to the opportune sector based on the sector angle_interval
   for (const auto& [angle, angleData] : world_model_long){
      for (const auto& [sectorLbl, sectorData] : sectorLbl_to_sectorData){
         if (angle >= sectorData.angle_interval[0] && angle <= sectorData.angle_interval[1]){
            //std::pair<CRadians,Real> p (angle, distance);
            sectorLbl_to_sectorData[sectorLbl].readings.push_back(angleData);
         }
      }
   }



   
   if (counter == 10){
      counter=0;
      lmr_new.clear();
      lMr.clear();
      pr.clear();
      processReadings('H'); 
      getLocalMinMaxReadings();

      std::cout << "ID: " << GetId() << "\n";

      std::cout << "MIN: " << lmr_new.size() << "\n";
      for(auto l : lmr_new)
         std::cout << l.angle << " " << l.distance << " " << l.age << "\n";

      std::cout << "MAX: " << lMr.size() << "\n";
      for(auto l : lMr)
         std::cout << l.angle << " " << l.distance << " " << l.age << "\n";

      std::cout << "RAB: " << rab_readings.size() << "\n"; 
      for(auto r : rab_readings)
         std::cout << r.HorizontalBearing << " " << r.Range << "\n";


      getZoneLabel(robot_state.Position);
      
      std::cout << "*******************************\n";

      lmr_old_copy = lmr_old;

      lmr_old = lmr_new;

      
   }

   counter++;
   tic++;
  
   



   

   Real v_l, v_r, v_l_def, v_r_def, v_l_dis, v_r_dis, v_l_ori, v_r_ori;
   Real r_distance_d = 35; // desired distance to the wall [cm]
   CRadians r_orientation_d = -CRadians::PI_OVER_TWO; // desired orientation wrt the wall [rad]
   Real distance_error, orientation_error;
   
   // default component
   v_r_def = 6.0;
   v_l_def = 6.0;

   // distance error component
   distance_error = r_distance_d - getMinReading('R').second;

   if(distance_error > 0.0){
      v_r_dis = 0.1*abs(distance_error);
      v_l_dis = -0.1*abs(distance_error);
   }
   else{
      v_r_dis = -0.1*abs(distance_error);
      v_l_dis = 0.1*abs(distance_error);
   }


   // orientation error component
   orientation_error = (r_orientation_d - getMinReading('R').first).SignedNormalize().GetValue();

   if(orientation_error <= CRadians::PI.GetValue()){
      if(orientation_error > 0.0){
         //std::cout << "R: " << orie_error << std::endl;
         v_r_ori = -2*abs(orientation_error);
         v_l_ori = 2*abs(orientation_error);
      }
      else{
         //std::cout << "L: " << orie_error << std::endl;
         v_r_ori = 2*abs(orientation_error);
         v_l_ori = -2*abs(orientation_error);
      }
   }

   v_r = v_r_def + v_r_dis + v_r_ori;
   v_l = v_l_def + v_l_dis + v_l_ori;


   m_pcWheels->SetLinearVelocity(v_l, v_r);

  





   /*
   std::cout << open_intervals.size() << "\n";

      

   Real distance_error, orientation_error;

   if(open_intervals.size() > 1 && !chosen && sector_to_data['R'].angle <= -CRadians::PI_OVER_TWO){

      // compute "corner orientation"
      Real corner_distance = 155.0;
      CRadians corner_orientation;
      for (const auto& [angle, distance] : world_model_long)
         if(angle > open_intervals[0][1] && angle < open_intervals[1][0])
            if (distance <= corner_distance){
               corner_distance = distance;
               corner_orientation = angle;
            }
      

      std::array<CRadians,2> possible_orientation = {open_intervals[0][0] + (open_intervals[0][1] - open_intervals[0][0]).SignedNormalize()/2.0,
                                                     corner_orientation};


      std::experimental::reseed();
      int index = std::experimental::randint(0, 1);                  
      chosen_direction = possible_orientation[index];

      std::cout << possible_orientation[0] << " - " << possible_orientation[1] << std::endl; 

      Real current_min = CRadians::TWO_PI.GetValue();
      int current_min_index;
      for(int i=0;i<possible_orientation.size();i++)
         if(abs((-CRadians::PI_OVER_TWO-possible_orientation[i]).SignedNormalize().GetValue()) < current_min){
            current_min_index = i;
            current_min = abs((-CRadians::PI_OVER_TWO-possible_orientation[i]).SignedNormalize().GetValue());
         }

      if (index != current_min_index)
         chosen = true;
   }

   if(chosen){

      std::cout << "F COMMANDING\n";
      
      if(sector_to_data['R'].angle > -CRadians::PI_OVER_TWO) chosen = false;
      

      // default component
      r_speed = 5.0;
      l_speed = 5.0;

      // orientation error component
      orientation_error = (chosen_direction - sector_to_data['F'].angle).SignedNormalize().GetValue();
      std::cout << "R_angle: " << sector_to_data['R'].angle << " - F_angle: " << sector_to_data['F'].angle << std::endl;

      if(orientation_error > 0.0){
         //std::cout << "R: " << orie_error << std::endl;
         r_speed += abs(orientation_error);
         l_speed += -1*abs(orientation_error);
      }
      else{
         //std::cout << "L: " << orie_error << std::endl;
         r_speed += -1*abs(orientation_error);
         l_speed += abs(orientation_error);
      }

   }
   else{

      

      std::cout << "R COMMANDING\n";

      // default component
      r_speed = 5.0;
      l_speed = 5.0;


      // distance error component
      distance_error = r_distance_d - sector_to_data['R'].distance;

      if(distance_error > 0.0){
         r_speed += 0.1*abs(distance_error);
         l_speed += -0.1*abs(distance_error);
      }
      else{
         r_speed += -0.1*abs(distance_error);
         l_speed += 0.1*abs(distance_error);
      }


      // orientation error component
      orientation_error = (r_orientation_d - sector_to_data['R'].angle).SignedNormalize().GetValue();

      if(orientation_error <= CRadians::PI.GetValue()){
         if(orientation_error > 0.0){
            //std::cout << "R: " << orie_error << std::endl;
            r_speed += -5*abs(orientation_error);
            l_speed += 5*abs(orientation_error);
         }
         else{
            //std::cout << "L: " << orie_error << std::endl;
            r_speed += 5*abs(orientation_error);
            l_speed += -5*abs(orientation_error);
         }
      }
   }


   
   std::cout << "distance_error: "<< distance_error << "\n ";
   std::cout << "orientation_error: "<< orientation_error << "\n ";
   std::cout << "l_speed: " << l_speed << " - r_speed: " << r_speed << "\n";
   
   m_pcWheels->SetLinearVelocity(l_speed, r_speed);

   */

}







REGISTER_CONTROLLER(CFootBotWall, "footbot_wall_controller")
