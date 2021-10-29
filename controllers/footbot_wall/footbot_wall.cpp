/* Include the controller definition */
#include "footbot_wall.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>



/****************************************/
/****************************************/

CFootBotWall::CFootBotWall() : // initializer list
   m_pcWheels(NULL),
   m_pcDistanceS(NULL),
   m_pcDistanceA(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CFootBotWall::Init(TConfigurationNode& t_node) {

   
   //Sensors & Actuators
   m_pcWheels     = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   
   m_pcDistanceA  = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");
   m_pcDistanceA->SetRPM(30.0);
   
   m_pcDistanceS  = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
   m_pcProximity  = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

   
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


bool isClosed(Real distance){
   if(distance != -2)
      return true;
   else
      return false;
}


void CFootBotWall::getLocalMinReadings(char sector_lbl){
   
   std::vector<std::pair<CRadians,Real>> local_min_readings;

   auto processed_readings = processReadings(sector_lbl);

   pr = processed_readings;

   for(int i = 0;i<processed_readings.size()-4;i++){
      Real ll, l ,c, r, rr; // sliding window |l|c|r|
      ll = processed_readings[i].second;
      l = processed_readings[i+1].second;
      c = processed_readings[i+2].second;
      r = processed_readings[i+3].second;
      rr = processed_readings[i+4].second;

      if(ll>l && l>c && c<r && r<rr){
         //local_min_readings.push_back(processed_readings[i+2]);
         lmr.push_back(processed_readings[i+2]);
         std::cout << ll-l << " " << l-c << " " << r-c << " " << rr-r << "\n";
      }

   }
}

std::pair<CRadians,Real> CFootBotWall::getMinReading(char sector_lbl){
   Real distance;
   std::pair <CRadians, Real> min_reading (0,150);
   
   for(int i = 0;i< sectorLbl_to_sectorData[sector_lbl].readings.size();i++){
      if(sectorLbl_to_sectorData[sector_lbl].readings[i].second < min_reading.second){
         min_reading.second = sectorLbl_to_sectorData[sector_lbl].readings[i].second;;
         min_reading.first = sectorLbl_to_sectorData[sector_lbl].readings[i].first;
      }
   }

   return min_reading;
}

std::vector<std::pair<CRadians,Real>> CFootBotWall::processReadings(char sector_lbl){
   
   std::vector<std::pair<CRadians,Real>> processed_readings;
   for (auto r : sectorLbl_to_sectorData[sector_lbl].readings)
      processed_readings.push_back(r);

   Real avg;
   int window_len = 7;
   int readings_len = sectorLbl_to_sectorData[sector_lbl].readings.size();
   int j = 0;

   for(int i = 0;i<readings_len;i++){
      avg = 0.0f;
      j = 0;
      for(j;j<window_len;j++)
         avg = avg + sectorLbl_to_sectorData[sector_lbl].readings[(i+j) % readings_len].second;
   
      avg = avg / window_len;

      processed_readings[(i+int((window_len-1)/2)) % readings_len].second = avg;
   }

   return processed_readings;
}





void CFootBotWall::ControlStep() {

   // reset sectors_data for the next control step
   for (const auto& [sectorLbl, sectorData] : sectorLbl_to_sectorData){
      sectorLbl_to_sectorData[sectorLbl].readings.clear();
   }

   
   
   // get the current step readings
   const CCI_FootBotDistanceScannerSensor::TReadingsMap& long_readings = m_pcDistanceS->GetLongReadingsMap();

   
   // add the current step readings in the map world_model_long (it starts empty then it grows then it stops)
   Real mod_distance;
   for (const auto& [angle, distance] : long_readings){
      mod_distance = distance;
      if (mod_distance == -1) mod_distance = 15.0f;
      if (mod_distance == -2) mod_distance = 150.0f;
      world_model_long[angle] = mod_distance;
   }

   std::cout << world_model_long.size() << "\n";

   // add the readings to the opportune sector based on the sector angle_interval
   for (const auto& [angle, distance] : world_model_long){
      for (const auto& [sectorLbl, sectorData] : sectorLbl_to_sectorData){
         if (angle >= sectorData.angle_interval[0] && angle <= sectorData.angle_interval[1]){
            //std::pair<CRadians,Real> p (angle, distance);
            sectorLbl_to_sectorData[sectorLbl].readings.push_back({angle,distance});
         }
      }
   }


   
   if(counter == 10){
      counter = 0;
      lmr.clear();
      pr.clear();
      getLocalMinReadings('H');
   }
   counter++;

   std::cout << "----------------------" << lmr.size() <<" min readings\n";
   for(auto l : lmr)
      std::cout << l.first << " " << l.second << "\n";
   
   std::cout << "*******************************\n";
   

   Real l_speed, r_speed;
   Real r_distance_d = 35; // desired distance to the wall [cm]
   CRadians r_orientation_d = -CRadians::PI_OVER_TWO; // desired orientation wrt the wall [rad]
   Real distance_error, orientation_error;
   
   // default component
   r_speed = 5.0;
   l_speed = 5.0;

   // distance error component
   distance_error = r_distance_d - getMinReading('R').second;

   if(distance_error > 0.0){
      r_speed += 0.1*abs(distance_error);
      l_speed += -0.1*abs(distance_error);
   }
   else{
      r_speed += -0.1*abs(distance_error);
      l_speed += 0.1*abs(distance_error);
   }


   // orientation error component
   orientation_error = (r_orientation_d - getMinReading('R').first).SignedNormalize().GetValue();

   if(orientation_error <= CRadians::PI.GetValue()){
      if(orientation_error > 0.0){
         //std::cout << "R: " << orie_error << std::endl;
         r_speed += -2*abs(orientation_error);
         l_speed += 2*abs(orientation_error);
      }
      else{
         //std::cout << "L: " << orie_error << std::endl;
         r_speed += 2*abs(orientation_error);
         l_speed += -2*abs(orientation_error);
      }
   }
   

   m_pcWheels->SetLinearVelocity(l_speed, r_speed);





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
