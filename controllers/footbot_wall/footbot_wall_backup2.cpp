/* Include the controller definition */
#include "footbot_wall.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


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
   
   m_pcDistanceS  = GetSensor  <CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
   m_pcProximity  = GetSensor  <CCI_FootBotProximitySensor>("footbot_proximity");

   
   //Parse the configuration file
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   counter = 0, counter_threshold = 100;


}





void CFootBotWall::ControlStep() {
   
   CRadians r_min_angle, l_min_angle;
   Real r_min_distance = 200.0;
   Real l_min_distance = 200.0;
   Real ref_distance = 75.0;
   CVector2 centroid;
   int f = 0,r = 0,l = 0;
   Real l_speed , r_speed ;

   std::array <int,3> free_status = {0,0,0};
   


   const CCI_FootBotDistanceScannerSensor::TReadingsMap& long_readings = m_pcDistanceS->GetLongReadingsMap();
   for (const auto& [key, value] : long_readings) world_model_long[key] = value;
   

   struct sector_data{
      CVector2 centroid;
      Real length;
      Real angle;
   } F, R, L;
   
   std::map<char, struct sector_data> sector_to_data = {{'F', F}, 
                                                        {'R', R},
                                                        {'L', L}};
    

       
   // analyze the long world models for Braitenberg navigation and line following
   for (const auto& [angle, distance] : world_model_long){
      if (distance != -1 && distance != -2){
         if (angle > -(CRadians::PI_OVER_SIX + CRadians::PI_OVER_TWO) && angle < -CRadians::PI_OVER_THREE && distance <= r_min_distance){
            r_min_distance = distance;
            r_min_angle = angle;
         }
         if (angle > CRadians::PI_OVER_THREE && angle < (CRadians::PI_OVER_SIX + CRadians::PI_OVER_TWO) && distance <= l_min_distance){
            l_min_distance = distance;
            l_min_angle = angle;
         }
      }

      if(angle > -CRadians::PI_OVER_SIX/2.0 && angle < CRadians::PI_OVER_SIX/2.0){
         if(distance == -1) 
            sector_to_data['F'].centroid += CVector2(distance*0.0, angle);
         if(distance == -2)
            sector_to_data['F'].centroid += CVector2(distance*-100.0, angle);
         if(distance >= -1)
            sector_to_data['F'].centroid += CVector2(distance, angle);
         f++;
      }
      if(angle > -(CRadians::PI_OVER_SIX/2.0 + CRadians::PI_OVER_TWO) && angle < -(CRadians::PI_OVER_THREE + CRadians::PI_OVER_SIX/2.0)){
         if(distance == -1) 
            sector_to_data['R'].centroid += CVector2(distance*0.0, angle);
         if(distance == -2)
            sector_to_data['R'].centroid += CVector2(distance*-100.0, angle);
         if(distance >= -1)
            sector_to_data['R'].centroid += CVector2(distance, angle);
         r++;
      }
      if(angle > (CRadians::PI_OVER_THREE + CRadians::PI_OVER_SIX/2.0) && angle < (CRadians::PI_OVER_SIX/2.0 + CRadians::PI_OVER_TWO)){
         if(distance == -1) 
            sector_to_data['L'].centroid += CVector2(distance*0.0, angle);
         if(distance == -2)
            sector_to_data['L'].centroid += CVector2(distance*-100.0, angle);
         if(distance >= -1)
            sector_to_data['L'].centroid += CVector2(distance, angle);
         l++;
      }
   }
         

   sector_to_data['F'].centroid /= f;
   sector_to_data['R'].centroid /= r;
   sector_to_data['L'].centroid /= l;



   Real min_angle = CRadians::PI.GetValue();

   Real free_threshold = 160.0;
   
   

   for (const auto& [sector, sector_data] : sector_to_data){
      sector_to_data[sector].length = sector_data.centroid.Length();
      sector_to_data[sector].angle = sector_data.centroid.Angle().GetValue();

      if(sector == 'F' && sector_data.length >= free_threshold)
         free_status[0] = 1;
      if(sector == 'R' && sector_data.length >= free_threshold)
         free_status[1] = 1;
      if(sector == 'L' && sector_data.length >= free_threshold)
         free_status[2] = 1;

      std::cout << sector << ": " << sector_data.length << " - " << sector_data.angle << "  ";
      
   }

   std::cout << "\n";

   std::cout << free_status[0] << free_status[1] << free_status[2] << "\n";
   std::cout << counter << "\n";

   

   if(counter >= counter_threshold)
   {
      if (!manouvering)
      {
         std::cout << "RANDOOOOOM\n";
         std::experimental::reseed();
         std::vector <char> allowed_choices;
         
         if(free_status[0]==1)
            allowed_choices.push_back('F');
         if(free_status[1]==1)
            allowed_choices.push_back('R');
         if(free_status[2]==1)
            allowed_choices.push_back('L');

         int n = allowed_choices.size()-1;
         int k = std::experimental::randint(0, n);

         desired_orientation = sector_to_data[allowed_choices[k]].angle;


         manouvering = true;
         std::cout << "chosen_direction: " << allowed_choices[k] << "\n";
      }


      r_speed = 5.0;
      l_speed = 5.0;

      std::cout << "desired_orientation: " << desired_orientation << "\n ";

      Real orientation_error = abs(desired_orientation) - CRadians::ZERO.GetValue();

      std::cout << "orientation_error: "   << orientation_error << "\n ";
      
      if(desired_orientation > 0.0){
         //std::cout << "R: " << orie_error << std::endl;
         r_speed = r_speed + 5*orientation_error;
         l_speed = l_speed + -5*orientation_error;
      }
      else{
         //std::cout << "L: " << orie_error << std::endl;
         r_speed = r_speed + -5*orientation_error;
         l_speed = l_speed + 5*orientation_error;
      }

      if(orientation_error <= 0.0001){
         counter = 0;
         manouvering = false;
      }
   }
   

   else
   {
      
      if(free_status == std::array<int,3>{1,1,1} or free_status == std::array<int,3>{1,0,1} or free_status == std::array<int,3>{1,1,0} or free_status == std::array<int,3>{0,1,1})
         counter++;
      else
         counter = 0;

      for(const auto& [sector, data] : sector_to_data)
         if(abs(data.angle) <= min_angle)
            min_angle = data.angle;

      desired_orientation = min_angle;
      std::cout << "desired_orientation: "<<desired_orientation << "\n ";

      r_speed = 5.0;
      l_speed = 5.0;

      if(desired_orientation <= CRadians::PI.GetValue()){

         Real orientation_error = abs(desired_orientation) - CRadians::ZERO.GetValue();
         
         if(desired_orientation > 0.0){
            //std::cout << "R: " << orie_error << std::endl;
            r_speed = r_speed + 5*orientation_error;
         }
         else{
            //std::cout << "L: " << orie_error << std::endl;
            l_speed = l_speed + 5*orientation_error;
         }
      }


      r_speed = r_speed + 0.02*l_min_distance;
      l_speed = l_speed + 0.02*r_min_distance;
   }





   

   std::cout << "vLEFT: " << l_speed << "           vRIGHT: " << r_speed << "\n";
   m_pcWheels->SetLinearVelocity(l_speed, r_speed);

}







REGISTER_CONTROLLER(CFootBotWall, "footbot_wall_controller")
