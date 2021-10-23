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

   //Perception map
   occupancy_to_zone = {{{1,0,0},"Co"},
                        {{0,0,0},"Ci"},
                        {{1,1,1},"In"},
                        {{0,1,1},"In"},
                        {{1,1,0},"In"},
                        {{1,0,1},"In"}};


   //Action fsa                                                          
   HalfTurn.behaviour = "manouver";
   HalfTurn.input = {5.0,-5.0,60.0};
   HalfTurn.successors = { {"Co",{{"Braitenberg"}}} };

   ArcLeft.behaviour = "manouver";
   ArcLeft.input = {-5.0,5.0,160.0};
   ArcLeft.successors = { {"Co",{{"Braitenberg"}}} };

   ArcRight.behaviour = "manouver";
   ArcRight.input = {5.0,-5.0,160.0};
   ArcRight.successors = { {"Co",{{"Braitenberg"}}} };

   StraightOn.behaviour = "manouver";
   StraightOn.input = {5.0,5.0,15.0};
   StraightOn.successors = { {"Co",{{"Braitenberg"}}},
                             {"In", {{"HalfTurn"},{"ArcLeft"},{"ArcRight"},{"StraightOn"}}},
                             {"Ci", {{"HalfTurn"}}}};

   Braitenberg.behaviour = "reactive";
   Braitenberg.input = {0.0,0.0,0.0};
   Braitenberg.successors = { {"Co", {{"Braitenberg"}}},
                              {"In", {{"HalfTurn"},{"ArcLeft"},{"ArcRight"},{"StraightOn"}}},
                              {"Ci", {{"HalfTurn"}}} };                                                   

   FSA = {{"HalfTurn",     HalfTurn},
         {"ArcLeft",      ArcLeft},
         {"ArcRight",     ArcRight},
         {"StraightOn",   StraightOn},
         {"Braitenberg",  Braitenberg}};

   //init 
   primitive = "StraightOn";
   //occupancy = {1,0,0,0}; //FRBL


}



int isFree(std::vector<CVector2> rayscoordinates){

   CVector2 centroid;
   Real threshold = 150;
   Real length = 0.0;
   
   for(int i=0;i<rayscoordinates.size();i++)
      centroid += rayscoordinates[i];

   centroid /= rayscoordinates.size();

   //std::cout << free_rays << "/" << distances.size() << "\n";

   length = centroid.Length();

   std::cout << length << "\n";

   if(length >= threshold)
      return 1;
   else
      return 0;

}

void getOccupancy(std::map<CRadians,Real>& angle_to_distance, std::array<int,4>& occupancy){

   std::map <char, std::vector<CVector2>> sector_to_rayscoordinates;
   char sectorr;
   
   for (const auto& [angle, distance] : angle_to_distance){
      if(angle > -CRadians::PI_OVER_SIX && angle < CRadians::PI_OVER_SIX) 
         sectorr = 'F';
      if(angle > -(CRadians::PI_OVER_SIX + CRadians::PI_OVER_TWO) && angle < -CRadians::PI_OVER_THREE) 
         sectorr = 'R';
      if(abs(angle.GetValue()) > 150.0/CRadians::RADIANS_TO_DEGREES) 
         sectorr = 'B';
      if(angle > CRadians::PI_OVER_THREE && angle < (CRadians::PI_OVER_SIX + CRadians::PI_OVER_TWO)) 
         sectorr = 'L';

   
      if(distance == -1) 
         sector_to_rayscoordinates[sectorr].push_back(CVector2(distance*0.0, angle));
      if(distance == -2)
         sector_to_rayscoordinates[sectorr].push_back(CVector2(distance*-100.0, angle));
      if(distance >= -1)
         sector_to_rayscoordinates[sectorr].push_back(CVector2(distance, angle));
   }


   
   for (const auto& [sector, rayscoordinates] : sector_to_rayscoordinates){
      std::cout << sector << "\n";
      if(sector == 'F')
         occupancy[0] = isFree(rayscoordinates);
      if(sector == 'R')
         occupancy[1] = isFree(rayscoordinates);
      if(sector == 'B')
         occupancy[2] = isFree(rayscoordinates);
      if(sector == 'L')
         occupancy[3] = isFree(rayscoordinates);
      
   }

}


void CFootBotWall::ControlStep() {

//    //sense
   
//    //const CCI_FootBotDistanceScannerSensor::TReadingsMap& short_readings = m_pcDistanceS->GetShortReadingsMap();
//    //const CCI_FootBotProximitySensor::TReadings& proximity_reads = m_pcProximity->GetReadings();

//    // populate the long and short world models
   
//    //for (const auto& [key, value] : short_readings) world_model_short[key] = value;

   
   CRadians r_min_angle, l_min_angle;
   Real r_min_distance = 200.0;
   Real l_min_distance = 200.0;
   Real ref_distance = 75.0;
   CVector2 centroid;
   int k = 0;


   Real l_speed = 5.0, r_speed = 5.0;
   Real orientation_error;

   const CCI_FootBotDistanceScannerSensor::TReadingsMap& long_readings = m_pcDistanceS->GetLongReadingsMap();
   for (const auto& [key, value] : long_readings) world_model_long[key] = value;
   
   std::string behaviour = FSA[primitive].behaviour;
   
    
    if(behaviour == "reactive"){
      
       if(primitive == "Braitenberg"){
       
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

            if(angle > -CRadians::PI_OVER_SIX && angle < CRadians::PI_OVER_SIX){
               if(distance == -1) 
                  centroid += CVector2(distance*0.0, angle);
               if(distance == -2)
                  centroid += CVector2(distance*-100.0, angle);
               if(distance >= -1)
                  centroid += CVector2(distance, angle);
               k++;
            }

         }

         centroid /= k;

         if(centroid.GetX() <= 200 && centroid.GetY() <= 200){
            CRadians desired_orientation = centroid.Angle();

            //std::cout << "Desired: " << desired_orientation << std::endl;

            Real orie_error = abs(desired_orientation.GetValue()) - CRadians::ZERO.GetValue();
            
            if(desired_orientation.GetValue() > 0.0){
               //std::cout << "R: " << orie_error << std::endl;
               r_speed += 10*abs(orie_error);
            }
            else{
               //std::cout << "L: " << orie_error << std::endl;
               l_speed += 10*abs(orie_error);
            }
         }



         l_speed = l_speed + 0.01*r_min_distance;
         r_speed = r_speed + 0.01*l_min_distance;

      
       }

    }
    else{
      
       l_speed = FSA[primitive].input[0];
       r_speed = FSA[primitive].input[1];
    }

    std::string old_primitive = primitive;
   
    getOccupancy(world_model_long, occupancy);

    std::cout << "F: " << occupancy[0] << "R: " << occupancy[1] << "B: " << occupancy[2] << "L: " << occupancy[3] << std::endl;

    
   
    std::string zone = "";
    std::vector<std::string> successors;
    auto search = occupancy_to_zone.find(occupancy);
    if (search != occupancy_to_zone.end()){
       zone = occupancy_to_zone[occupancy];
       successors = FSA[primitive].successors[zone];
    }
   
   
    int s = successors.size();

    if(tick < FSA[primitive].input[2] && s == 0){
       primitive = primitive;
       tick++;
    }
    else{

       if(s > 1){
          std::cout << "RANDOM\n";
          std::experimental::reseed();

          std::vector <std::string> allowed_successors;
          for(int i=0;i<occupancy.size();i++)
             if(occupancy[i]==1 && successors[i] != "HalfTurn")
                allowed_successors.push_back(successors[i]);

          int n = allowed_successors.size()-1;
         
          std::cout << "eee" <<n;

          int k = std::experimental::randint(0, n);

          primitive = allowed_successors[k];
          tick = 0.0;

       }
       if(s == 1){
          primitive = successors[0];
          tick = 0.0;
       }
       if(s == 0)
       {
          primitive = "StraightOn";
          tick = 0.0;
       }
    }
   
    std::cout << old_primitive << "--" << zone << "-->" << primitive << tick << "\n";
    std::cout << "vLEFT: " << l_speed << "           vRIGHT: " << r_speed << "\n";
    m_pcWheels->SetLinearVelocity(l_speed, r_speed);

}







REGISTER_CONTROLLER(CFootBotWall, "footbot_wall_controller")
