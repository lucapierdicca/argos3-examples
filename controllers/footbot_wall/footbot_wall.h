/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_WALL_H
#define FOOTBOT_WALL_H

/*
 * Include some necessary headers.
 */

#include <argos3/core/control_interface/ci_controller.h>

// actuators
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_actuator.h>

//sensors
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

#include <experimental/random>

/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotWall : public CCI_Controller {

private:


   CCI_DifferentialSteeringActuator* m_pcWheels;
   CCI_FootBotDistanceScannerActuator* m_pcDistanceA;
   CCI_FootBotDistanceScannerSensor* m_pcDistanceS;
   CCI_FootBotProximitySensor* m_pcProximity;

   std::map<CRadians,Real> world_model_short;
   std::map<CRadians,Real> world_model_long;

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */

   /* Maximum tolerance for the angle between
    * the robot heading direction and
    * the closest obstacle detected. */
   CDegrees m_cAlpha;
   /* Maximum tolerance for the proximity reading between
    * the robot and the closest obstacle.
    * The proximity reading is 0 when nothing is detected
    * and grows exponentially to 1 when the obstacle is
    * touching the robot.
    */
   Real m_fDelta;
   Real m_fWheelVelocity;

   
   struct state_data {
      std::string behaviour;
      std::array<float,3> input;
      std::map<std::string, std::vector<std::string>> successors;
   } HalfTurn, ArcLeft, ArcRight, StraightOn, Braitenberg;



   

   CRadians chosen_direction;
   bool chosen = false;


   

   std::string primitive;
   std::map<std::string, struct state_data> FSA;
   
   std::array<int,4> occupancy;
   std::map<std::array<int,4>, std::string> occupancy_to_zone;

   int counter = 1, counter_threshold;
   Real desired_orientation;
   Real orientation_error;
   
   


public:

   std::vector<std::pair<CRadians,Real>> lmr;
   std::vector<std::pair<CRadians,Real>> pr;

   struct sector_data{
      std::array<CRadians,2> angle_interval;
      std::vector<std::pair<CRadians,Real>> readings;

   } R, F, H;

   std::map<char, struct sector_data> sectorLbl_to_sectorData;

   /* Class constructor. */
   CFootBotWall();

   /* Class destructor. */
   virtual ~CFootBotWall() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   std::pair<CRadians,Real> getMinReading(char sector_lbl);
   void getLocalMinReadings(char sector_lbl);
   std::vector<std::pair<CRadians,Real>> processReadings(char sector_lbl);




};

#endif
