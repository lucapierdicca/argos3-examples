#include "wall_qt_user_functions.h"


/****************************************/
/****************************************/

CWALLQTUserFunctions::CWALLQTUserFunctions() {
   RegisterUserFunction<CWALLQTUserFunctions,CFootBotEntity>(&CWALLQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CWALLQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */

   
   CFootBotWall& controller = dynamic_cast<CFootBotWall&> (c_entity.GetControllableEntity().GetController());

   std::vector<std::pair<CRadians,Real>> local = controller.H.local_min_readings;
   


   CRay3 ray = CRay3(CVector3(0.0, 0.0, 0.3),CVector3(0.5,0.5,0.3));
   DrawRay(ray);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CWALLQTUserFunctions, "wall_qt_user_functions")
