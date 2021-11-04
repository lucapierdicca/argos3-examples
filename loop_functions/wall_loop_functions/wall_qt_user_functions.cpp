#include "wall_qt_user_functions.h"


/****************************************/
/****************************************/

CWALLQTUserFunctions::CWALLQTUserFunctions() {
   RegisterUserFunction<CWALLQTUserFunctions,CFootBotEntity>(&CWALLQTUserFunctions::Draw);
}

/****************************************/
/****************************************/



void CWALLQTUserFunctions::Draw(CFootBotEntity& c_entity) {


   
   controller = dynamic_cast<CFootBotWall&> (c_entity.GetControllableEntity().GetController());
   
   for (auto p : controller.pr){
      CVector2 end = CVector2(p.distance, p.angle);
      CRay3 ray = CRay3(CVector3(0.0, 0.0, 0.01) ,CVector3(end.GetX()*0.01, (end.GetY())*0.01, 0.01));
      DrawRay(ray);
      DrawText(CVector3(end.GetX()*0.01, (end.GetY())*0.01, 0.01), std::to_string(p.age));
   }
   for (auto min : controller.lmr){
      CVector2 end = CVector2(min.distance, min.angle);
      CRay3 ray = CRay3(CVector3(0.0, 0.0, 0.02),CVector3(end.GetX()*0.01, end.GetY()*0.01, 0.02));
      DrawRay(ray, CColor::BLACK, 2.0f);
   }
}


void CWALLQTUserFunctions::DrawInWorld() {
   Real x = 0.85;
   Real y = 0.0;
   Real dx = 0.03;
   Real l;
   bool in;
   
   for (auto p : controller.pr){
      in = false;
      CVector2 end = CVector2(p.distance, p.angle);
      l = end.Length();
      CRay3 ray = CRay3(CVector3(x, y, 0.01), CVector3(x, y+l*0.01, 0.01));
      for(auto min : controller.lmr)
         if(p.angle == min.angle)
            in = true;
      if (in)
         DrawRay(ray, CColor::BLACK, 2.0f);
      else
         DrawRay(ray);
      
      x = x+dx;
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CWALLQTUserFunctions, "wall_qt_user_functions")
