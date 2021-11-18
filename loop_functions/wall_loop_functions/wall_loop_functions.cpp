#include "wall_loop_functions.h"

void CWallLoopFunctions::PostStep() {
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    CFootBotEntity* pcFB;

    for(CSpace::TMapPerType::iterator it = tFBMap.begin();it != tFBMap.end();++it)
        pcFB = any_cast<CFootBotEntity*>(it->second);

    controller = dynamic_cast<CFootBotWall&> (pcFB->GetControllableEntity().GetController());


    if (controller.tic % 10 == 0){
        MoveEntity(pcFB->GetEmbodiedEntity(), CVector3(0,0,0), CQuaternion());
    }


  
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CWallLoopFunctions, "wall_loop_functions")
