#include "wall_loop_functions.h"



void CWallLoopFunctions::Init(TConfigurationNode& t_tree){

    // CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    
    // for(CSpace::TMapPerType::iterator it = tFBMap.begin();it != tFBMap.end();++it)
    //     pcFB = any_cast<CFootBotEntity*>(it->second);

    pcFB = dynamic_cast<CFootBotEntity&> (GetSpace().GetEntity("fb_0"));
     
}


void CWallLoopFunctions::PreStep() {


    if (GetSpace().GetSimulationClock() % 10 == 0 && GetSpace().GetSimulationClock() != 0){

        std::cout << "SPOSTA" << "\n";

        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<float> distr(0, 1);

        float x = (-4+0.35) + distr(eng)*(8-0.35*2);
        float y = (2+0.35) + distr(eng)*(1.5-0.35*2);
        //MoveEntity(pcFB.GetEmbodiedEntity(), CVector3(x,y,0), CQuaternion());

    }


  
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CWallLoopFunctions, "wall_loop_functions")
