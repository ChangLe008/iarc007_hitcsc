#include <fstream>
#include <sstream>
#include <iostream>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/dji_sdk.h>
#include "iarc/pilot/control.h"
#include "iarc/obstacle.h"
#include "iarc/vehicle_pos.h"
#include "iarc/pilot/subscribe.h"
#include "iarc/pilot/pid_controller.h"
#include <ros/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iarc/pilot/XU_LADRC.h>
#include "iarc/pilot/target_processor.h"

#include <vector>

using namespace std;
using namespace DJI::onboardSDK;


/****
This file should include two function 
1. deciding whether to interact or not
2. to generate and send a serial of command [roll,pitch,yaw',z']
*****/

// RVIZ flags and variables begins



// RVIZ flags and variables ends





//////////////////////////////////////////////////////////////////////////////////////////
void control()
{


}
