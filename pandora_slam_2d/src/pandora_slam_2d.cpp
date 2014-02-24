  /** 
  * File Description slam - Contains tha main SLAM class, and the basic SLAM functionalities.
  * Contents:
		Slam::Slam
		Slam::mainMotorControlStatusAlert
		Slam::findTransformation
		Slam::pickRaysByScanParts
		Slam::fixNewScans
		Slam::updateMapColor
		Slam::updateMapColorMeans
		Slam::minimizeMap
		Slam::sendMap
		Slam::startTransition
		Slam::completeTransition
		Slam::updateMapColorHybrid
  * Author: SLAM team
  */

#include "pandora_slam_2d/pandora_slam_2d.h"

using namespace pandora_slam_2d;

PandoraSlam::PandoraSlam(int argc, char **argv):crsmSlam(argc,argv){
	
	state = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState = state_manager_communications::robotModeMsg::MODE_OFF;
	clientInitialize();

}


/**
	@brief Implemented from StateClient. Called when the robot state changes
	@param newState type: int
	@return void
	**/
void PandoraSlam::startTransition(int newState){

	state = newState;
	
	if(state==state_manager_communications::robotModeMsg::MODE_TERMINATING){
		ROS_ERROR("[Pandora SLAM] Terminating node");
		exit(0);
	}
	
	bool currStateOn = (state!=state_manager_communications::robotModeMsg::MODE_OFF);
	bool prevStateOn = (prevState!=state_manager_communications::robotModeMsg::MODE_OFF);
	
	if(currStateOn && !prevStateOn){
		crsmSlam.startLaserSubscriber();
		crsmSlam.startOGMPublisher();
		crsmSlam.startTrajectoryPublisher();
	}
	else if(!currStateOn && prevStateOn){
		crsmSlam.stopLaserSubscriber();
		crsmSlam.stopOGMPublisher();
		crsmSlam.stopTrajectoryPublisher();
	}
	
	prevState=state;
	
	transitionComplete(state);
}

/**
	@brief Implements the state transition end. Inherited from State client.
	@param void
	@return void
	**/
void PandoraSlam::completeTransition(void){
}

