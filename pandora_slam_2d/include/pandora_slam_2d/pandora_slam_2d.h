  /** 
  * File Description slam (header file) - Contains tha main SLAM class, and the basic SLAM functionalities.
  * Contents:
		enum ROBOT_STATES
		class Slam
  * Author: SLAM team
  */

#ifndef PANDORA_SLAM_H
#define PANDORA_SLAM_H

#include "state_client.h"
#include <crsm_slam/crsm_slam.h>

namespace pandora_slam_2d {

/**
 @class Slam
 @brief The main slam class. Contains the main functionalities of slam.
 **/ 
class PandoraSlam : public StateClient {
private:

	int state;
	int prevState;
	
	crsm_slam::CrsmSlam crsmSlam;
	
public:	
	/**
	@brief Default costructor
	@param argc [int] The number of input arguments
	@param argv [char **] The input arguments
	@return void
	**/
	PandoraSlam(int argc, char **argv);

	/**
	@brief Implemented from StateClient. Called when the robot state changes
	@param newState type: int
	@return void
	**/
	void startTransition(int newState);
	
	/**
	@brief Implements the state transition end. Inherited from State client.
	@return void
	**/
	void completeTransition(void);

};

}


#endif

