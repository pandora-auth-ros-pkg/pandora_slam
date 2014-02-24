#include "pandora_icp_slam_3d_laser_map/pandora_icp_slam_3d_laser_map.h"

namespace pandora_slam_3d
{
	
PandoraIcpSlam3d::PandoraIcpSlam3d()
{	
	_currentTransform.setOrigin( tf::Vector3(0, 0, 0) );
	_currentTransform.setBasis( tf::Matrix3x3(1, 0, 0,
										     0, 1, 0,
											 0, 0, 1) );
											 
	_transformBroadcastTimer = _nodeHandle.createTimer(ros::Duration(0.0001), 
	                                                   &PandoraIcpSlam3d::transformBroadcastTimerCallback,this);	
	                                                   
	
	ros::Duration(1).sleep();
	
    try{
      _tfListener.lookupTransform("/laser", "/base_footprint",
                               ros::Time(0), _laserToBaseFootprintTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    	
	_transformBroadcastTimer.start();						  
		  
	
	_inputCloudSubscriber = _nodeHandle.subscribe("/laser/point_cloud", 1, 
													  &PandoraIcpSlam3d::inputCloudCallback,this);
													  
	_octomapCloudSubscriber = _nodeHandle.subscribe("/octomap_point_cloud_centers", 1, 
													  &PandoraIcpSlam3d::octomapCloudCallback,this);
													  
	_transformedCloudPubliser = _nodeHandle.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);
}

PandoraIcpSlam3d::~PandoraIcpSlam3d()
{
}

void PandoraIcpSlam3d::transformBroadcastTimerCallback(const ros::TimerEvent&)
{
	_tfbroadcaster.sendTransform(tf::StampedTransform(_currentTransform, ros::Time::now(), "map", "base_footprint"));
}

void PandoraIcpSlam3d::inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{	
	if(_octomapCloud.size()==0)
	{
		sensor_msgs::PointCloud2 PointCloudMsg;
		PointCloudMsg = *msg;
		PointCloudMsg.header.stamp = ros::Time::now();
		_transformedCloudPubliser.publish(PointCloudMsg);
		
		return;
	}
	
	boost::lock_guard<boost::mutex> lock(_octomapCloudMutex);
	
	std::cout << "ICP begun " << ros::Time::now() << std::endl;
	
		
	PointCloud pointCloud;
	pcl::fromROSMsg(*msg,pointCloud);
	
	std::vector< int > 	index;

	PointCloudPtr inputCloudPtr (new PointCloud);
	
	pcl::removeNaNFromPointCloud(pointCloud,*inputCloudPtr,index);
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	
	icp.setInputSource(inputCloudPtr);
	
	tf::StampedTransform laserTransform;
    try{
      _tfListener.lookupTransform("/laser", "/map",
                               ros::Time(0), laserTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
	
	tf::Matrix3x3 laserBasis = laserTransform.getBasis();
	
	tf::Vector3 laserOrigin = laserTransform.getOrigin();
	
	Eigen::Matrix4f transformMatrix;
	transformMatrix(0,0) = laserBasis.getRow(0)[0];
	transformMatrix(0,1) = laserBasis.getRow(0)[1];
	transformMatrix(0,2) = laserBasis.getRow(0)[2];
	
	transformMatrix(1,0) = laserBasis.getRow(1)[0];
	transformMatrix(1,1) = laserBasis.getRow(1)[1];
	transformMatrix(1,2) = laserBasis.getRow(1)[2];
	
	transformMatrix(2,0) = laserBasis.getRow(2)[0];
	transformMatrix(2,1) = laserBasis.getRow(2)[1];
	transformMatrix(2,2) = laserBasis.getRow(2)[2];
	
	transformMatrix(0,3) = laserOrigin[0];
	transformMatrix(1,3) = laserOrigin[1];
	transformMatrix(2,3) = laserOrigin[2];
	
	transformMatrix(3,0) = 0;
	transformMatrix(3,1) = 0;
	transformMatrix(3,2) = 0;
	transformMatrix(3,3) = 1;
	
	pcl::transformPointCloud (_octomapCloud, pointCloud, transformMatrix);
	
	PointCloud transformedOctomapCloud;
	
	index.clear();
	
	pcl::removeNaNFromPointCloud(pointCloud,transformedOctomapCloud,index);
	
	icp.setInputTarget(boost::make_shared<PointCloud>(transformedOctomapCloud));
	

	PointCloud finalCloud;
	icp.align(finalCloud);
	
	Eigen::Matrix4f transformationMatrix;
	transformationMatrix = icp.getFinalTransformation();
	
	tf::Transform movementTransform;
	
	movementTransform.setBasis(tf::Matrix3x3 (transformationMatrix(0,0),transformationMatrix(0,1),transformationMatrix(0,2),
									  transformationMatrix(1,0),transformationMatrix(1,1),transformationMatrix(1,2),
									  transformationMatrix(2,0),transformationMatrix(2,1),transformationMatrix(2,2)));
									  
	movementTransform.setOrigin(tf::Vector3(transformationMatrix(0,3),transformationMatrix(1,3),transformationMatrix(2,3)));
	
	std::cout << "movementTransform " //Inverse
	          << movementTransform.getBasis().getRow(0)[0] <<" "
	          << movementTransform.getBasis().getRow(0)[1] <<" "
	          << movementTransform.getBasis().getRow(0)[2] <<" "
		      << movementTransform.getOrigin()[0] <<" " << std::endl;
    std::cout << movementTransform.getBasis().getRow(1)[0] <<" "
	          << movementTransform.getBasis().getRow(1)[1] <<" "
	          << movementTransform.getBasis().getRow(1)[2] <<" "
		      << movementTransform.getOrigin()[1] <<" " << std::endl;
	std::cout << movementTransform.getBasis().getRow(2)[0] <<" "
	          << movementTransform.getBasis().getRow(2)[1] <<" "
	          << movementTransform.getBasis().getRow(2)[2] <<" "
		      << movementTransform.getOrigin()[2] <<" " << std::endl;
	
	_transformBroadcastTimer.stop();
	
    try{
      _tfListener.lookupTransform("/map", "/laser",
                               ros::Time(0), laserTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    
    tf::Transform newBaseFootprintTransform;
    
    newBaseFootprintTransform= (laserTransform * movementTransform) * _laserToBaseFootprintTransform;
	
	_tfbroadcaster.sendTransform(tf::StampedTransform(newBaseFootprintTransform, ros::Time::now(), "map", "base_footprint"));
	
	_currentTransform=newBaseFootprintTransform;
	
	_transformBroadcastTimer.start();

	
	sensor_msgs::PointCloud2 PointCloudMsg;
	PointCloudMsg = *msg;
	PointCloudMsg.header.stamp = ros::Time::now();
	_transformedCloudPubliser.publish(PointCloudMsg);
	
	
	std::cout << "Has converged: " << icp.hasConverged() << " score: " 
	          << icp.getFitnessScore() << std::endl;
	std::cout << "ICP done! " << ros::Time::now() << std::endl;
	

}

void PandoraIcpSlam3d::octomapCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	boost::lock_guard<boost::mutex> lock(_octomapCloudMutex);
	
	PointCloud pointCloud;
	pcl::fromROSMsg(*msg,pointCloud);
	
	std::vector< int > 	index;
	
	pcl::removeNaNFromPointCloud(pointCloud,_octomapCloud,index);
}


}
