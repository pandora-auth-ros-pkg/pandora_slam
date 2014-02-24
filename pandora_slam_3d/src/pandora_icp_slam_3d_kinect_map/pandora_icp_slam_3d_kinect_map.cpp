#include "pandora_icp_slam_3d_kinect_map/pandora_icp_slam_3d_kinect_map.h"

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
	                                                   
	
	ros::Duration(0.5).sleep();
    try{
      _tfListener.lookupTransform("/kinect_camera", "/base_footprint",
                               ros::Time(0), _kinectToBaseFootprintTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    	
	_transformBroadcastTimer.start();						  
		  
	
	_inputCloudSubscriber = _nodeHandle.subscribe("/kinect/point_cloud", 1, 
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
	
	tf::StampedTransform kinectTransform;
    try{
      _tfListener.lookupTransform("/kinect", "/map",
                               ros::Time(0), kinectTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
	
	tf::Matrix3x3 kinectBasis = kinectTransform.getBasis();
	
	tf::Vector3 kinectOrigin = kinectTransform.getOrigin();
	
	Eigen::Matrix4f transformMatrix;
	transformMatrix(0,0) = kinectBasis.getRow(0)[0];
	transformMatrix(0,1) = kinectBasis.getRow(0)[1];
	transformMatrix(0,2) = kinectBasis.getRow(0)[2];
	
	transformMatrix(1,0) = kinectBasis.getRow(1)[0];
	transformMatrix(1,1) = kinectBasis.getRow(1)[1];
	transformMatrix(1,2) = kinectBasis.getRow(1)[2];
	
	transformMatrix(2,0) = kinectBasis.getRow(2)[0];
	transformMatrix(2,1) = kinectBasis.getRow(2)[1];
	transformMatrix(2,2) = kinectBasis.getRow(2)[2];
	
	transformMatrix(0,3) = kinectOrigin[0];
	transformMatrix(1,3) = kinectOrigin[1];
	transformMatrix(2,3) = kinectOrigin[2];
	
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
      _tfListener.lookupTransform("/map", "/kinect",
                               ros::Time(0), kinectTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    
    tf::Transform newBaseFootprintTransform;
    
    newBaseFootprintTransform= (kinectTransform * movementTransform) * _kinectToBaseFootprintTransform;
	
	_tfbroadcaster.sendTransform(tf::StampedTransform(newBaseFootprintTransform, ros::Time::now(), "map", "base_footprint"));
	
	tf::Transform oldKinectTransform;
	oldKinectTransform=kinectTransform * movementTransform.inverse();
	_tfbroadcaster.sendTransform(tf::StampedTransform(newBaseFootprintTransform, ros::Time::now(), "map", "old_kinect"));
	
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
