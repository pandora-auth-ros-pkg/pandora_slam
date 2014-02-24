#ifndef PANDORA_ICP_SLAM_3D
#define PANDORA_ICP_SLAM_3D

#include "ros/ros.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/filters/filter.h>

 
namespace pandora_slam_3d
{
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
	class PandoraIcpSlam3d 
	{
		private:
			ros::NodeHandle _nodeHandle;
			
			tf::TransformBroadcaster _tfbroadcaster;
		
			PointCloud _previousCloud;
			
			tf::Transform _currentTransform;
			tf::StampedTransform _kinectToBaseFootprintTransform;
			tf::TransformListener _tfListener;
			
			ros::Timer _transformBroadcastTimer;
			
			ros::Subscriber _inputCloudSubscriber;

			ros::Publisher _transformedCloudPubliser;
			
			void inputCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
			
			void transformBroadcastTimerCallback(const ros::TimerEvent&);
			
		public:
			PandoraIcpSlam3d();
			~PandoraIcpSlam3d();
	};
}

#endif
