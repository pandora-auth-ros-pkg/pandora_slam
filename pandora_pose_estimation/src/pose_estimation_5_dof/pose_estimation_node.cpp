#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <string>

namespace pose_estimation_5_dof {


class PoseEstimation {
	
	public:
		PoseEstimation(int argc, char **argv);
		void serveImuMessage(const sensor_msgs::ImuConstPtr& msg);
		void publishPose(const ros::TimerEvent&);
		
	private:
		
		ros::NodeHandle _nh;
		
		tf::TransformBroadcaster _poseBroadcaster;
		ros::Timer _poseBroadcastTimer;
		ros::Subscriber _imuSubscriber;
		
		std::string _imuTopic;
		std::string _frameFrom;
		std::string _frameTo;
		
		double _imuYaw;	
		double _imuPitch;	
		double _imuRoll;	
		
		double _pose_freq;
};

PoseEstimation::PoseEstimation(int argc, char **argv) {
	
	if (_nh.hasParam("/pose_estimation/imu_topic")) {
		_nh.getParam("/pose_estimation/imu_topic", _imuTopic);
	}
	else {
		ROS_WARN("[Pandora pose estimation] : Parameter imu_topic not found. Using Default");
		_imuTopic = "/sensors/imu" ;
	}
	
	if (_nh.hasParam("/pose_estimation/frame_from")) {
		_nh.getParam("/pose_estimation/frame_from", _frameFrom);
	}
	else {
		ROS_WARN("[Pandora pose estimation] : Parameter frame_from not found. Using Default");
		_frameFrom = "base_footprint" ;
	}
	
	if (_nh.hasParam("/pose_estimation/frame_to")) {
		_nh.getParam("/pose_estimation/frame_to", _frameTo);
	}
	else {
		ROS_WARN("[Pandora pose estimation] : Parameter frame_to not found. Using Default");
		_frameTo = "base_link" ;
	}
	
	if (_nh.hasParam("/pose_estimation/pose_freq")) {
		_nh.getParam("/pose_estimation/pose_freq", _pose_freq);
	}
	else {
		ROS_WARN("[Pandora pose estimation] : Parameter pose_freq not found. Using Default");
		_pose_freq = 5.0 ;
	}
	
	_imuSubscriber = _nh.subscribe(_imuTopic, 1, &PoseEstimation::serveImuMessage, this);
	
	_poseBroadcastTimer = _nh.createTimer(ros::Duration(1.0/_pose_freq),&PoseEstimation::publishPose,this);
	_poseBroadcastTimer.start();
	
}

void PoseEstimation::serveImuMessage(const sensor_msgs::ImuConstPtr& msg) {
	
	tf::Matrix3x3 matrix(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
	matrix.getRPY(_imuRoll, _imuPitch, _imuYaw);
}

void PoseEstimation::publishPose(const ros::TimerEvent&) {
	tf::Vector3 translation(0,0, 0.15);
	tf::Quaternion rotation;
	rotation.setRPY(_imuRoll,_imuPitch,0);
	tf::Transform tfTransformFinal(rotation,translation);
	_poseBroadcaster.sendTransform(tf::StampedTransform(tfTransformFinal, ros::Time::now(),
				_frameFrom, _frameTo));	
}
}

int main (int argc, char **argv){
	ros::init(argc,argv,"pose_estimation_node",ros::init_options::NoSigintHandler);	
	pose_estimation_5_dof::PoseEstimation poseEstimation(argc, argv);
	ROS_DEBUG("Pandora pose estimation node initialised");
	ros::spin();
	return 0;
}
