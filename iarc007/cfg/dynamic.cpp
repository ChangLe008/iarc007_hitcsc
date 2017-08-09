#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <iarc007/iarc007Config.h>

void callback(iarc007::iarc007Config &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
		config.int_param, config.double_param, 
		config.str_param.c_str(), 
		config.bool_param?"True":"False", 
		config.size);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamic_tutorials");

	dynamic_reconfigure::Server<iarc007::iarc007Config> server;
	dynamic_reconfigure::Server<iarc007::iarc007Config>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}
