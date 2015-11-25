#include <ros/ros.h>
#include <face_srv/AddTwoInts.h>

bool func(face_srv::AddTowInts::Request &req,
		face_srv::AddTowInts::Response &res)
{
	res.sum = req.a + req.b;
	ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "service_node");
	ros::NodeHandle nh;
	ros::ServiceServer service = n.advertiseService("sum_service", func);
	ROS_INFO("service reay!");
	ros::spin();

	return();
}
