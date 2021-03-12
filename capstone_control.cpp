#include "ros/ros.h"
#include "capstone/Msg.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#define PI 3.141592653589
#define LEFT 1100
#define min 18
#define des 1.1
#define Kp 1.15
#define Ki 0.3
#define Kd 0.2
#define dt 0.05 //sapmling time

int count;


class MazeService
{
protected:
	ros::NodeHandle nh_;
	ros::Subscriber ros_maze_sub_;

	ros::Publisher ros_steer_pub_;

	capstone::Msg go;
	float msg180;
	float msg90, msg95, msg100, msg105, msg110, msg115, msg120, msg125, msg130, msg270;

	float des1,des2,des3,des4,des5,des6,des7,des8, des_sum;

	float err, I_err, D_err, err_pre,  Kp_term, Ki_term, Kd_term, control;

	float msg_sum;

public:
	MazeService()
		
	{
		ros_maze_sub_ = nh_.subscribe("/scan", 100, &MazeService::msgCallback, this);
		ros_steer_pub_= nh_.advertise<capstone::Msg>("/STEER", 100);
		
	}

////////////////range data [m] (Note: values < range_min or > range_max should be discarded)
	void angle_algorithm(const sensor_msgs::LaserScan::ConstPtr& msg) 

	{
		go.servo = 1538;
		msg90 = 0.15;
		msg95 = 0.15;
		msg100 = 0.15;
		msg105 = 0.15;
		msg110 = 0.15;
		msg115 = 0.15;
		msg120 = 0.15;
		msg125 = 0.15;
		msg130 = 0.15;

		// inf value controll
		msg90 = msg->ranges[90] > 8 || msg->ranges[90] < 0 ? msg90 : msg->ranges[90];
		msg95 = msg->ranges[95] > 8 || msg->ranges[95] < 0 ? msg95 : msg->ranges[95];
		msg100 = msg->ranges[100] > 8 || msg->ranges[100] < 0 ? msg100 : msg->ranges[100];
		msg105 = msg->ranges[105] > 8 || msg->ranges[105] < 0 ? msg105 : msg->ranges[105];
		msg110 = msg->ranges[110] > 8 || msg->ranges[110] < 0 ? msg110 : msg->ranges[110];
		msg115 = msg->ranges[115] > 8 || msg->ranges[115] < 0 ? msg115 : msg->ranges[115];
		msg120 = msg->ranges[120] > 8 || msg->ranges[120] < 0 ? msg120 : msg->ranges[120];
		msg125 = msg->ranges[125] > 8 || msg->ranges[125] < 0 ? msg125 : msg->ranges[125];
		msg130 = msg->ranges[130] > 8 || msg->ranges[130] < 0 ? msg130 : msg->ranges[130];
		msg270 = msg->ranges[270] > 8 || msg->ranges[270] < 0 ? msg270 : msg->ranges[270];
		// calculating desire distance
		des1 = des/cos(PI/36);
		des2 = des/cos(PI/18);
		des3 = des/cos(PI/12);
		des4 = des/cos(PI/9);
		des5 = des/cos(PI*5/36);
		des6 = des/cos(PI/6);
		des7 = des/cos(PI*7/36);
		des8 = des/cos(PI*2/9);
		
		//PID controller
		msg_sum = msg105 + msg110 + msg115 + msg120 + msg125 + msg130;

		des_sum = des3 + des4 + des5 + des6 + des7 + des8;

		err = des_sum - msg_sum;

		Kp_term = Kp*err;

		I_err = err*dt;

		Ki_term = Ki*I_err;

		D_err = (err-err_pre)/dt;

		Kd_term = Kd*D_err;

		control = Kp_term + Ki_term + Kd_term;

		go.servo = go.servo + control*min < 1288 ? 1288 : go.servo + control*min;
		go.servo = go.servo + control*min > 1788 ? 1788 : go.servo + control*min;

		err_pre = err;

		if(count == 0 && msg270 < 0.5)
			go.servo = 1288;	
	}


	// Subscriber and Publisher
	void msgCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
	{  
		ros::Rate loop_rate(10);	
		angle_algorithm(msg); //angle calculation

		if(count<20)
			go.traction = 102;
		else if(count>=20 && count < 33)
			go.traction = 103;
		else
			go.traction = 109;
		ROS_INFO("traction = %d", go.traction);	
		ROS_INFO("servo = %d", go.servo);	
		ros_steer_pub_.publish(go);
		loop_rate.sleep();
		count++;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "capstone_publisher");

	MazeService mazeservice;

	ros::Rate loop_rate(10);
	
	ros::spin();


	return 0;
}

