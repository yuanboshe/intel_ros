#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atlbase.h>

class RosSpeaker
{
private:
	ros::NodeHandle node_;
	ros::Publisher pub_;
	void init()
	{
		pub_ = node_.advertise<std_msgs::String>("voice_synthesis", 10);
	}

public:
	RosSpeaker(const ros::NodeHandle initNode) : node_(initNode)
	{
		init();
	}

	RosSpeaker()
	{
		init();
	}

	~RosSpeaker(void){}

 	//template<typename T>
	void speak(const char* sentence)
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << sentence;
		msg.data = ss.str();
		pub_.publish(msg);
		ROS_INFO("Speaker: [%s]", msg.data.c_str());
	}

	void speak(const wchar_t* sentence)
	{
		ATL::CW2A csentence(sentence);
		speak(csentence);
	}
};
