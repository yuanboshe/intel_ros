// VoiceSynthesis.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "VoiceSynthesis.h"


// for ATL::CA2W
#include <atlbase.h>

class MyVoiceSynthesis : public VoiceSynthesis
{
public:
	MyVoiceSynthesis(PXCSession *session = 0) : VoiceSynthesis(session){}

	void dealInfo(char* info)
	{
		ROS_INFO("%s", info);
	}

	void dealError(char* error)
	{
		ROS_ERROR("%s", error);
	}
};

MyVoiceSynthesis* speaker;

void speakerCallback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data == "")
	{
		ROS_WARN("Empty data!");
		return;
	}
	speaker->speak(msg->data.c_str());
}

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "speaker");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("voice_synthesis", 5, speakerCallback);
	
	PXCSession *session = 0;
	session = PXCSession::CreateInstance();
	if (!session)
	{
		throw L"Failed to create an SDK session";
	}

	pxcCHAR* sentence = L"Hello world！";
	speaker = new MyVoiceSynthesis(session);
	speaker->speak(sentence);

	ros::spin();

	session->Release();
	delete speaker;
	return 0;
}
