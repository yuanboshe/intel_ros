// AgeEstimation.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "ros/ros.h"
#include "FaceSensor.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>
#include "RosSpeaker.h"

void test()
{
	FaceSensor::init();

	const char* faceCascadeFile = "C:/opt/vc11/lamda/lib/haarcascade_frontalface_alt.xml";
	const char* modelFile = "c:/opt/vc11/lamda/lib/aging_model.mat";
	cv::Mat im = imread("c:/opt/vc11/lamda/lib/face.jpg");

	// Create a FaceSensor
	FaceSensor faceSensor = FaceSensor(modelFile, faceCascadeFile);

	// Store the detected faces and related information (age, etc.).
	std::vector<FaceBiometrics> fbs = std::vector<FaceBiometrics>();

	// The image passed to FaceSensor must be a gary image.
	cvtColor(im, im, CV_BGR2GRAY);

	// Show the gray image
	imshow("gray image", im); waitKey();

	// Analyse this image
	faceSensor.analyse(im, fbs);

	// Show the results.
	cout << " Number of faces detected : " << fbs.size() << endl;
	for (size_t i = 0; i < fbs.size(); i++)
	{
		cout << "location " << i << " : " << fbs[i].getLocation() << ", age is " << fbs[i].getAge() << endl;
		imshow("face", fbs[i].getFace());
		waitKey();
	}

	// Don't forget to close the FaceSensor
	faceSensor.close();
}

FaceSensor* faceSensor;

int gAge = 0;
double gAgeNum = 0;

int updateAge(int age)
{
	const int bias = 10;
	const int minIgnore = 20;

	if (gAgeNum == 0)
	{
		gAge = age;
		gAgeNum = 1;
	}
	else if (gAgeNum < minIgnore || abs(gAge - age) < bias)
	{
		// 取均值
		gAge = (gAgeNum / (gAgeNum + 1.0)) * gAge + age / (gAgeNum + 1.0);
		gAgeNum++;
	}
	else
	{
		// drop
	}

	return gAge;
}


cv::Rect gFaceRect;
int gNum = 0;

void rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	try
	{
		// Store the detected faces and related information (age, etc.).
		std::vector<FaceBiometrics> fbs = std::vector<FaceBiometrics>();

		// The image passed to FaceSensor must be a gary image.
		cv::Mat im(msg->height, msg->width, CV_8UC3);
		cv::Mat gravyIm(im.size(), CV_8UC1);

		memcpy(im.data, &msg->data[0], msg->height * msg->width * 3);

		cvtColor(im, gravyIm, CV_BGR2GRAY);

		// Analyse this image
		faceSensor->analyse(gravyIm, fbs);

		if (fbs.size() == 1)
		{
			int age = updateAge(fbs[0].getAge());

			// Show the results.
			cv::Rect faceRect = fbs[0].getLocation();
			rectangle(im, faceRect, Scalar(255, 255, 255), 3);		
			putText(im, boost::lexical_cast<std::string>(age), cv::Point(faceRect.x, faceRect.y), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0, 0), 2);

			cout << "location : " << fbs[0].getLocation() << ", age is " << fbs[0].getAge() << endl;

			gFaceRect = faceRect;
			gNum = 0;
		}
		else if (gNum < 3)
		{
			int age = gAge;
			rectangle(im, gFaceRect, Scalar(255, 255, 255), 3);
			putText(im, boost::lexical_cast<std::string>(age), cv::Point(gFaceRect.x, gFaceRect.y), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0, 0), 2);
			gNum++;
		}

		// Show the gray image
		imshow("gray image", im);
		waitKey(1);
	}
	catch (Exception e)
	{
		std::cout << e.what() << std::endl;
	}
}

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "age_estimation");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	ros::Subscriber subImg = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, rgbCallback);

	FaceSensor::init();

	const char* faceCascadeFile = "C:/opt/vc11/lamda/lib/haarcascade_frontalface_alt.xml";
	const char* modelFile = "c:/opt/vc11/lamda/lib/aging_model.mat";
	cv::Mat im = imread("c:/opt/vc11/lamda/lib/face.jpg");

	// Create a FaceSensor
	faceSensor = new FaceSensor(modelFile, faceCascadeFile, 1);

	ros::Rate rate(10);
	bool isSpeak = TRUE;
	RosSpeaker speaker;
	while (ros::ok())
	{

		// TODO: pub message
		if (isSpeak && gAgeNum > 30)
		{
			std::string str = "Hello, I think you are ";
			str.append(boost::lexical_cast<std::string>(gAge));
			str.append(" years old!");
			speaker.speak(str.c_str());
			isSpeak = FALSE;
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}