// RGBDCollector.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "ros/ros.h"

#include "pxcsensemanager.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <opencv2/opencv.hpp>

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "rgbd_collector");
	ros::NodeHandle node;
	ros::NodeHandle ph("~");

	int rate;
	if (!node.getParam("rgbd_collector/rate", rate))
	{
		ROS_WARN("Param [%s] not found.", "rgbd_collector/rate");
		rate = 5;
	}
	ROS_INFO("Set rate to [%d]", rate);
	bool viewImage = true;
	if (!node.getParam("rgbd_collector/viewImage", viewImage))
	{
		ROS_WARN("Param [%s] not found.", "rgbd_collector/viewImage");
	}
	ROS_INFO("View image [%s]", viewImage ? "true" : "false");
	ros::Publisher depthPub = node.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
	ros::Publisher rgbPub = node.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 1);
	ros::Publisher depthCameraInfoPub = node.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1);
	ros::Rate loopRate(rate); // pub rate at 20Hz;

	// Create a PXCSenseManager instance
	PXCSenseManager *sm = PXCSenseManager::CreateInstance();

	// Select the color and depth streams
	sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 30);
	sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480, 30);

	// Initialize and Stream Samples
	sm->Init();

	UINT32 seq = 0;
	while (ros::ok())
	{
		// This function blocks until both samples are ready
		if (sm->AcquireFrame(true)<PXC_STATUS_NO_ERROR) break;

		// retrieve the sample
		PXCCapture::Sample *sample = sm->QuerySample();

		// work on the image sample->color
		PXCImage::ImageInfo pxcRgbInfo = sample->color->QueryInfo();
		PXCImage::ImageInfo pxcDepthInfo = sample->depth->QueryInfo();

		PXCImage::ImageData pxcRgbData;
		PXCImage::ImageData pxcDepthData;
		sample->color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &pxcRgbData);
		sample->depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &pxcDepthData);

		// Convert Intel rgb to Ros rgb image
		std_msgs::Header header;
		header.seq = seq;
		header.stamp = ros::Time::now();
		sensor_msgs::Image rosRgbImg;
		rosRgbImg.header = header;
		rosRgbImg.step = pxcRgbInfo.width * sizeof(uchar) * 3;
		rosRgbImg.width = pxcRgbInfo.width;
		rosRgbImg.height = pxcRgbInfo.height;
		rosRgbImg.is_bigendian = false;
		rosRgbImg.encoding = sensor_msgs::image_encodings::BGR8;
		size_t sizeRgb = rosRgbImg.step * rosRgbImg.height;
		rosRgbImg.data.resize(sizeRgb);
		memcpy((uchar*)(&rosRgbImg.data[0]), pxcRgbData.planes[0], sizeRgb);

		// Convert Intel depth to Ros depth image
		sensor_msgs::CameraInfo rosDepthCameraInfo;
		rosDepthCameraInfo.header = header;
		rosDepthCameraInfo.width = pxcDepthInfo.width;
		rosDepthCameraInfo.height = pxcDepthInfo.height;
		//rosDepthCameraInfo.distortion_model = "plumb_bob";
		sensor_msgs::Image rosDpthImg;
		rosDpthImg.header = header;
		rosDpthImg.step = pxcDepthInfo.width * sizeof(uint16_t);
		rosDpthImg.width = pxcDepthInfo.width;
		rosDpthImg.height = pxcDepthInfo.height;
		rosDpthImg.is_bigendian = false;
		rosDpthImg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		size_t sizeDepth = rosDpthImg.step * rosDpthImg.height;
		rosDpthImg.data.resize(sizeDepth);
		memcpy((uchar*)(&rosDpthImg.data[0]), pxcDepthData.planes[0], sizeDepth);

		// Ros pub
		rgbPub.publish(rosRgbImg);
		depthPub.publish(rosDpthImg);
		depthCameraInfoPub.publish(rosDepthCameraInfo);

		if (viewImage /*&& FALSE*/)
		{
			// View with OpenCV
			cv::Mat rgbImg(pxcRgbInfo.height, pxcRgbInfo.width, CV_8UC3);
			cv::Mat depthImg(pxcDepthInfo.height, pxcDepthInfo.width, CV_16UC1);

			memcpy(rgbImg.data, pxcRgbData.planes[0], pxcRgbInfo.width * pxcRgbInfo.height * 3 * sizeof(uchar));
			memcpy(depthImg.data, pxcDepthData.planes[0], pxcDepthInfo.width * pxcDepthInfo.height * sizeof(uint16_t));

			depthImg.convertTo(depthImg, CV_8UC1);
			cv::imshow("RGB", rgbImg);
			cv::imshow("Depth", depthImg);
			cv::waitKey(1);
		}

		
		// go fetching the next sample
		sample->color->ReleaseAccess(&pxcRgbData);
		sample->depth->ReleaseAccess(&pxcDepthData);
		sm->ReleaseFrame();

		ROS_INFO("Pub image seq: [%d]", seq);
		loopRate.sleep();
		seq++;
	}

	// Close down
	sm->Release();

	ros::waitForShutdown();

	return 0;
}

