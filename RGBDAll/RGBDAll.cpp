// RGBDAll.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"

#include "ros/ros.h"

#include "pxcsensemanager.h"
#include "pxcemotion.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include "FaceSensor.h"

static WCHAR *EmotionLabels[] = {
	L"ANGER",
	L"CONTEMPT",
	L"DISGUST",
	L"FEAR",
	L"JOY",
	L"SADNESS",
	L"SURPRISE"
};

static CHAR *EmotionLabelsA[] = {
	"ANGER",
	"CONTEMPT",
	"DISGUST",
	"FEAR",
	"JOY",
	"SADNESS",
	"SURPRISE"
};

static WCHAR *SentimentLabels[] = {
	L"NEGATIVE",
	L"POSITIVE",
	L"NEUTRAL"
};

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

std::string gEmotionStr;
cv::Rect gFaceRect;
int gNum = 0;

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
	ros::Publisher emotionPub = node.advertise<std_msgs::String>("/emotion", 1);
	ros::Rate loopRate(rate); // pub rate at 20Hz;

	// Create a PXCSenseManager instance
	PXCSenseManager *sm = PXCSenseManager::CreateInstance();

	// Select the color and depth streams
	sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 30);
	sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480, 30);
	sm->EnableEmotion();

	// Initialize and Stream Samples
	sm->Init();

	// Age estimation
	FaceSensor::init();

	const char* faceCascadeFile = "C:/opt/vc11/lamda/lib/haarcascade_frontalface_alt.xml";
	const char* modelFile = "c:/opt/vc11/lamda/lib/aging_model.mat";
	cv::Mat im = imread("c:/opt/vc11/lamda/lib/face.jpg");

	faceSensor = new FaceSensor(modelFile, faceCascadeFile, 1);

	// loop
	UINT32 seq = 0;
	bool isViewRGB = TRUE;
	bool isViewDepth = TRUE;
	bool isViewEmotion = TRUE;
	bool isViewAge = TRUE;

	// videos
	VideoWriter writer("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(640, 480));

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

		// Emotion
		PXCEmotion *em = sm->QueryEmotion();
		if (em != NULL)
		{
			pxcI32 numFaces = em->QueryNumFaces();
			for (pxcI32 fid = 0; fid < numFaces; fid++)
			{
				// retrieve all estimation data
				PXCEmotion::EmotionData arrData[10] = { 0 };
				em->QueryAllEmotionData(fid, arrData);

				// find the emotion with maximum 
				int epidx = -1; pxcI32 maxscoreE = -3; pxcF32 maxscoreI = 0;
				for (int i = 0; i<7; i++) {
					if (arrData[i].evidence < maxscoreE)  continue;
					if (arrData[i].intensity < maxscoreI) continue;
					maxscoreE = arrData[i].evidence;
					maxscoreI = arrData[i].intensity;
					epidx = i;
				}

				PXCRectI32 rect = arrData->rectangle;
				gFaceRect.x = rect.x;
				gFaceRect.y = rect.y;
				gFaceRect.height = rect.h;
				gFaceRect.width = rect.w;

				int spidx = -1;
				maxscoreE = -3; maxscoreI = 0;
				for (int i = 0; i < 3; i++) {
					if (arrData[7 + i].evidence  < maxscoreE) continue;
					if (arrData[7 + i].intensity < maxscoreI) continue;
					maxscoreE = arrData[7 + i].evidence;
					maxscoreI = arrData[7 + i].intensity;
					spidx = i;
				}

				// do something with the outstanding primary emotion
				//std::wcout << "  " << EmotionLabels[epidx] << " | " << SentimentLabels[spidx] << "    ";
				std_msgs::String emotionStr;
				emotionStr.data = EmotionLabelsA[epidx];
				emotionPub.publish<std_msgs::String>(emotionStr);

				gEmotionStr.clear();
				gEmotionStr.append(EmotionLabelsA[epidx]);
			}
		}

		// Age estimation
		std::vector<FaceBiometrics> fbs = std::vector<FaceBiometrics>();

		// The image passed to FaceSensor must be a gary image.
		cv::Mat im(pxcRgbInfo.height, pxcRgbInfo.width, CV_8UC3);
		cv::Mat gravyIm(im.size(), CV_8UC1);

		memcpy(im.data, pxcRgbData.planes[0], sizeRgb);

		cvtColor(im, gravyIm, CV_BGR2GRAY);

		// Analyse this image
		faceSensor->analyse(gravyIm, fbs);

		if (fbs.size() == 1)
		{
			int age = updateAge(fbs[0].getAge());
		}

		if (isViewRGB || isViewDepth || isViewEmotion || isViewAge)
		{
			if (isViewEmotion || isViewAge)
			{
				// Show the results.
				//cv::Rect faceRect = fbs[0].getLocation();
				cv::Rect faceRect = gFaceRect;

				rectangle(im, faceRect, Scalar(255, 255, 255), 3);
				std::string ageStr("Age: ");
				ageStr.append(boost::lexical_cast<std::string>(gAge));
				putText(im, ageStr, cv::Point(faceRect.x, faceRect.y), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0, 0), 2);
				putText(im, gEmotionStr, cv::Point(faceRect.x, faceRect.y + 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 2);

				cv::imshow("RGB", im);
				//gFaceRect = faceRect;
				gNum = 0;

				writer << im;

			}
			else if (isViewRGB)
			{
				cv::Mat rgbImg(pxcRgbInfo.height, pxcRgbInfo.width, CV_8UC3);
				memcpy(rgbImg.data, pxcRgbData.planes[0], pxcRgbInfo.width * pxcRgbInfo.height * 3 * sizeof(uchar));
				cv::imshow("RGB", rgbImg);
			}
			
			if (isViewDepth)
			{
				cv::Mat depthImg(pxcDepthInfo.height, pxcDepthInfo.width, CV_16UC1);
				memcpy(depthImg.data, pxcDepthData.planes[0], pxcDepthInfo.width * pxcDepthInfo.height * sizeof(uint16_t));
				depthImg.convertTo(depthImg, CV_8UC1);
				cv::imshow("Depth", depthImg);
			}

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

