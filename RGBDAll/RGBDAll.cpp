// RGBDAll.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"

#include "ros/ros.h"

#include "pxcsensemanager.h"
#include "pxcemotion.h"
#include "pxchandconfiguration.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include "FaceSensor.h"

//ATL
#include <atlbase.h>

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
	"Anger",
	"Contempt",
	"Disgust",
	"Fear",
	"Joy",
	"Sandness",
	"Surprise"
};

static WCHAR *SentimentLabels[] = {
	L"NEGATIVE",
	L"POSITIVE",
	L"NEUTRAL"
};

FaceSensor* faceSensor;

int gAge = 0;
int gAgeNum = 0;
int gLastAge = 0;

int updateAge(int age)
{
	const int bias = 40;
	const int minIgnore = 20;

	if (gAgeNum == 0)
	{
		gAge = age;
		gAgeNum = 1;
	}
	else if (gAgeNum < minIgnore)
	{
		// 取均值
		gAge = ((double)gAgeNum / ((double)gAgeNum + 1.0)) * (double)gAge + age / ((double)gAgeNum + 1.0);
		gAgeNum++;
	}
	else if (abs(gAge - age) < bias)
	{
		double a = gAge;
		double b = age;
		gAge = 0.9 * a + 0.1 * b;
	}
	else
	{
		// drop
	}

	return gAge;
}

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "rgbd_all");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	// params
	int rate;
	ph.param<int>("rate", rate, 10);
	ROS_INFO("Param set rate to [%d]", rate);

	std::string age_model_file;
	ph.param<std::string>("age_model_file", age_model_file, "C:/opt/vc11/lamda/lib/aging_model.mat");
	ROS_INFO("Param set age_model_file to [%s]", age_model_file);

	std::string face_cascade_file;
	ph.param<std::string>("face_cascade_file", face_cascade_file, "C:/opt/vc11/lamda/lib/haarcascade_frontalface_alt.xml");
	ROS_INFO("Param set face_cascade_file to [%s]", face_cascade_file);

	bool viewImage;
	ph.param<bool>("view_image", viewImage, true);
	ROS_INFO("Param set view_image [%d]", viewImage);

	bool enableRGB;
	ph.param<bool>("enable_rgb", enableRGB, true);
	ROS_INFO("Param set enable_rgb [%d]", enableRGB);

	bool enableDepth;
	ph.param<bool>("enable_depth", enableDepth, true);
	ROS_INFO("Param set enable_depth [%d]", enableDepth);

	bool enableAge;
	ph.param<bool>("enable_age", enableAge, true);
	ROS_INFO("Param set enable_age [%d]", enableAge);

	bool enableEmotion;
	ph.param<bool>("enable_emotion", enableEmotion, true);
	ROS_INFO("Param set enable_emotion [%d]", enableEmotion);

	bool enableGesture;
	ph.param<bool>("enable_gesture", enableGesture, true);
	ROS_INFO("Param set enable_gesture [%d]", enableGesture);

	bool enableVideo;
	ph.param<bool>("enable_video", enableVideo, false);
	ROS_INFO("Param set enable_video [%d]", enableVideo);

	// publishers
	ros::Publisher depthPub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
	ros::Publisher rgbPub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw", 1);
	ros::Publisher depthCameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 1);
	ros::Publisher emotionPub = nh.advertise<std_msgs::String>("/emotion", 1);
	ros::Publisher faceBiasPub = nh.advertise<geometry_msgs::Point>("/face_bias", 1);
	ros::Publisher agePub = nh.advertise<std_msgs::Int32>("/age", 1);
	ros::Publisher gesturePub = nh.advertise<std_msgs::String>("/gesture", 1);
	ros::Rate loopRate(rate);

	// Create a PXCSenseManager instance
	PXCSenseManager *sm = PXCSenseManager::CreateInstance();

	// Select the color and depth streams
	if (enableRGB | enableAge | enableEmotion)
	{
		sm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480, 30);
	}
	if (enableDepth)
	{
		sm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480, 30);
	}
	if (enableEmotion)
	{
		sm->EnableEmotion();
	}

	// Gestures
	PXCHandModule* handModule = NULL;
	PXCHandData* handAnalysis = NULL;
	std::wstring gestureStr;
	std::wstring leftRightHand;
	int handCount = 0;
	if (enableGesture)
	{
		sm->EnableHand(0);
		handModule = sm->QueryHand();
		handAnalysis = handModule->CreateOutput();

		// Hand Module Configuration
		PXCHandConfiguration *handConfig = handModule->CreateActiveConfiguration();
		handConfig->EnableAllAlerts();
		handConfig->EnableGesture(L"spreadfingers");
		//handConfig->EnableGesture(L"fist");
		handConfig->EnableGesture(L"thumb_down");
		handConfig->EnableGesture(L"thumb_up");
		handConfig->EnableGesture(L"v_sign");
		handConfig->EnableGesture(L"wave");
		handConfig->EnableNormalizedJoints(TRUE);
		//handConfig->EnableSegmentationImage(true);
		handConfig->ApplyChanges();
		handConfig->Update();
	}

	// Initialize and Stream Samples
	pxcStatus status = sm->Init();
	if (status < PXC_STATUS_NO_ERROR) throw L"pxc init error!";

	// Age
	std::string emotionStr;
	cv::Rect faceRect;
	int ageCount = 0;
	if (enableAge)
	{
		// Age estimation
		FaceSensor::init();

		faceSensor = new FaceSensor(age_model_file.c_str(), face_cascade_file.c_str(), 1);
	}

	// loop
	UINT32 seq = 0;
	bool isViewRGB = TRUE;
	bool isViewDepth = FALSE;
	bool isViewEmotion = TRUE;
	bool isViewAge = TRUE;
	bool isViewHands = TRUE;
	bool isViewGesture = TRUE;

	// videos
	VideoWriter writer;
	if (enableVideo)
	{
		writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(640, 480));
	}

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

		// The image passed to FaceSensor must be a gary image.
		cv::Mat im(pxcRgbInfo.height, pxcRgbInfo.width, CV_8UC3);
		cv::Mat gravyIm(im.size(), CV_8UC1);

		memcpy(im.data, pxcRgbData.planes[0], sizeRgb);

		cvtColor(im, gravyIm, CV_BGR2GRAY);

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
				faceRect.x = rect.x;
				faceRect.y = rect.y;
				faceRect.height = rect.h;
				faceRect.width = rect.w;

				int spidx = -1;
				maxscoreE = -3; maxscoreI = 0;
				for (int i = 0; i < 3; i++) 
				{
					if (arrData[7 + i].evidence  < maxscoreE) continue;
					if (arrData[7 + i].intensity < maxscoreI) continue;
					maxscoreE = arrData[7 + i].evidence;
					maxscoreI = arrData[7 + i].intensity;
					spidx = i;
				}

				std_msgs::String rosEmotionStr;
				rosEmotionStr.data = EmotionLabelsA[epidx];
				emotionPub.publish<std_msgs::String>(rosEmotionStr);

				emotionStr.clear();
				emotionStr.append(EmotionLabelsA[epidx]);
				ageCount = 0;

				// pub face center
				int x = faceRect.x + faceRect.width / 2;
				int y = faceRect.y + faceRect.height / 2;
				geometry_msgs::Point faceBias;
				faceBias.x = (double)x / (double)im.cols;
				faceBias.y = (double)y / (double)im.rows;
				faceBiasPub.publish(faceBias);

				std::cout << "FaceBias x = [" << faceBias.x << "] y = [" << faceBias.y << "]" << std::endl;
			}
		}

		// Gestures
		if (enableGesture && handModule)
		{
			handAnalysis->Update();
			pxcI32 numOfGesture = handAnalysis->QueryFiredGesturesNumber();

			if (numOfGesture > 0)
			{
				//Iterate fired gestures
				std::string tmpStr;
				for (int i = 0; i < numOfGesture; i++)
				{
					//Get fired gesture data
					PXCHandData::GestureData gestureData;
					if (handAnalysis->QueryFiredGestureData(i, gestureData) == PXC_STATUS_NO_ERROR)
					{
						//Get hand data related to fired gesture
						PXCHandData::IHand* handData;
						if (handAnalysis->QueryHandDataById(gestureData.handId, handData) == PXC_STATUS_NO_ERROR)
						{
							gestureStr = gestureData.name;
							if (handData->QueryBodySide() == PXCHandData::BodySideType::BODY_SIDE_LEFT)
							{
								leftRightHand = L"left hand: ";
							}
							else if (handData->QueryBodySide() == PXCHandData::BodySideType::BODY_SIDE_RIGHT)
							{
								leftRightHand = L"right hand: ";
							}
							handCount = 0;
						}
					}
				}
				std_msgs::String rosMsg;
				rosMsg.data = ATL::CW2A(leftRightHand.c_str());
				rosMsg.data.append(ATL::CW2A(gestureStr.c_str()));
				gesturePub.publish(rosMsg);
			}
		}

		// Age estimation
		std::vector<FaceBiometrics> fbs = std::vector<FaceBiometrics>();

		// Analyse this image
		faceSensor->analyse(gravyIm, fbs);

		if (fbs.size() == 1)
		{
			std_msgs::Int32 rosInt;
			gLastAge = fbs[0].getAge();
			rosInt.data = updateAge(gLastAge);
			agePub.publish(rosInt);
		}

		if (isViewHands)
		{
			const int rgbBiasX = 40;
			const int rgbBiasY = 30;
			PXCHandData::JointData nodes[2][PXCHandData::NUMBER_OF_JOINTS] = {};

			//Iterate hands
			for (pxcI32 i = 0; i < handAnalysis->QueryNumberOfHands(); i++)
			{
				//Get hand by time of appearence
				PXCHandData::IHand* handData;
				if (handAnalysis->QueryHandData(PXCHandData::AccessOrderType::ACCESS_ORDER_BY_TIME, i, handData) == PXC_STATUS_NO_ERROR)
				{
					PXCHandData::JointData jointData;
					//Iterate Joints
					for (int j = 0; j < PXCHandData::NUMBER_OF_JOINTS; j++)
					{
						handData->QueryNormalizedJoint((PXCHandData::JointType)j, jointData);
						nodes[i][j] = jointData;
					}
				}
			}

			for (int i = 0; i < 2; ++i)
			{

				int wristX = (int)nodes[i][0].positionImage.x + rgbBiasX;
				int wristY = (int)nodes[i][0].positionImage.y + rgbBiasY;

				cv::Point lastJoint = cv::Point(wristX, wristY);

				//Draw Bones
				if (true)
				{
					for (int j = 1; j < PXCHandData::NUMBER_OF_JOINTS; ++j)
					{
						if (nodes[i][j].confidence == 0) continue;

						int x = (int)nodes[i][j].positionImage.x + rgbBiasX;
						int y = (int)nodes[i][j].positionImage.y + rgbBiasY;

						if (j == 2 || j == 6 || j == 10 || j == 14 || j == 18)
						{
							lastJoint = cv::Point(wristX, wristY);
							circle(im, lastJoint, 2, cv::Scalar(0, 0, 255));
						}

						cv::Point currentJoint = cv::Point(x, y);
						line(im, lastJoint, currentJoint, cv::Scalar(155, 0, 0, 0.2), 4);
						circle(im, currentJoint, 5, cv::Scalar(0, 255, 0));

						lastJoint = currentJoint;

					}//end for joints
				}
			}//end if jointNodes
		}

		if (ageCount < 5 && (isViewEmotion || isViewAge))
		{
			// Show the results.
			//cv::Rect faceRect = fbs[0].getLocation();

			rectangle(im, faceRect, Scalar(255, 255, 255), 3);
			std::string ageStr("Age: ");
			ageStr.append(boost::lexical_cast<std::string>(gAge));
			putText(im, ageStr, cv::Point(faceRect.x, faceRect.y), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 0, 0), 2);
			putText(im, emotionStr, cv::Point(faceRect.x, faceRect.y + 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 2);

			ageCount++;

			ageStr = "Last age: ";
			ageStr.append(boost::lexical_cast<std::string>(gLastAge));
			putText(im, ageStr, cv::Point(10, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 255), 1);

			int x = faceRect.x + faceRect.width / 2;
			int y = faceRect.y + faceRect.height / 2;
			cv::circle(im, cv::Point(x, y), 2, cv::Scalar(0, 200, 200), 2);
		}
		else if (ageCount >= 5)
		{
			gAge = 0;
			gAgeNum = 0;
			gLastAge = 0;
		}

		if (isViewGesture && handCount < 5)
		{
			std::string str(ATL::CW2A(leftRightHand.c_str()));
			putText(im, str, cv::Point(10, 40), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 255), 1);
			str = ATL::CW2A(gestureStr.c_str());
			putText(im, str, cv::Point(10, 60), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 255, 255), 1);

			handCount++;
		}

		if (enableVideo)
		{
			writer << im;
		}

		if (isViewDepth)
		{
			cv::Mat depthImg(pxcDepthInfo.height, pxcDepthInfo.width, CV_16UC1);
			memcpy(depthImg.data, pxcDepthData.planes[0], pxcDepthInfo.width * pxcDepthInfo.height * sizeof(uint16_t));
			depthImg.convertTo(depthImg, CV_8UC1);
			cv::imshow("Depth", depthImg);
		}

		if (isViewRGB)
		{
			cv::imshow("RGB", im);
		}
		cv::waitKey(1);


		// go fetching the next sample
		sample->color->ReleaseAccess(&pxcRgbData);
		sample->depth->ReleaseAccess(&pxcDepthData);
		sm->ReleaseFrame();

		ROS_INFO("Pub image seq: [%d]", seq);
		loopRate.sleep();
		seq++;
	} // End while

	// Close down
	faceSensor->close();
	if (enableGesture && handAnalysis != NULL)
	{
		handAnalysis->Release();
	}
	sm->Close();
	sm->Release();

	//ros::waitForShutdown();
	return 0;
}

