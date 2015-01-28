#include <opencv2\opencv.hpp>
#include "AgeEstimation.h"

using namespace std;
using namespace cv;

#ifndef    FACE_SENSOR
#define    FACE_SENSOR

class FaceBiometrics {
private:

	Mat face;

	Rect location;

	int age;


public:

	friend class FaceSensor;

	FaceBiometrics(){};

	Rect getLocation() { return location; };

	int getAge() { return age; };

	Mat getFace() { return face; };

};

class FaceSensor {
	
private:
	CascadeClassifier faceCascade;
	
	int maxFacesNum;
	// training samples for knn
	//mwArray knn_train_x = mwArray(), knn_train_y = mwArray();
	mwArray knn_train_x, knn_train_y;

	// model parameters: linear projections
	//mwArray w = mwArray();
	mwArray w;


protected:

	// Convert 
	template<typename T> static mwArray Mat2mwArray2d(const Mat& m);

	// Age Estimation
	int estimate_age(mwArray faceArray);

public:
	// Before create instance, should init Matlab and AgeEstimation libs first.
	static bool init();

	// After useout instance, should close this instance.
	bool close();

	bool analyse(Mat oriImage, std::vector<FaceBiometrics> &fb, Size faceSize = Size(60, 60));

	void setMaxFacesNum(int num) { maxFacesNum = num; };

	// Initialization. Load the model file.
	FaceSensor(const char* modelFile, const char* cascadeFile, int maxFacesNum = 3);

};

bool FaceSensor::init() 
{
	if (!mclInitializeApplication(NULL, 0))
	{
		cerr << " (!) Error connecting to MATLAB Compiler Runtime." << endl;
		return false;
	}

    if( !AgeEstimationInitialize())
    {
        cerr << " (!) Error connecting to the function library (AgeEstimation)." << endl;
        return false;
    }

	return true;
}

FaceSensor::FaceSensor(const char* modelFile, const char* cascadeFile, int maxFacesNum) 
{
	// Load face detector from OPENCV
	if (!faceCascade.load(cascadeFile))
	{
		cerr << " (!) Error loading face cascade model file." << endl;
		return;
	};

	this->maxFacesNum = maxFacesNum;

	// Initialize.
	knn_train_x = mwArray();
	knn_train_y = mwArray();
	w = mwArray();

	// Load aging model parameters.
	load_aging_model_matlab(3, w, knn_train_x, knn_train_y, mwArray(modelFile));

	cout << "Leave FaceSensor Constructor." << endl;
}


bool FaceSensor::close() 
{
	AgeEstimationTerminate();
    mclTerminateApplication();
	return true;
}


template<typename T> mwArray FaceSensor::Mat2mwArray2d(const Mat& m)
{
	int rows = m.rows;
	int cols = m.cols;

	//Mat data is float, and mwArray uses double, so we need to convert.
	mwArray res = mwArray(rows, cols, mxDOUBLE_CLASS);
	double *tmp = new double[rows * cols];

	for (int i = 0; i<rows; i++){
		for (int j = 0; j<cols; j++){
			tmp[i + j*rows] = (double)m.at<T>(i, j);
		}
	}
	res.SetData(tmp, rows * cols);
	delete tmp;
	return res;
}

int FaceSensor::estimate_age(mwArray faceArray)
{
	mwArray res = mwArray(1, 1, mxDOUBLE_CLASS);
	
	estimate_age_matlab(1, res, faceArray, w, knn_train_x, knn_train_y);

	return (int)res(1);
}

bool FaceSensor::analyse(Mat oriImage, std::vector<FaceBiometrics> &fbs, Size faceSize)
{
	cout << "Enter FaceSensor->analyse." << endl;
	
	double t = (double)cvGetTickCount();
	// Face locations
	std::vector<Rect> locations;
	
	// Face Biometrics
	FaceBiometrics fb;
	mwArray faceArray;

	// Face Detection
	faceCascade.detectMultiScale(oriImage, locations, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

	size_t facesNum = ((size_t)maxFacesNum < locations.size()) ? (size_t)maxFacesNum : locations.size();

	// Store locations of detected faces.
	for (size_t i = 0; i < facesNum; i++)
	{
		fb = FaceBiometrics();
		fb.location = locations[i];
		cout << "location " << i << " : " << locations[i] << endl;
		resize(oriImage(locations[i]), fb.face, faceSize);
		
		// Histgorm equalization.		
		equalizeHist(fb.face, fb.face);

		faceArray = Mat2mwArray2d<uchar>(fb.face);

		fb.age = estimate_age(faceArray);

		fbs.push_back(fb);
	}

	t = (double)cvGetTickCount() - t;
	cout << "Leave FaceSensor->analyse. " << (t/(double(cvGetTickFrequency()) * 1000)) << " ms" << endl;
	
	return true;
}

#endif