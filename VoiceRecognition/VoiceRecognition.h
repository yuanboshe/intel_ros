#pragma once
#include "pxcsensemanager.h"
#include "pxcspeechrecognition.h"
#include "VoiceCommands.h"
#include "ros/ros.h"
#include "RosSpeaker.h"
#include "DescList.h"

class VoiceRecognition :public PXCSpeechRecognition::Handler
{
public:
	
	VoiceCommands voiceCmds_;
	ros::Publisher voiceRecPub_;
	RosSpeaker speaker_;
	

	PXCSession* session;
	PXCSpeechRecognition* speechRec;
	PXCAudioSource* audioSource;
	DescList* descList;

public:
	~VoiceRecognition();
	VoiceRecognition(PXCSession* session = 0);

public:
	virtual void  PXCAPI OnRecognition(const PXCSpeechRecognition::RecognitionData *data);
	virtual void  PXCAPI OnAlert(const PXCSpeechRecognition::AlertData *data);
	virtual void dealInfo(char* info);
	virtual void dealError(char* error);
	virtual void dealInfo(std::wstring info);
	virtual void dealError(std::wstring error);
	void setSource(int id);
	void VoiceRecognition::setModule();
	void VoiceRecognition::setLanguage(int id);
};

