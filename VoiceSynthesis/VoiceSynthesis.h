#pragma once
#include "pxcspeechsynthesis.h"
#include "pxcsession.h"
class VoiceSynthesis
{
public:
	VoiceSynthesis(PXCSession *session = 0);
	~VoiceSynthesis();

	virtual void dealInfo(char* info);
	virtual void dealError(char* error);

	void speak(pxcCHAR* sentence);
	PXCSession* session;
	PXCSpeechSynthesis* synth;
	PXCSpeechSynthesis::ProfileInfo pinfo;

	void speak(const char* sentence);
	//void speak(std::string sentence);
};

