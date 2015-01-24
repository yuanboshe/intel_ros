#include "StdAfx.h"
#include "VoiceSynthesis.h"
#include "voice_out.h"
#include "DescList.h"

//#include <iostream>
// for ATL::CA2W
#include <atlbase.h>

VoiceSynthesis::VoiceSynthesis(PXCSession *session)
{
	if (session == 0)
	{
		session = PXCSession::CreateInstance();
		if (!session)
		{
			throw L"Failed to create an SDK session";
		}
	}
	else
	{
		this->session = session;
	}

	//setlocale(LC_CTYPE, ".936");

	DescList descList(session);
	PXCSession::ImplDesc desc = {};
	std::wstring moduleName = descList.GetModuleName();
	memcpy(desc.friendlyName, moduleName.c_str(), moduleName.size() * sizeof(pxcCHAR));

	// Create a PXCSpeechSynthesis instance
	pxcStatus status;
	status = session->CreateImpl<PXCSpeechSynthesis>(&desc, &synth);
	if (status < PXC_STATUS_NO_ERROR) throw "CreateImpl<PXCSpeechSynthesis> failed";

	// Configure the module
	status = synth->QueryProfile(0, &pinfo);

	//Set options
	pinfo.volume = 100;
	if (pinfo.volume > 100 || pinfo.volume < 0)
	{
		throw "Incorrect Volume value. Range for Volume is 0 - 100.";
	}

	pinfo.pitch = 100;
	if (pinfo.pitch>200 || pinfo.pitch < 50)
	{
		throw L"Incorrect Pitch value. Range for Pitch is 50 - 200.";
	}

	pinfo.rate = 100;
	if (pinfo.rate>400 || pinfo.rate < 50)
	{
		throw L"Incorrect Speech Rate value. Range for Speech Rate  is 50 - 400.";
	}

	status = synth->SetProfile(&pinfo);
	if (status < PXC_STATUS_NO_ERROR)
	{
		synth->Release();
		throw L"Failed to set configuration";
	}
}

VoiceSynthesis::~VoiceSynthesis()
{
	synth->Release();
}

void VoiceSynthesis::dealInfo(char* info)
{
	printf_s("[INFO]: %s\n", info);
}

void VoiceSynthesis::dealError(char* error)
{
	printf_s("[ERROR]: %s\n", error);
}

void VoiceSynthesis::speak(pxcCHAR* sentence)
{
	// Synthesize the speech
	if (wcslen(sentence) <= 0)
	{
		throw L"String should not be empty";
	}
	synth->BuildSentence(1, sentence);
	dealInfo(ATL::CW2A(sentence));

	// Render the sentence or write it to vo's internal memory buffer
	VoiceOut vo(&pinfo);
	int nbuffers = synth->QueryBufferNum(1);
	for (int i = 0; i < nbuffers; i++)
	{
		PXCAudio *sample = synth->QueryBuffer(1, i);
		vo.RenderAudio(sample);
	}
}

void VoiceSynthesis::speak(const char* sentence)
{
	speak(ATL::CA2W(sentence));
}

//void VoiceSynthesis::speak(std::string sentence)
//{
//	speak(ATL::CA2W(sentence.c_str()));
//}