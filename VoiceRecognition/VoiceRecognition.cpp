#include "stdafx.h"

// for Intel part
#include "VoiceRecognition.h"
#include <sstream>
#include <atlbase.h>

// for ROS part
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "RosSpeaker.h"

VoiceRecognition::VoiceRecognition(PXCSession* session)
{
	// Create session
	if (session == 0)
	{
		session = PXCSession::CreateInstance();
		if (!session)
		{
			throw L"Failed to create an SDK session";
		}
	}
	this->session = session;

	//setlocale(LC_CTYPE, ".936"); // display Chinese in Unicode 不注释掉会导致输出字符串断词问题
	descList = new DescList(session);
}

VoiceRecognition::~VoiceRecognition()
{
}

void PXCAPI VoiceRecognition::OnRecognition(const PXCSpeechRecognition::RecognitionData *data)
{
	//dealInfo(L"onrecog");
	//if (data->scores[0].label<0) 
	//{
	//	dealInfo((pxcCHAR*)data->scores[0].sentence);
	//	if (data->scores[0].tags[0])
	//		dealInfo((pxcCHAR*)data->scores[0].tags);
	//}
	//else
	//{
	//	for (int i = 0; i<sizeof(data->scores) / sizeof(data->scores[0]); i++) 
	//	{
	//		if (data->scores[i].label < 0 || data->scores[i].confidence == 0) continue;
	//		std::wstringstream info;
	//		info << L"label: " << data->scores[i].label;
	//		info << L" speetch: " << data->scores[i].sentence;
	//		info << L" confidence: " << data->scores[i].confidence;
	//		dealInfo(info.str().c_str());
	//	}
	//	if (data->scores[0].tags[0])
	//		dealInfo((pxcCHAR*)data->scores[0].tags);
	//}


	int label = data->scores[0].label;
	if (label < 0)
	{
		ROS_ERROR("Wrowng label: [%d]", label);
		return;
	}
	std::string command = voiceCmds_.cmds[label].command;
	std::string speech = voiceCmds_.cmds[label].speech;
	ROS_INFO("Voice recognized: speach [%s] command [%s]", speech.c_str(), command.c_str());
	if ("time" == command)
	{
		time_t t;
		struct tm local;
		struct tm gmt;
		char buf[128] = { 0 };
		t = time(NULL);
		localtime_s(&local, &t);
		gmtime_s(&gmt, &t);
		strftime(buf, 128, "Current time is %X", &gmt);
		speaker_.speak(buf);
	}
	else if ("date" == command)
	{
		time_t t;
		struct tm gmt;
		char buf[128] = { 0 };
		t = time(NULL);
		gmtime_s(&gmt, &t);
		strftime(buf, 128, "Current date is %x", &gmt);
		speaker_.speak(buf);
	}
	else
	{
		std_msgs::String msg;
		msg.data = command;
		voiceRecPub_.publish(msg);
		//speaker_.speak(speech.c_str());
	}
}

void  PXCAPI VoiceRecognition::OnAlert(const PXCSpeechRecognition::AlertData *data)
{
	dealInfo(DescList::AlertToString(data->label));
}

void VoiceRecognition::dealInfo(char* info)
{
	//printf_s("[INFO]: %s\n", info);
	ROS_INFO("%s", info);
}

void VoiceRecognition::dealError(char* error)
{
	//printf_s("[ERROR]: %s\n", error);
	ROS_ERROR("%s", error);
}

void VoiceRecognition::dealInfo(std::wstring info)
{
	dealInfo(ATL::CW2A(info.c_str()));
}

void VoiceRecognition::dealError(std::wstring error)
{
	dealError(ATL::CW2A(error.c_str()));
}

void VoiceRecognition::setSource(int id)
{
	/* Create an Audio Source */
	audioSource = session->CreateAudioSource();
	if (!audioSource)
	{
		throw L"Failed to create the audio source";
	}

	/* Set Audio Source */
	pxcStatus sts = audioSource->SetDevice(&descList->sources[id]);
	if (sts < PXC_STATUS_NO_ERROR)
	{
		throw L"Failed to set the audio source";
	}
}

void VoiceRecognition::setModule()
{
	/* Set Module */
	pxcStatus sts = session->CreateImpl<PXCSpeechRecognition>(&speechRec);
	if (sts < PXC_STATUS_NO_ERROR)
	{
		throw L"Failed to create the module instance";
	}

}

void VoiceRecognition::setLanguage(int id)
{
	/* Set configuration according to user-selected language */
	PXCSpeechRecognition::ProfileInfo pinfo;
	speechRec->QueryProfile(id, &pinfo);
	pxcStatus sts = speechRec->SetProfile(&pinfo);
	if (sts < PXC_STATUS_NO_ERROR)
	{
		throw L"Failed to set language";
	}
}
//---------------------------------------------------------------------
// main
//---------------------------------------------------------------------

int _tmain(int argc, char** argv)
{
	ros::init(argc, argv, "voice_recognition");
	ros::NodeHandle node;
	ros::NodeHandle ph("~");

	// params
	std::string path;
	ph.param<std::string>("cmd_csv_path", path, "../common/commands.csv");
	ROS_INFO("Param set cmd_csv_path to [%s]", path);

	int device_id = 0;
	ph.param<int>("device_id", device_id, 0);
	ROS_INFO("Param set device_id to [%d]", device_id);

	// Create session
	PXCSession* session = PXCSession::CreateInstance();
	if (!session)
	{
		throw L"Failed to create an SDK session";
	}

	VoiceRecognition* voiceRec = new VoiceRecognition(session);
	RosSpeaker speaker = voiceRec->speaker_;
	voiceRec->voiceRecPub_ = node.advertise<std_msgs::String>("/recognizer/output", 1);

	DescList* descList = voiceRec->descList;
	std::vector<PXCAudioSource::DeviceInfo> sources = descList->sources;
	std::vector<std::wstring> modules = descList->modules;
	std::vector<std::wstring> languages = descList->languages;

	int size = sources.size();
	for (int i = 0; i < size; i++)
	{
		voiceRec->dealInfo(L"Source[" + boost::lexical_cast<std::wstring>(i) + L"]: " + sources[i].name);
	}
	
	size = modules.size();
	for (int i = 0; i < size; i++)
	{
		voiceRec->dealInfo(L"Module[" + boost::lexical_cast<std::wstring>(i) + L"]: " + modules[i]);
	}

	size = languages.size();
	for (int i = 0; i < size; i++)
	{
		voiceRec->dealInfo(L"Language[" + boost::lexical_cast<std::wstring>(i)+L"]: " + languages[i]);
	}

	voiceRec->setSource(device_id);
	voiceRec->setModule();
	voiceRec->setLanguage(0);

	// Load commands
	VoiceCommands& voiceCmds = voiceRec->voiceCmds_;
	voiceCmds.loadCsvFile(path);
	std::string str = boost::lexical_cast<string>(voiceCmds.cmds.size()) + "commands loaded!";
	Sleep(1000);
	speaker.speak("Voice recognition system start...");
	speaker.speak(str.c_str());
	std::vector<std::wstring> cmds = voiceCmds.getWCommands();
	if (cmds.empty())
	{
		voiceRec->dealError(L"No Command List. Dictation instead.");
		return -1;
	}
	else
	{
		pxcUID grammar = 1;
		pxcEnum lbl = 0;
		pxcCHAR ** _cmds = new pxcCHAR *[cmds.size()];
		for (std::vector<std::wstring >::iterator itr = cmds.begin(); itr != cmds.end(); itr++, lbl++) {
			_cmds[lbl] = (pxcCHAR *)itr->c_str();
		}

		pxcStatus sts;

		sts = voiceRec->speechRec->BuildGrammarFromStringList(grammar, _cmds, NULL, cmds.size());
		if (sts < PXC_STATUS_NO_ERROR) throw L"Can not set Command List.";

		sts = voiceRec->speechRec->SetGrammar(grammar);
		if (sts < PXC_STATUS_NO_ERROR) throw L"Can not set Command List.";
	}

	/* Start Recognition */
	voiceRec->dealInfo(L"Start Recognition");
	pxcStatus sts = voiceRec->speechRec->StartRec(voiceRec->audioSource, voiceRec);
	if (sts < PXC_STATUS_NO_ERROR) 
	{
		throw L"Failed to start recognition";
	}

	ros::waitForShutdown();

	return 0;
}
