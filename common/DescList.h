#pragma once
#include "pxcspeechrecognition.h"
#include "pxcspeechsynthesis.h"
#include "pxcsession.h"
#include <vector>
class DescList
{
public:
	PXCSession  *session;
	PXCAudioSource *source;
	std::vector<PXCAudioSource::DeviceInfo> sources;
	std::vector<std::wstring> modules;
	std::vector<std::wstring> languages;
	pxcI32 selectedSource;
	pxcI32 selectedModule;
	pxcI32 selectedLanguage;

	DescList(PXCSession *session = 0)
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

		selectedSource = 0;
		selectedModule = 0;
		selectedLanguage = 0;
		if (sources.size() == 0)
		{
			PopulateSources();
		}
		if (modules.size() == 0)
		{
			PopulateModules();
		}
		if (languages.size() == 0)
		{
			PopulateLanguages(selectedModule);
		}
	}

	~DescList()
	{
	}

	static pxcCHAR *LanguageToString(PXCSpeechSynthesis::LanguageType language)
	{
		switch (language)
		{
		case PXCSpeechSynthesis::LANGUAGE_US_ENGLISH:		return L"US English";
		case PXCSpeechSynthesis::LANGUAGE_GB_ENGLISH:		return L"British English";
		case PXCSpeechSynthesis::LANGUAGE_DE_GERMAN:		return L"Deutsch";
		case PXCSpeechSynthesis::LANGUAGE_IT_ITALIAN:		return L"Italiano";
		case PXCSpeechSynthesis::LANGUAGE_BR_PORTUGUESE:	return L"Português";
		case PXCSpeechSynthesis::LANGUAGE_CN_CHINESE:		return L"中文";
		case PXCSpeechSynthesis::LANGUAGE_FR_FRENCH:		return L"Français";
		case PXCSpeechSynthesis::LANGUAGE_JP_JAPANESE:	    return L"日本語";
		case PXCSpeechSynthesis::LANGUAGE_LA_SPANISH:		return L"LA Español";
		case PXCSpeechSynthesis::LANGUAGE_US_SPANISH:		return L"US Español";
		}
		return 0;
	}

	static pxcCHAR *AlertToString(PXCSpeechRecognition::AlertType label) {
		switch (label) {
		case PXCSpeechRecognition::ALERT_SNR_LOW: return L"SNR_LOW";
		case PXCSpeechRecognition::ALERT_SPEECH_UNRECOGNIZABLE: return L"SPEECH_UNRECOGNIZABLE";
		case PXCSpeechRecognition::ALERT_VOLUME_HIGH: return L"VOLUME_HIGH";
		case PXCSpeechRecognition::ALERT_VOLUME_LOW: return L"VOLUME_LOW";
		case PXCSpeechRecognition::ALERT_SPEECH_BEGIN: return L"SPEECH_BEGIN";
		case PXCSpeechRecognition::ALERT_SPEECH_END: return L"SPEECH_END";
		case PXCSpeechRecognition::ALERT_RECOGNITION_ABORTED: return L"RECOGNITION_ABORTED";
		case PXCSpeechRecognition::ALERT_RECOGNITION_END: return L"RECOGNITION_END";
		}
		return L"Unknown";
	}

	std::vector<PXCAudioSource::DeviceInfo> PopulateSources()
	{

		sources.clear();
		PXCAudioSource *source = session->CreateAudioSource();
		if (source)
		{
			source->ScanDevices();

			for (int i = 0;; i++)
			{
				PXCAudioSource::DeviceInfo dinfo = {};
				if (source->QueryDeviceInfo(i, &dinfo) < PXC_STATUS_NO_ERROR) break;
				sources.push_back(dinfo);
			}

			source->Release();
		}
		return sources;
	}

	std::vector<std::wstring> PopulateModules()
	{
		PXCSession::ImplDesc desc, desc1;
		memset(&desc, 0, sizeof(desc));
		desc.cuids[0] = PXCSpeechSynthesis::CUID;
		int i;
		modules.clear();
		for (i = 0;; i++)
		{
			if (session->QueryImpl(&desc, i, &desc1) < PXC_STATUS_NO_ERROR) break;
			modules.push_back(std::wstring(desc1.friendlyName));
		}
		return modules;
	}

	std::vector<std::wstring> PopulateLanguages(pxcI32 index)
	{
		PXCSession::ImplDesc desc, desc1;
		memset(&desc, 0, sizeof(desc));
		desc.cuids[0] = PXCSpeechSynthesis::CUID;
		unsigned int checkedItem = 0;
		languages.clear();
		if (session->QueryImpl(&desc, index /*ID_MODULE*/, &desc1) >= PXC_STATUS_NO_ERROR)
		{
			PXCSpeechSynthesis *vs = 0;
			if (session->CreateImpl<PXCSpeechSynthesis>(&desc1, &vs) >= PXC_STATUS_NO_ERROR)
			{
				for (int k = 0;; k++)
				{
					PXCSpeechSynthesis::ProfileInfo pinfo;
					if (vs->QueryProfile(k, &pinfo) < PXC_STATUS_NO_ERROR) break;
					languages.push_back(std::wstring(LanguageToString(pinfo.language)));
					if (pinfo.language == PXCSpeechSynthesis::LANGUAGE_US_ENGLISH) checkedItem = k;
				}
				vs->Release();
			}
		}
		return languages;
	}

	void setSource(pxcI32 id)
	{
		this->selectedSource = id;
	}

	void setModule(pxcI32 id)
	{
		this->selectedModule = id;
	}

	void setLanguage(pxcI32 id)
	{
		this->selectedLanguage = id;
	}

	std::wstring GetModuleName()
	{
		return modules.at(selectedModule);
	}
};