#pragma once
#include "../common/csvparser.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <direct.h>
#include <atlbase.h>

class VoiceCommand
{
public:
	VoiceCommand(void)
		: id(0)
	{

	}

	~VoiceCommand(void){}
	int id;
	std::string speech; // user's speach
 	std::string command; // command
	std::string response; // robot's response
};

class VoiceCommands
{
public:
	
	std::vector<VoiceCommand> cmds;
	VoiceCommands(void)
	{
	}

	~VoiceCommands(void)
	{
	}

	void loadCsvFile(std::string path)
	{
		ifstream infile(path);
		if (!infile)
		{
			cout << "Can not open the file!" << endl;
			cout << _getcwd(NULL, 0) << endl;
			system("pause");
			exit(1);
		}

		string sLine;
		CSVParser parser;

		while (!infile.eof()) {
			getline(infile, sLine); // Get a line
			if (sLine == "")
				continue;

			parser << sLine; // Feed the line to the parser

			// Now extract the columns from the line
			VoiceCommand cmd;
			parser >> cmd.id >> cmd.command >> cmd.speech >> cmd.response;
			
			// save cmd into cmds
			if (cmd.id > -1)
			{
				cmds.push_back(cmd);
			}
			else continue;
			
			cout << "Loaded: [" << cmd.id << "] [" << cmd.command << "] [" << cmd.speech << "] [" << cmd.response << "]" << endl;
		}
		infile.close();
	}

	std::vector<std::wstring> getWCommands(void)
	{
		std::vector<std::wstring> wcmds;
		wcmds.reserve(cmds.size());
		for (vector<VoiceCommand>::iterator iter = cmds.begin(); iter != cmds.end(); iter++)
		{
			std::wstring wcmd(ATL::CA2W(iter->speech.c_str()));
			wcmds.push_back(wcmd);
		}
		
		return wcmds; 
	}
};