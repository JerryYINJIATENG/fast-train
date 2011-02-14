//  Portions Copyright 2010 Xuesong Zhou

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of FastTrain.

//    FastTrain is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    FastTrain is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with FastTrain.  If not, see <http://www.gnu.org/licenses/>.

// FastTrain.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "Network.h"
#include "FastTrain.h"

#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
using namespace std;
ofstream g_LogFile;
CTime g_AppStartTime;

int g_OptimizationMethod;

/*************************************
How to build a simple transportation scheduling and routing package

step 0: basic input functions
ReadInputFiles()

step 1: dynamic memory management
FreeMemory() to deallocate dynamic memory for the whole simulation program
Allocate and deallocate dynamic arrays for NetworkForSP

step 2: network building for traffic assignment
NetworkForSP::BuildNetwork(int ZoneID)

step 3: generate train data, sorted by departure time
XXXX

step 4: Scan eligiable list and shortest path algorithm
SEList functions in NetworkForSP
TDLabelCorrecting_DoubleQueue(int origin, int departure_time)

step 5: assign inital path to vehicles
GetLinkNoByNodeIndex(int usn_index, int dsn_index)
VehicleBasedPathAssignment(int zone,int departure_time_begin, int departure_time_end, int iteration)

step 6: integerate network building, shortest path algorithm and path assignment to dynamic traffic assignment

step 7: output vehicle trajectory file

step 8: Lagrangian framework

/**************************
menu -> project -> property -> configuraiton -> debugging -> setup working directory

***************************/
// The one and only application object

CWinApp theApp;
std::set<CNode*>		g_NodeSet;
std::map<int, CNode*> g_NodeMap;
std::vector<CTrain*> g_TrainVector;

std::map<int, int> g_NodeIDtoNameMap;
std::map<int, int> g_NodeNametoIDMap;
std::map<unsigned long, CLink*> g_NodeIDtoLinkMap;

int g_OptimizationHorizon = 1440;

std::set<CLink*>		g_LinkSet;
std::map<int, CLink*> g_LinkIDMap;

// maximal # of adjacent links of a node (including physical nodes and centriods( with connectors))
int g_AdjLinkSize = 30; // initial value of adjacent links
// assignment and simulation settings
int g_NumberOfIterations = 1;


using namespace std;



CString g_GetAppRunningTime()
{

	CString str;

	CTime EndTime = CTime::GetCurrentTime();
	CTimeSpan ts = EndTime  - g_AppStartTime;

	str = ts.Format( "App Clock: %H:%M:%S --" );
	return str;
}



void ReadInputFiles()
{
	FILE* st = NULL;
	cout << "Reading file input_node.csv..."<< endl;

	fopen_s(&st,"input_node.csv","r");
	if(st!=NULL)
	{
		int i=0;
		CNode* pNode = 0;
		while(!feof(st))
		{
			int id			= g_read_integer(st);
			TRACE("node %d\n ", id);

			if(id == -1)  // reach end of file
				break;

			float x	= g_read_float(st);
			float y	= g_read_float(st);
			// Create and insert the node
			pNode = new CNode;
			pNode->m_NodeID = i;
			pNode->m_ZoneID = 0;
			g_NodeSet.insert(pNode);
			g_NodeMap[id] = pNode;
			g_NodeIDtoNameMap[i] = id;
			g_NodeNametoIDMap[id]= i;
			i++;
		}
		fclose(st);
	}else
	{
		cout << "Error: File node.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
	cout << "Reading file input_link.csv..."<< endl;

	fopen_s(&st,"input_link.csv","r");

	int i = 0;
	if(st!=NULL)
	{
		CLink* pLink = 0;
		while(!feof(st))
		{
			int FromID =  g_read_integer(st);
			if(FromID == -1)  // reach end of file
				break;
			int ToID = g_read_integer(st);

			pLink = new CLink(g_OptimizationHorizon);
			pLink->m_LinkID = i;
			pLink->m_FromNodeNumber = FromID;
			pLink->m_ToNodeNumber = ToID;
			pLink->m_FromNodeID = g_NodeMap[pLink->m_FromNodeNumber ]->m_NodeID;
			pLink->m_ToNodeID= g_NodeMap[pLink->m_ToNodeNumber]->m_NodeID;
			float length = g_read_float(st);
			pLink->m_NumLanes= g_read_integer(st);
			pLink->m_SpeedLimit= g_read_float(st);
			pLink->m_Length= max(length, pLink->m_SpeedLimit*0.1f/60.0f);  // minimum distance
			pLink->m_MaximumServiceFlowRatePHPL= g_read_float(st);
			pLink->m_LinkType= g_read_integer(st);

				g_NodeMap[pLink->m_FromNodeNumber ]->m_TotalCapacity += (pLink->m_MaximumServiceFlowRatePHPL* pLink->m_NumLanes);

			g_LinkSet.insert(pLink);
			g_LinkIDMap[i]  = pLink;
			i++;


		}
		fclose(st);
	}else
	{
		cout << "Error: File input_link.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}



	cout << "Number of Nodes = "<< g_NodeSet.size() << endl;
	cout << "Number of Links = "<< g_LinkSet.size() << endl;

	cout << "Running Time:" << g_GetAppRunningTime()  << endl;

	g_LogFile << "Number of Nodes = "<< g_NodeSet.size() << endl;
	g_LogFile << "Number of Links = "<< g_LinkSet.size() << endl;

}



void FreeMemory()
{
	std::set<CNode*>::iterator iterNode;
	std::set<CLink*>::iterator iterLink;

	std::vector<CTrain*>::iterator iterTrain;



	cout << "Free node set... " << endl;

	for (iterNode = g_NodeSet.begin(); iterNode != g_NodeSet.end(); iterNode++)
	{
		delete *iterNode;
	}

	g_NodeSet.clear();
	g_NodeMap.clear();
	g_NodeIDtoNameMap.clear();

	cout << "Free link set... " << endl;
	for (iterLink = g_LinkSet.begin(); iterLink != g_LinkSet.end(); iterLink++)
	{
		delete *iterLink;
	}

	g_LinkSet.clear();
	g_LinkIDMap.clear();

	cout << "Free train set... " << endl;
	for (iterTrain = g_TrainVector.begin(); iterTrain != g_TrainVector.end(); iterTrain++)
	{
		delete *iterTrain;
	}

}



int g_InitializeLogFiles()
{
	g_AppStartTime = CTime::GetCurrentTime();

	g_LogFile.open ("summary.log", ios::out);
	if (g_LogFile.is_open())
	{
		g_LogFile.width(12);
		g_LogFile.precision(3) ;
		g_LogFile.setf(ios::fixed);
	}else
	{
		cout << "File summary.log cannot be opened, and it might be locked by another program or the target data folder is read-only." << endl;
		cin.get();  // pause
		return 0;
	}

	cout << "FastTrain: A Fast Open-Source FT Simulation Engine"<< endl;
		cout << "sourceforge.net/projects/dtalite/"<< endl;
		cout << "Version 0.5, Release Date 02/14/2011."<< endl;

		g_LogFile << "---FastTrain: A Fast Open-Source Train Scheduling/Routing Engine---"<< endl;
		g_LogFile << "http://code.google.com/p/fast-train/"<< endl;
		g_LogFile << "Version 0.95, Release Date 01/15/2011."<< endl;

		return 1;
}

void g_ReadSchedulingSettings()
{
		TCHAR IniFilePath_FT[_MAX_PATH] = _T("./FTSettings.ini");

		// if  ./FTSettings.ini does not exit, then we should print out all the default settings for user to change
		//

		g_OptimizationMethod = g_GetPrivateProfileInt("optimization", "method", 0, IniFilePath_FT);	


		if(g_OptimizationMethod ==0)  //priority rule
		{

		}
}

void g_FreeMemory()
{
		cout << "Free memory... " << endl;
		FreeMemory();

		g_LogFile.close();

		g_LogFile << "Scheduling Completed. " << g_GetAppRunningTime() << endl;
}

void g_TransportationRoutingScheduling()
{
		ReadInputFiles();

}

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

		// initialize MFC and print and error on failure
	if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: MFC initialization failed\n"));
		nRetCode = 1;

		return 0;
	}

	if(g_InitializeLogFiles()==0) 
		return 0;


	g_ReadSchedulingSettings();

//		g_OutputSimulationStatistics();
		g_FreeMemory();
	return nRetCode;
}




