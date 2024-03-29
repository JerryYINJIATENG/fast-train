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

int g_OptimizationMethod = 0;   //0: Priority rule, //1: Lagrangian relaxation method

int g_MaxNumberOfLRIterations = 50;
int g_MaxNumberOfFeasibilityPenaltyIterations = 20;
int g_SafetyHeadway = 2;
int g_NumberOfTrainTypes=1;
int g_CellBasedNetworkFlag = 0;
int g_CellBasedOutputFlag = 0;
/*************************************
How to build a simple transportation scheduling and routing package

step 0: basic input functions
g_ReadInputFiles()

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
std::map<int, CNode*> g_NodeIDMap;
std::vector<CTrain*> g_TrainVector;
std::vector<CSimpleTrainData> g_SimpleTrainDataVector;  // for STL sorting


std::map<int, int> g_NodeIDToNumberMap;
std::map<int, int> g_NodeNumbertoIDMap;
std::map<unsigned long, CLink*> g_NodeIDtoLinkMap;

int g_OptimizationHorizon = 1440;
int g_LastestDepartureTime = 0;
int g_TotalFreeRunningTime = 0;

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


bool operator<(const CSimpleTrainData& train1, const CSimpleTrainData& train2)
{
  if(train1.m_TrainType  < train2.m_TrainType) 
	  return true;
	  
  if(train1.m_TrainType  == train2.m_TrainType && train1.m_EarlyDepartureTime < train2.m_EarlyDepartureTime) 
	  return true;

  return false;
};
void g_ReadInputFiles()
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
			int node_name			= g_read_integer(st);
			TRACE("node %d\n ", node_name);

			if(node_name == -1)  // reach end of file
				break;

			if(node_name>= MAX_PHYSICAL_NODE_NUMBER)
			{
				cout << "Error: Node number..."<< endl;
				g_ProgramStop();
			}

			float x	= g_read_float(st);
			float y	= g_read_float(st);
			// Create and insert the node
			pNode = new CNode;
			pNode->m_NodeID = i;
			pNode->m_SpacePosition = y;
			pNode->m_PhysicalNodeFlag = true;

			g_NodeSet.insert(pNode);
			g_NodeIDMap[i] = pNode;
			g_NodeIDToNumberMap[i] = node_name;
			g_NodeNumbertoIDMap[node_name]= i;
			i++;
		}
		fclose(st);
	}else
	{
		cout << "Error: File node.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
	cout << "Reading file input_link.csv..."<< endl;

	g_ReadTimetableCVSFile();

	fopen_s(&st,"input_link.csv","r");

	int i = 0;
	if(st!=NULL)
	{
		CLink* pLink = 0;
		while(!feof(st))
		{
			int FromNodeName =  g_read_integer(st);
			if(FromNodeName == -1)  // reach end of file
				break;
			int ToNodeName = g_read_integer(st);

			pLink = new CLink(g_OptimizationHorizon);
			pLink->m_LinkID = i;
			pLink->m_FromNodeNumber = FromNodeName;
			pLink->m_ToNodeNumber = ToNodeName;
			pLink->m_FromNodeID = g_NodeNumbertoIDMap[FromNodeName ];
			pLink->m_ToNodeID= g_NodeNumbertoIDMap[ToNodeName];
			pLink->m_Length= g_read_float(st);
			pLink->m_NumTracks= g_read_integer(st);
			pLink->m_SpeedLimit= g_read_float(st);
			pLink->m_TrackCapacity= g_read_float(st);
			pLink->m_LinkCapacity = pLink->m_NumTracks*pLink->m_TrackCapacity;
			pLink->m_LinkType= g_read_integer(st);
			pLink->m_PhysicalLinkFlag = true; 

			g_LinkSet.insert(pLink);
			g_LinkIDMap[i]  = pLink;
			unsigned long LinkKey = GetLinkKey( pLink->m_FromNodeID, pLink->m_ToNodeID);
			g_NodeIDtoLinkMap[LinkKey] = pLink;

			i++;


		}
		fclose(st);
	}else
	{
		cout << "Error: File input_link.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}


	g_ReadTrainProfileCSVFile();

	cout << "Number of Physical Nodes = "<< g_NodeSet.size() << endl;
	cout << "Number of Physical Links = "<< g_LinkSet.size() << endl;



	if(g_CellBasedNetworkFlag ==1) 
	{	// expand network for double tracks
		g_ExpandNetworkForDoubleTracks();

	cout << "Number of Nodes = "<< g_NodeSet.size() << endl;
	cout << "Number of Links = "<< g_LinkSet.size() << endl;

	}


	cout << "Number of Trains = "<< g_TrainVector.size() << endl;


	cout << "Running Time:" << g_GetAppRunningTime()  << endl;

	g_LogFile << "Number of Nodes = "<< g_NodeSet.size() << endl;
	g_LogFile << "Number of Links = "<< g_LinkSet.size() << endl;
	g_LogFile << "Number of Trains = "<< g_TrainVector.size() << endl;

}

void CreateCell(int FromNodeNumber, int ToNodeNumber, CLink* pLink , int CellNo)
{
	CLink* pCell = new CLink(g_OptimizationHorizon);

	pCell->m_LinkID = g_LinkSet.size ();
	pCell->m_FromNodeNumber = FromNodeNumber;
	pCell->m_ToNodeNumber = ToNodeNumber;
	pCell->m_FromNodeID = g_NodeNumbertoIDMap[FromNodeNumber ];
	pCell->m_ToNodeID= g_NodeNumbertoIDMap[ToNodeNumber];
	pCell->m_Length = pLink->m_Length/pLink->m_MinRunningTime;
	pCell->m_NumTracks= pLink->m_NumTracks;
	pCell->m_SpeedLimit= pLink->m_SpeedLimit;
	pCell->m_TrackCapacity= pLink->m_TrackCapacity;
	pCell->m_LinkCapacity = pLink->m_LinkCapacity;
	pCell->m_LinkType= pLink->m_LinkType;
	pCell->m_MinRunningTime = 1;
	pCell->m_PhysicalLinkFlag = false;   // virtual link -> cell

	g_LogFile << "Adding Cell " << FromNodeNumber << " -> " << ToNodeNumber  << " ID " << g_NodeNumbertoIDMap[FromNodeNumber ] << " -> " << g_NodeNumbertoIDMap[ToNodeNumber];

		for(int type = 1; type < MAX_TRAIN_TYPE; type++)
		{
			if(pLink->m_FreeRuningTimeAry[type] < MAX_TRAIN_RUNNING_TIME )
			{
				pCell->m_FreeRuningTimeAry[type] = int(CellNo*1.0/pLink->m_MinRunningTime*pLink->m_FreeRuningTimeAry[type]+0.5) - int((CellNo-1)*1.0/pLink->m_MinRunningTime*pLink->m_FreeRuningTimeAry[type]+0.5);
				pCell->m_MaxWaitingTimeAry[type] = 0;

				g_LogFile << "RT (" << type << ")" << pCell->m_FreeRuningTimeAry[type] << ";";

			}
		}

		g_LogFile << endl;

	g_LinkSet.insert(pCell);
	g_LinkIDMap[pCell->m_LinkID]  = pCell;
	unsigned long LinkKey = GetLinkKey( pCell->m_FromNodeID, pCell->m_ToNodeID);
	g_NodeIDtoLinkMap[LinkKey] = pCell;
}

void g_ExpandNetworkForDoubleTracks()
{
	std::set<CLink*>::iterator iterLink;
	for (iterLink = g_LinkSet.begin(); iterLink != g_LinkSet.end(); iterLink++)
	{
		CLink* pLink = (*iterLink);
		if(pLink->m_PhysicalLinkFlag == true && pLink->m_LinkType == 2)
		{

			//  construct new node set
			// keep all physical nodes
			int t;
			// add pLink->m_MinRunningTime - 1 nodes
			for (t = 1; t<=pLink->m_MinRunningTime-1; t++)
			{
				//  add virtual node between link for each min of min_running_time

				CNode* pNode = new CNode;
				int node_name = MAX_PHYSICAL_NODE_NUMBER*pLink->m_FromNodeNumber  + t;
				int id = g_NodeSet.size ();

				pNode->m_NodeID = id;
				pNode->m_SpacePosition = 0;
				pNode->m_PhysicalNodeFlag = false;

				g_NodeSet.insert(pNode);
				g_NodeIDMap[id] = pNode;
				g_NodeIDToNumberMap[id] = node_name;
				g_NodeNumbertoIDMap[node_name]= id;

			}

		//  construct new cells
		//  add virtual links between link for each min of min_running_time for type 1

			int FromNodeNumber;
			int ToNodeNumber;
			if(pLink->m_MinRunningTime >= 2)
			{
				//add first cell
				FromNodeNumber =  pLink->m_FromNodeNumber;
				ToNodeNumber = MAX_PHYSICAL_NODE_NUMBER*pLink->m_FromNodeNumber+1;

				CreateCell(FromNodeNumber,ToNodeNumber,pLink, 1);


				// add in-between cells
				for (t = 1; t<=pLink->m_MinRunningTime-2; t++)  //pLink->m_MinRunningTime -1 is the last virtual node along the link
				{
					FromNodeNumber =  MAX_PHYSICAL_NODE_NUMBER*pLink->m_FromNodeNumber + t;
					ToNodeNumber =    MAX_PHYSICAL_NODE_NUMBER*pLink->m_FromNodeNumber + t+1;

					CreateCell(FromNodeNumber,ToNodeNumber,pLink,t+1);

				}

				//add last cell
				FromNodeNumber =  1000*pLink->m_FromNodeNumber + pLink->m_MinRunningTime-1;
				ToNodeNumber =    pLink->m_ToNodeNumber;

				CreateCell(FromNodeNumber,ToNodeNumber,pLink,pLink->m_MinRunningTime);

			}

		}
		//  construct new input_train_cell_running_time.txt
		//  for each train_type_link record
		//  type 1: add one record for each min
		//  other types: nearest interger >=1, last cell: the remaining running time,
		//  add no wait flag for each non-first cell

	}


}
void g_ReadTrainProfileCSVFile()
{
	FILE* st = NULL;
	fopen_s(&st,"input_train_link_running_time.csv","r");


	if(st!=NULL)
	{
		CLink* pLink = 0;

		double default_distance_sum=0;
		double length_sum = 0;
		while(!feof(st))
		{
			int FromNodeNumber =  g_read_integer(st);
			if(FromNodeNumber == -1)  // reach end of file
				break;
			int ToNodeNumber = g_read_integer(st);

			CLink* pLink = g_FindLinkWithNodeNumbers(FromNodeNumber, ToNodeNumber);

			if(pLink!=NULL)
			{
				int TrainType = g_read_integer(st);
				int TrainRunningTime = g_read_integer(st);
				int TrainMaxWaitingTime = g_read_integer(st);
				pLink->m_FreeRuningTimeAry[TrainType] = TrainRunningTime;
				pLink->m_MaxWaitingTimeAry [TrainType] = TrainMaxWaitingTime;

				if(TrainType==1)
					g_TotalFreeRunningTime+= TrainRunningTime;

				if(pLink->m_MinRunningTime > TrainRunningTime)
					pLink->m_MinRunningTime = TrainRunningTime;


			}else
			{
				cout << "Error: Link " << FromNodeNumber <<  " -> " << ToNodeNumber << " in File input_train_link_running_time.csv has not been defined in input_link.csv."<< endl;
				g_ProgramStop();

			}
		}

		fclose(st);
	}else
	{
		cout << "Error: File input_train_link_running_time.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}
}

bool g_ReadTimetableCVSFile()
{
	FILE* st = NULL;
	fopen_s(&st,"input_timetable.csv","r");

	bool b_Initialized = false;
	if(st!=NULL)
	{
		int train_no = 0;
		while(!feof(st))
		{
			int train_id =  g_read_integer(st);

			if(train_id == -1)
				break;
			CTrain* pTrain = new CTrain();
			pTrain->m_TrainID = train_id;

			pTrain->m_TrainType =  g_read_integer(st);

			if( pTrain->m_TrainType > g_NumberOfTrainTypes)
				g_NumberOfTrainTypes = pTrain->m_TrainType;

			pTrain->m_OriginNodeNumber =  g_read_integer(st);
			pTrain->m_DestinationNodeNumber =  g_read_integer(st);
			pTrain->m_OriginNodeID =  g_NodeNumbertoIDMap[pTrain->m_OriginNodeNumber];
			pTrain->m_DestinationNodeID =  g_NodeNumbertoIDMap[pTrain->m_DestinationNodeNumber ];

			pTrain->m_EarlyDepartureTime=  g_read_integer(st);
			pTrain->m_AllowableSlackAtDeparture=  g_read_integer(st);

			pTrain->m_PreferredArrivalTime =  g_read_integer(st);
			pTrain->m_FixedRevenue =  g_read_float(st);
			pTrain->m_Incentive4EarlyDepartureFromOrigin =  g_read_float(st);
			pTrain->m_Incentive4EarlyArrrivalToDestination =  g_read_float(st);
			pTrain->m_CostPerUnitTimeStopped =  g_read_float(st);
			pTrain->m_NodeSize	=0;
			pTrain->m_ActualTripTime =  0;


			g_TrainVector.push_back(pTrain);

			CSimpleTrainData train_element;

			train_element.m_TrainNo = train_no;
			train_element.m_TrainType = pTrain->m_TrainType;
			train_element.m_EarlyDepartureTime = pTrain->m_EarlyDepartureTime;
			train_element.m_PreferredArrivalTime  = pTrain->m_PreferredArrivalTime ;

			g_SimpleTrainDataVector.push_back (train_element);

		train_no++;
		}

		std::sort(g_SimpleTrainDataVector.begin(), g_SimpleTrainDataVector.end());


		fclose(st);

	}else
	{
		cout << "Error: File input_train_link_running_time.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	return false;
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
	g_NodeIDMap.clear();
	g_NodeIDToNumberMap.clear();

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
	g_OptimizationHorizon= g_GetPrivateProfileInt("optimization", "optimization_horizon", 1440, IniFilePath_FT);	
	g_MaxNumberOfLRIterations = g_GetPrivateProfileInt("optimization", "max_num_of_LR_iterations", 20, IniFilePath_FT);	
	g_MaxNumberOfFeasibilityPenaltyIterations = g_GetPrivateProfileInt("optimization", "max_num_of_iterations_to_adding_feasiblity_penalty", 30, IniFilePath_FT);	
	g_SafetyHeadway = g_GetPrivateProfileInt("optimization", "safety_time_headway", 2, IniFilePath_FT);
	g_CellBasedNetworkFlag = g_GetPrivateProfileInt("optimization", "cell_based_network", 0, IniFilePath_FT);
	g_CellBasedOutputFlag =  g_GetPrivateProfileInt("optimization", "cell_based_output", 0, IniFilePath_FT);
}

bool g_ExportTimetableDataToCSVFile()
{
	FILE* st;
	fopen_s(&st,"output_timetable.csv","w");


	bool bOutputAllNodes = (bool)(g_CellBasedOutputFlag);

	if(st!=NULL)
	{
		fprintf(st, "train ID,train type,origin,destination,departure time,# of nodes, actual trip time,,node, time_stemp, node_position\n");
		g_LogFile << " ----------------" <<  endl;

		g_LogFile << "train ID,train type,origin,destination,departure time,# of nodes, actual trip time" <<  endl;

		float TotalTripTime = 0;

		for(unsigned int v = 0; v<g_TrainVector.size(); v++)
		{

			CTrain* pTrain = g_TrainVector[v];

			pTrain->m_ActualTripTime = pTrain->m_aryTN[pTrain->m_NodeSize -1].NodeArrivalTimestamp - pTrain->m_aryTN[0].NodeArrivalTimestamp;


			int n;
			int number_of_physical_nodes = 0;

			for( n = 0; n< pTrain->m_NodeSize; n++)
			{
				int NodeID = pTrain->m_aryTN[n].NodeID;

				if(g_NodeIDToNumberMap[NodeID]<MAX_PHYSICAL_NODE_NUMBER || bOutputAllNodes)
					number_of_physical_nodes++;

			}

			fprintf(st,"%d,%d,%d,%d,%d,%d,%d\n", pTrain->m_TrainID , pTrain->m_TrainType ,g_NodeIDToNumberMap [pTrain->m_OriginNodeID] ,g_NodeIDToNumberMap[pTrain->m_DestinationNodeID ],pTrain->m_EarlyDepartureTime ,
				number_of_physical_nodes,pTrain->m_ActualTripTime);

			g_LogFile << pTrain->m_TrainID << ", "<< pTrain->m_TrainType<< ", " << g_NodeIDToNumberMap [pTrain->m_OriginNodeID] << ", "<< g_NodeIDToNumberMap[pTrain->m_DestinationNodeID ] << ", "<< pTrain->m_EarlyDepartureTime << ", "<< number_of_physical_nodes << ", "<< pTrain->m_ActualTripTime <<  endl;

			TotalTripTime += pTrain->m_ActualTripTime ;

			for( n = 0; n< pTrain->m_NodeSize; n++)
			{
				int NodeID = pTrain->m_aryTN[n].NodeID;
				if(g_NodeIDToNumberMap[NodeID]<MAX_PHYSICAL_NODE_NUMBER || bOutputAllNodes)
				{
				fprintf(st,",,,,,,,,%d,%d,%5.2f\n", g_NodeIDToNumberMap[NodeID], pTrain->m_aryTN[n].NodeArrivalTimestamp,g_NodeIDMap[NodeID]->m_SpacePosition);
				}

			}
		}

		fclose(st);

		g_LogFile << "Total Trip Time =" << TotalTripTime << ",Avg Trip Time = " << TotalTripTime / max(1,g_TrainVector.size()) <<  endl;

		return true;
	}else
	{
		cout << "Error: File output_timetable.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	return false;
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
	g_ReadInputFiles();

	if(g_OptimizationMethod == 0)
		g_TimetableOptimization_Priority_Rule();

	if(g_OptimizationMethod == 1)
		g_TimetableOptimization_Lagrangian_Method();

	g_ExportTimetableDataToCSVFile();

	g_FreeMemory();
	return nRetCode;
}




