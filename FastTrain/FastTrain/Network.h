//  Portions Copyright 2010 Xuesong Zhou (xzhou99@gmail.com)

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html


//    This file is part of FastTrain  (Open-source).

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
#pragma once

#include <math.h>
#include <deque>
#include <map>
#include <set>
#include <iostream>
#include <vector>
#include <list>

using namespace std;
#define PI 3.1415926

#define	MAX_SPLABEL 99999
#define MAX_NODE_SIZE_IN_A_PATH 4000
#define MAX_TRAIN_TYPE 5
#define MAX_TRAIN_RUNNING_TIME 600
#define MAX_PHYSICAL_NODE_NUMBER 1000


extern void g_ProgramStop();

struct GDPoint
{
	float x;
	float y;
};


class CNode
{
public:
	CNode(){
		m_NodeNumber = 0;
		m_Connections = 0;
	};
	~CNode(){};
	GDPoint pt;
	int m_NodeNumber;  //  original node number
	int m_NodeID;  ///id, starting from zero, continuous sequence
	int m_Connections;  // number of connections
	float m_SpacePosition;
	bool m_PhysicalNodeFlag;

};


class CapacityReduction
{
public:
	float StartTime;
	float EndTime;
	float LaneClosureRatio;
	float SpeedLimit;
	float ServiceFlowRate;
};
class STrainNode
{  public:
int  NodeID; 
int  LinkID; // starting from the second element, [i-1,i]

int TaskProcessingTime;  // the task is associated with the previous arc, // first node: origin node release time, other nodes: station entry or exit time, 1, 3, 5,: entry station, 2, 4, 6: exit station
int TaskScheduleWaitingTime;  // to be scheduled by the program
int NodeDepartureTimestamp;  // to be calculated 
int NodeArrivalTimestamp;  // to be calculated 

};


class SResource
{  
public:
	int UsageCount;
	float Price;
	int LastUseIterationNo;



	SResource()
	{
		UsageCount = 0;
		Price = 0;
		LastUseIterationNo = -100;
	}



};

class CSimpleTrainData  // for train sorting
{
public:

	int m_TrainNo;  //range: +2,147,483,647
	int m_TrainType; // e.g. 1: high-speed train, 2: medium-speed train
	int m_EarlyDepartureTime;   // given input
	int m_PreferredArrivalTime;  // determined by the schedule




};

class CTrain  // for train timetabling
{
public:

	int m_TrainID;  //range: +2,147,483,647
	int m_TrainType; // e.g. 1: high-speed train, 2: medium-speed train
	int m_OriginNodeNumber;  // defined from node.csv
	int m_DestinationNodeNumber; // defined from node.csv
	int m_OriginNodeID;  // internal node id, for shortest path
	int m_DestinationNodeID; // internal node id, for shortest path
	int m_EarlyDepartureTime;   // given input
	int m_AllowableSlackAtDeparture;   // given input
	int m_PreferredArrivalTime;  // determined by the schedule
	int m_FixedRevenue;
	int m_Incentive4EarlyDepartureFromOrigin;
	int m_Incentive4EarlyArrrivalToDestination;
	int m_CostPerUnitTimeStopped;

	int m_ActualTripTime;  // determined by the schedule
	int m_NodeSize;        // initial value could be 0, given from extern input or calculated from shortest path algorithm
	STrainNode *m_aryTN; // node list arrary of a vehicle path


	CTrain()
	{
		m_NodeSize = 0;
		m_AllowableSlackAtDeparture = 20;
		m_aryTN = NULL;
	}
	~CTrain()
	{
		if(m_aryTN != NULL)
			delete m_aryTN;
	}

};
class CLink
{
public:

	CLink(int TimeHorizon)  // TimeHorizon's unit: per min
	{
		m_ResourceAry = NULL;
		m_MinRunningTime = 9999;

		for(int type = 0; type < MAX_TRAIN_TYPE; type++)
		{
		m_FreeRuningTimeAry[type] = MAX_TRAIN_RUNNING_TIME;
		m_MaxWaitingTimeAry[type] = 0;

		}
	};

	int m_LinkID;
	int m_FromNodeID;  // index starting from 0
	int m_ToNodeID;    // index starting from 0

	int m_FromNodeNumber;
	int m_ToNodeNumber;

	float	m_Length;  // in miles
	int		m_NumTracks;
	float	m_SpeedLimit;
	float m_LinkCapacity;
	float	m_TrackCapacity;  //Capacity used in BPR for each link, reduced due to link type and other factors.
	int m_LinkType;  // 1: single track, 2: double track
	int m_MinRunningTime;
	bool m_PhysicalLinkFlag;

	/* For min-by-min train timetabling, m_LinkCapacity is 1 for each min. 
	Example in airspace scheduling
	a sector is a volume of airspace for which a single air traffic control team has responsibility.
	The number of aircraft that can safely occupy a sector simultaneously is determined by controllers. 
	A typical range is around 8-15. 
	During a 15 minute time interval in US en-route airspace, 
	the typical upper bound limits of the order of 15-20.

	*/

	//for timetabling use
	int m_FreeRuningTimeAry[MAX_TRAIN_TYPE];  //indexed by train type
	int m_MaxWaitingTimeAry[MAX_TRAIN_TYPE];  //indexed by train type

	int GetTrainRunningTime(int TrainType)
	{
		if(m_NumTracks<0.001 || TrainType >=MAX_TRAIN_TYPE)
			return 600;  // use a default value in case user inputs lane capacity as zero

		return m_FreeRuningTimeAry[TrainType];
	}

	int GetTrainMaxWaitingTime(int TrainType)
	{
		return m_MaxWaitingTimeAry[TrainType];
	}

	GDPoint m_FromPoint, m_ToPoint;
	SResource *m_ResourceAry;
	void ResetResourceAry(int OptimizationHorizon)
	{
		if(m_ResourceAry!=NULL)
			delete m_ResourceAry;

		m_ResourceAry = new SResource[OptimizationHorizon];

			for(int t=0; t< OptimizationHorizon; t++)
			{
				m_ResourceAry[t].Price =0;
			}

	}


	std::vector<CapacityReduction> CapacityReductionVector;

	~CLink(){
		if(m_ResourceAry!=NULL) delete m_ResourceAry;
	};

};


template <typename T>
T **AllocateDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new T*[nRows];

	for( int i = 0 ; i < nRows ; i++ )
	{
		dynamicArray[i] = new T [nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray,int nRows, int nCols)
{
	for(int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete [] dArray;

}


template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficent memory.";
		g_ProgramStop();
	}

	for( int x = 0 ; x < nX ; x++ )
	{
		dynamicArray[x] = new T* [nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

		for( int y = 0 ; y < nY ; y++ )
		{
			dynamicArray[x][y] = new T [nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficent memory.";
				g_ProgramStop();
			}
		}
	}

	return dynamicArray;

}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	for(int x = 0; x < nX; x++)
	{
		for(int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}

class NetworkForSP  // mainly for shortest path calculation, not just physical network
	// for shortes path calculation between zone centroids, for origin zone, there are only outgoing connectors, for destination zone, only incoming connectors
	// different shortest path calculations have different network structures, depending on their origions/destinations
{
public:
	int m_OptimizationIntervalSize;
	int m_NodeSize;
	int m_PhysicalNodeSize;
	int m_OptimizationHorizon;
	int m_OptimizationTimeInveral;
	int m_ListFront;
	int m_ListTail;
	int m_LinkSize;

	int* m_LinkList;  // dimension number of nodes

	int** m_OutboundNodeAry; //Outbound node array
	int** m_OutboundLinkAry; //Outbound link array
	int* m_OutboundSizeAry;  //Number of outbound links

	int** m_InboundLinkAry; //inbound link array
	int* m_InboundSizeAry;  //Number of inbound links

	int* m_FromIDAry;
	int* m_ToIDAry;

	float** m_LinkTDTimeAry;
	float** m_LinkTDCostAry;
	float* m_LinkTDMaxWaitingTimeAry;

	int* NodeStatusAry;                // Node status array used in KSP;

	float* LabelTimeAry;               // label - time
	int* NodePredAry;
	float* LabelCostAry;

	//below are time-dependent cost label and predecessor arrays
	float** TD_LabelCostAry;
	int** TD_NodePredAry;  // pointer to previous NODE INDEX from the current label at current node and time
	int** TD_TimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time


	int m_Number_of_CompletedVehicles;
	int m_AdjLinkSize;

	NetworkForSP(int NodeSize, int LinkSize, int TimeHorizon, int TimeInterval){
		m_AdjLinkSize = 15;
		m_NodeSize = NodeSize;
		m_LinkSize = LinkSize;

		m_OptimizationHorizon = TimeHorizon;
		m_OptimizationTimeInveral = TimeInterval;



		m_OutboundSizeAry = new int[m_NodeSize];
		m_InboundSizeAry = new int[m_NodeSize];


		m_OutboundNodeAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_OutboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_InboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);


		m_LinkList = new int[m_NodeSize];

		m_OptimizationIntervalSize = int(m_OptimizationHorizon/m_OptimizationTimeInveral+0.1);  // make sure there is no rounding error
		m_LinkTDTimeAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);
		m_LinkTDMaxWaitingTimeAry    =  new float[m_LinkSize];

		m_LinkTDCostAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);

		m_FromIDAry = new int[m_LinkSize];

		m_ToIDAry = new int[m_LinkSize];

		NodeStatusAry = new int[m_NodeSize];                    // Node status array used in KSP;
		NodePredAry = new int[m_NodeSize];
		LabelTimeAry = new float[m_NodeSize];                     // label - time
		LabelCostAry = new float[m_NodeSize];                     // label - cost

		TD_LabelCostAry =  AllocateDynamicArray<float>(m_NodeSize,m_OptimizationIntervalSize);
		TD_NodePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);
		TD_TimePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);


		if(m_OutboundSizeAry==NULL || m_LinkList==NULL || m_FromIDAry==NULL || m_ToIDAry==NULL  ||
			NodeStatusAry ==NULL || NodePredAry==NULL || LabelTimeAry==NULL || LabelCostAry==NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

	};

	NetworkForSP();

	void Init(int NodeSize, int LinkSize, int TimeHorizon,int AdjLinkSize)
	{
		m_NodeSize = NodeSize;
		m_LinkSize = LinkSize;

		m_OptimizationHorizon = TimeHorizon;
		m_AdjLinkSize = AdjLinkSize;


		m_OutboundSizeAry = new int[m_NodeSize];
		m_InboundSizeAry = new int[m_NodeSize];


		m_OutboundNodeAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_OutboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_InboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);


		m_LinkList = new int[m_NodeSize];

		m_OptimizationIntervalSize = int(TimeHorizon/m_OptimizationTimeInveral)+1;  // make sure it is not zero
		m_LinkTDTimeAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);
		m_LinkTDCostAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);

		m_FromIDAry = new int[m_LinkSize];

		m_ToIDAry = new int[m_LinkSize];

		NodeStatusAry = new int[m_NodeSize];                    // Node status array used in KSP;
		NodePredAry = new int[m_NodeSize];
		LabelTimeAry = new float[m_NodeSize];                     // label - time
		LabelCostAry = new float[m_NodeSize];                     // label - cost


		TD_LabelCostAry =  AllocateDynamicArray<float>(m_NodeSize,m_OptimizationIntervalSize);
		TD_NodePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);
		TD_TimePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);


		if(m_OutboundSizeAry==NULL || m_LinkList==NULL || m_FromIDAry==NULL || m_ToIDAry==NULL  ||
			NodeStatusAry ==NULL || NodePredAry==NULL || LabelTimeAry==NULL || LabelCostAry==NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

	};


	~NetworkForSP()
	{
		if(m_OutboundSizeAry)  delete m_OutboundSizeAry;
		if(m_InboundSizeAry)  delete m_InboundSizeAry;

		DeallocateDynamicArray<int>(m_OutboundNodeAry,m_NodeSize, m_AdjLinkSize);
		DeallocateDynamicArray<int>(m_OutboundLinkAry,m_NodeSize, m_AdjLinkSize);
		DeallocateDynamicArray<int>(m_InboundLinkAry,m_NodeSize, m_AdjLinkSize);


		if(m_LinkList) delete m_LinkList;

		if(m_LinkTDMaxWaitingTimeAry)
			delete m_LinkTDMaxWaitingTimeAry;

		DeallocateDynamicArray<float>(m_LinkTDTimeAry,m_LinkSize,m_OptimizationIntervalSize);
   
		DeallocateDynamicArray<float>(m_LinkTDCostAry,m_LinkSize,m_OptimizationIntervalSize);

		DeallocateDynamicArray<float>(TD_LabelCostAry,m_NodeSize,m_OptimizationIntervalSize);
		DeallocateDynamicArray<int>(TD_NodePredAry,m_NodeSize,m_OptimizationIntervalSize);
		DeallocateDynamicArray<int>(TD_TimePredAry,m_NodeSize,m_OptimizationIntervalSize);

		if(m_FromIDAry)		delete m_FromIDAry;
		if(m_ToIDAry)	delete m_ToIDAry;

		if(NodeStatusAry) delete NodeStatusAry;                 // Node status array used in KSP;
		if(NodePredAry) delete NodePredAry;
		if(LabelTimeAry) delete LabelTimeAry;
		if(LabelCostAry) delete LabelCostAry;



	};

	void BuildSpaceTimeNetworkForTimetabling(std::set<CNode*>* p_NodeSet, std::set<CLink*>* p_LinkSet, int TrainType);

	bool SimplifiedTDLabelCorrecting_DoubleQueue(int origin, int departure_time, int vehicle_type);   // Pointer to previous node (node)
	// simplifed version use a single node-dimension of LabelCostAry, NodePredAry

	//these two functions are for timetabling
	bool OptimalTDLabelCorrecting_DoubleQueue(int origin, int departure_time, int destination, int AllowableSlackAtDeparture, int CostPerUnitTimeStopped);
	// optimal version use a time-node-dimension of TD_LabelCostAry, TD_NodePredAry
    int FindOptimalSolution(int origin,  int departure_time,  int destination, CTrain* pTrain, float& TripPrice);
	// return node arrary from origin to destination, return travelling timestamp at each node
	// return number_of_nodes in path

    int FindInitiallSolution(int origin,  int departure_time,  int destination, CTrain* pTrain);

	// SEList: Scan List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_ListFront= -1;
		m_ListTail= -1;
	}

	void SEList_push_front(int node)
	{
		if(m_ListFront == -1)  // start from empty
		{
			m_LinkList[node] = -1;
			m_ListFront  = node;
			m_ListTail  = node;
		}
		else
		{
			m_LinkList[node] = m_ListFront;
			m_ListFront  = node;
		}

	}
	void SEList_push_back(int node)
	{
		if(m_ListFront == -1)  // start from empty
		{
			m_ListFront = node;
			m_ListTail  = node;
			m_LinkList[node] = -1;
		}
		else
		{
			m_LinkList[m_ListTail] = node;
			m_LinkList[node] = -1;
			m_ListTail  = node;
		}
	}

	bool SEList_empty()
	{
		return(m_ListFront== -1);
	}

	int SEList_front()
	{
		return m_ListFront;
	}

	void SEList_pop_front()
	{
		int tempFront = m_ListFront;
		m_ListFront = m_LinkList[m_ListFront];
		m_LinkList[tempFront] = -1;
	}

	int  GetLinkNoByNodeIndex(int usn_index, int dsn_index);

};

// for train timetabling
// node element of a train path


#pragma warning(disable:4244)  // stop warning: "conversion from 'int' to 'float', possible loss of data"
// Stop bugging me about this, live isn't perfect


