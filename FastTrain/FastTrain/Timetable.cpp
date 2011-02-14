// Timetable.cpp : implementation of timetabling algorithm in the CTLiteDoc class
//

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html


//    This file is part of FastTrain Version 3 (Open-source).

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

#include "stdafx.h"
#include "Network.h"
#include "FastTrain.h"
using namespace std;
#define _MAX_OPTIMIZATION_HORIZON 1440

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
void g_ReadTrainProfileCSVFile(LPCTSTR lpszFileName)
{
        FILE* st = NULL;
        fopen_s(&st,lpszFileName,"r");

        int i = 0;
        if(st!=NULL)
        {
                CLink* pLink = 0;

                double default_distance_sum=0;
                double length_sum = 0;
                while(!feof(st))
                {
                        int FromNode =  g_read_integer(st);
                        if(FromNode == -1)  // reach end of file
                                break;
                        int ToNode = g_read_integer(st);

                        int m_SimulationHorizon = 1;

                        CLink* pLink = g_FindLinkWithNodeNumbers(FromNode, ToNode);

                        if(pLink!=NULL)
                        {
                        int TrainType = g_read_integer(st);
                        int TrainRunningTime = g_read_integer(st);
                        pLink->m_RuningTimeMap[TrainType] = TrainRunningTime;
						i++;
                
                        }else
                        {
                        // to do: output error here
                        
                        }
                }

                fclose(st);
        }
}

bool g_ReadTimetableCVSFile(LPCTSTR lpszFileName)
{
	FILE* st = NULL;
	fopen_s(&st,lpszFileName,"r");

	bool b_Initialized = false;
	if(st!=NULL)
	{
		int train_no = 1;
		while(!feof(st))
		{

			int train_id =  g_read_integer(st);

			if(train_id == -1)
				break;
			CTrain* pTrain = new CTrain();
			pTrain->m_TrainID = train_id;

			pTrain->m_TrainType =  g_read_integer(st);
			pTrain->m_OriginNodeNumber =  g_read_integer(st);
			pTrain->m_DestinationNodeNumber =  g_read_integer(st);
			pTrain->m_OriginNodeID =  g_NodeNametoIDMap[pTrain->m_OriginNodeNumber];
			pTrain->m_DestinationNodeID =  g_NodeNametoIDMap[pTrain->m_DestinationNodeNumber ];

			pTrain->m_DepartureTime=  g_read_integer(st);
			pTrain->m_NodeSize	=  g_read_integer(st);
			pTrain->m_PreferredArrivalTime =  g_read_integer(st);
			pTrain->m_ActualTripTime =  g_read_integer(st);

			if(pTrain->m_NodeSize>0)
			{
				pTrain->m_aryTN = new STrainNode[pTrain->m_NodeSize];

				for(int i =0; i< pTrain->m_NodeSize; i++)
				{
					pTrain->m_aryTN[i].TaskProcessingTime = g_read_integer(st);
					pTrain->m_aryTN[i].TaskScheduleWaitingTime = g_read_integer(st);
					pTrain->m_aryTN[i].NodeTimestamp = g_read_integer(st);

					int NodeNumber = g_read_integer(st);
					pTrain->m_aryTN[i].NodeID = g_NodeNametoIDMap[NodeNumber];

					if(i>=1)
					{
						CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );

						pTrain->m_aryTN[i].LinkID = pLink->m_LinkID;
						if(pLink==NULL)
						{
							CString msg;
							msg.Format("train %d's link %d -> %d does not exist",pTrain->m_TrainID ,g_NodeIDtoNameMap[pTrain->m_aryTN[i-1].NodeID],g_NodeIDtoNameMap[pTrain->m_aryTN[i].NodeID]);
							
							fclose(st);
							return false;
						}

					}
				}
			}else
			{
				b_Initialized = false;
			}
			g_TrainVector.push_back(pTrain);

		}

//		m_TimetableDataLoadingStatus.Format ("%d trains are loaded from file %s.",g_TrainVector.size(),lpszFileName);

		fclose(st);

	}

	return false;
}





bool g_TimetableOptimization_Lagrangian_Method()
{
	NetworkForSP* m_pNetwork =NULL;

	int NumberOfIterationsWithMemory = 200;  // this is much greater than -100 when  LR_Iteration = 0, because  LastUseIterationNo is initialized as -100;


	int OptimizationHorizon = _MAX_OPTIMIZATION_HORIZON;  // we need to dynamically determine the optimization 


	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
	// reset resource usage counter for each timestamp
		(*iLink)->ResetResourceAry(OptimizationHorizon);
	}

	int max_number_of_LR_iterations = 100;
	// first loop for each LR iteration
	for(int LR_Iteration = 0; LR_Iteration< max_number_of_LR_iterations; LR_Iteration++)
	{

		TRACE ("Lagrangian Iteration %d", LR_Iteration);

		// step 1. reset resource usage counter for each timestamp
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{
			for(int t=0; t< OptimizationHorizon; t++)
			{
				(*iLink)->m_ResourceAry[t].UsageCount =0;
			}
		}


		// // step 2. for each train, record their resource usage on the corresponding link
		unsigned int v;
		for(v = 0; v<g_TrainVector.size(); v++)
		{
			CTrain* pTrain = g_TrainVector[v];

			// start from n=1, as only elements from n=1 to m_NodeSize hold link information, the first node element has no link info
			for(int n = 1; n< pTrain->m_NodeSize; n++)
			{
				CLink* pLink = g_LinkIDMap[pTrain->m_aryTN[n].LinkID];

				// inside loop for each link traveled by each tran
				for(int t = pTrain->m_aryTN[n-1].NodeTimestamp; t< pTrain->m_aryTN[n].NodeTimestamp; t++)
				{

					ASSERT(t>=0 && t<OptimizationHorizon);
					pLink->m_ResourceAry[t].UsageCount+=1;
					pLink->m_ResourceAry[t].LastUseIterationNo = LR_Iteration;

				}

			}
		}

		//step 3: subgradient algorithm
		//MSA as step size, use subgradient
		float StepSize = 1.0f/(LR_Iteration+1.0f);

		if(StepSize< 0.05f)  //keep the minimum step size
			StepSize = 0.05f;   


		// step 4. resource pricing algorithm
		// reset resource usage counter for each timestamp
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{
			for(int t=0; t< OptimizationHorizon; t++)
			{
				(*iLink)->m_ResourceAry[t].Price  += StepSize*((*iLink)->m_ResourceAry[t].UsageCount - (*iLink)->m_LaneCapacity);

				//			if((*iLink)->m_ResourceAry[t].Price > 0)
				//				TRACE("\n arc %d, time %d, price %f", (*iLink)->m_LinkID, t, (*iLink)->m_ResourceAry[t].Price );

				// if the total usage (i.e. resource consumption > capacity constraint) 
				// then the resource price increases, otherwise decrease

				if((*iLink)->m_ResourceAry[t].Price < 0 || LR_Iteration - ((*iLink)->m_ResourceAry[t].LastUseIterationNo) > NumberOfIterationsWithMemory)
					(*iLink)->m_ResourceAry[t].Price = 0;
			}
		}

		// step 5. build time-dependent network with resource price

		// here we allocate OptimizationHorizon time and cost labels for each node

		if(LR_Iteration==0) //only allocate once
			m_pNetwork = new NetworkForSP(g_NodeSet.size(), g_LinkSet.size(), OptimizationHorizon, 1);  

		// step 6. for each train (of the subproblem), find the subprogram solution (time-dependent path) so we know its path and timetable
		for(v = 0; v<g_TrainVector.size(); v++)
		{

			CTrain* pTrain = g_TrainVector[v];

			//step 6.1 find time-dependent shortest path with resource price label
			m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain->m_TrainType );
			//step 6.2 perform shortest path algorithm
			m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_DepartureTime );
			//step 6.3 fetch the train path  solution
			pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_DepartureTime, pTrain->m_DestinationNodeID,pTrain);


			//find the link no along the path

			for (int i=1; i< pTrain->m_NodeSize ; i++)
			{
				CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
				ASSERT(pLink!=NULL);
				pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

			}


		}

	}


	if(m_pNetwork !=NULL)     // m_pNetwork is used to calculate time-dependent generalized least cost path 
		delete m_pNetwork;

	return true;
}



void g_Initializetimetable()
{
	NetworkForSP* m_pNetwork =NULL;

	// this function only find the intial path for each train, and layout the initial (but infeasible timetable)
	// this is a simplifed version of Lagrangian method without actual resource price

	// we have to estimate the optimization horizion here, as it is used in the shortst path algorithm
	int OptimizationHorizon = _MAX_OPTIMIZATION_HORIZON;

	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
	// reset resource usage counter for each timestamp
		(*iLink)->ResetResourceAry(OptimizationHorizon);
	}

	// build time-dependent network with resource price

	// here we allocate OptimizationHorizon time and cost labels for each node

	m_pNetwork = new NetworkForSP(g_NodeSet.size(), g_LinkSet.size(), OptimizationHorizon, 1);  

	unsigned int v;
	for(v = 0; v<g_TrainVector.size(); v++)
	{

		CTrain* pTrain = g_TrainVector[v];

		//find time-dependent shortest path with time label
		m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain->m_TrainType );
		m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_DepartureTime );
		pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_DepartureTime, pTrain->m_DestinationNodeID,pTrain);


		for (int i=1; i< pTrain->m_NodeSize ; i++)
		{
			CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
			ASSERT(pLink!=NULL);
			pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

		}

	}
	if(m_pNetwork !=NULL)
		delete m_pNetwork;

}
bool g_TimetableOptimization_Priority_Rule()
{
	// this algorithm schedules a train at a time, and record the use of resource so that the following trains cannot use the previously consumped resource, which leads to a feasible solution
	// the priority of trains are assumed to be given from the train sequence of the input file timetabl.csv
	NetworkForSP* m_pNetwork =NULL;


	int OptimizationHorizon = _MAX_OPTIMIZATION_HORIZON;

	// step 1. Initialization

	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
	// reset resource usage counter for each timestamp
		(*iLink)->ResetResourceAry(OptimizationHorizon);
	}
	// step 2. for each train, record their resource usage on the corresponding link
	// outer loop: for each train

		m_pNetwork = new NetworkForSP(g_NodeSet.size(), g_LinkSet.size(), OptimizationHorizon, 1);  

		unsigned int v;
	for(v = 0; v<g_TrainVector.size(); v++)
	{

		CTrain* pTrain = g_TrainVector[v];

		//find time-dependent shortest path with time label
		m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain->m_TrainType );
		m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_DepartureTime );
		pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_DepartureTime, pTrain->m_DestinationNodeID,pTrain);


		for (int i=1; i< pTrain->m_NodeSize ; i++)
		{
			CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
			ASSERT(pLink!=NULL);
			pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

		}

		for (int n=1; n< pTrain->m_NodeSize ; n++)
		{		//for each used timestamp
			CLink* pLink = g_LinkIDMap[pTrain->m_aryTN[n].LinkID];

		for(int t = pTrain->m_aryTN[n-1].NodeTimestamp; t< pTrain->m_aryTN[n].NodeTimestamp; t++)
		{
				ASSERT(t>=0 && t<OptimizationHorizon);
				pLink->m_ResourceAry[t].UsageCount+=1;
				// check resource usage counter for each timestamp,
				if(pLink->m_ResourceAry[t].UsageCount >= pLink->m_LaneCapacity)  //over capacity
				{
					pLink->m_ResourceAry[t].Price  = MAX_SPLABEL;  // set the maximum price so the followers cannot use this time
				}
			}
		}

	}

	if(m_pNetwork !=NULL)
		delete m_pNetwork;

	return true;

}



