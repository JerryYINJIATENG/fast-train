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
#include <iostream>
#include <fstream>
#include "Network.h"
#include "FastTrain.h"
using namespace std;
#define _MAX_OPTIMIZATION_HORIZON 1440

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

bool g_TimetableOptimization_Lagrangian_Method()
{
	NetworkForSP* m_pNetwork =NULL;

	int NumberOfIterationsWithMemory = 5;  // this is much greater than -100 when  LR_Iteration = 0, because  LastUseIterationNo is initialized as -100;

	int OptimizationHorizon = g_OptimizationHorizon;  // we need to dynamically determine the optimization 

	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		// reset resource usage counter for each timestamp
		(*iLink)->ResetResourceAry(OptimizationHorizon);
	}

	// first loop for each LR iteration
	for(int LR_Iteration = 0; LR_Iteration< g_MaxNumberOfLRIterations + g_MaxNumberOfFeasibilityPenaltyIterations; LR_Iteration++)
	{

		TRACE ("Lagrangian Iteration %d", LR_Iteration);
		cout << "Lagrangian Iteration " << LR_Iteration << endl;

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

				if(g_CellBasedNetworkFlag == 1)   // cell based representation, only apply to node departure time
				{
					// inside loop for each link traveled by each tran
					for(int t = pTrain->m_aryTN[n-1].NodeArrivalTimestamp-g_SafetyHeadway; t<= pTrain->m_aryTN[n-1].NodeArrivalTimestamp + g_SafetyHeadway; t++)
					{
						if(t>=0 && t<OptimizationHorizon)
						{
							pLink->m_ResourceAry[t].UsageCount+=1;
							pLink->m_ResourceAry[t].LastUseIterationNo = LR_Iteration;

							if(pLink->m_ResourceAry[t].UsageCount > pLink->m_LinkCapacity)
							{
								TRACE("");
							}

						}

					}
				}

				if(g_CellBasedNetworkFlag == 0)  // general capacity constraint
				{
					// inside loop for each link traveled by each tran
					for(int t = pTrain->m_aryTN[n-1].NodeArrivalTimestamp; t<=pTrain->m_aryTN[n].NodeArrivalTimestamp; t++)
					{
						if(t>=0 && t<OptimizationHorizon)
						{
							pLink->m_ResourceAry[t].UsageCount+=1;
							pLink->m_ResourceAry[t].LastUseIterationNo = LR_Iteration;
						}

					}
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

		bool bFeasibleFlag = true;
		for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
		{
			for(int t=0; t< OptimizationHorizon; t++)
			{
				if((*iLink)->m_ResourceAry[t].UsageCount > (*iLink)->m_LinkCapacity)
				{
					bFeasibleFlag = false;
//					TRACE("\n arc %d --> %d, time %d, capacity is exceeded", (*iLink)->m_FromNodeNumber , (*iLink)->m_ToNodeNumber ,t);
				}

		if(LR_Iteration < g_MaxNumberOfLRIterations)
		{
				(*iLink)->m_ResourceAry[t].Price  += StepSize*((*iLink)->m_ResourceAry[t].UsageCount - (*iLink)->m_LinkCapacity);
		}else 
		{   //adding feasiblity penalty
				
			if((*iLink)->m_ResourceAry[t].UsageCount > (*iLink)->m_LinkCapacity)
			{
				(*iLink)->m_ResourceAry[t].Price  += StepSize*((*iLink)->m_ResourceAry[t].UsageCount - (*iLink)->m_LinkCapacity);
			}

		}

				if((*iLink)->m_ResourceAry[t].Price > 0)
					TRACE("\n arc %d --> %d, time %d, price %f", (*iLink)->m_FromNodeNumber , (*iLink)->m_ToNodeNumber ,t, (*iLink)->m_ResourceAry[t].Price );

				// if the total usage (i.e. resource consumption > capacity constraint) 
				// then the resource price increases, otherwise decrease

				if((*iLink)->m_ResourceAry[t].Price < 0 || (LR_Iteration - (*iLink)->m_ResourceAry[t].LastUseIterationNo) > NumberOfIterationsWithMemory)
					(*iLink)->m_ResourceAry[t].Price = 0;
			}
		}

		// step 5. build time-dependent network with resource price

		// here we allocate OptimizationHorizon time and cost labels for each node

		if(LR_Iteration==0) //only allocate once
			m_pNetwork = new NetworkForSP(g_NodeSet.size(), g_LinkSet.size(), OptimizationHorizon, 1);  

		// step 6. for each train (of the subproblem), find the subprogram solution (time-dependent path) so we know its path and timetable
		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		
		for(v = 0; v<g_TrainVector.size(); v++)
		{

			CTrain* pTrain = g_TrainVector[v];

			//step 6.1 find time-dependent shortest path with resource price label
			m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain->m_TrainType );
			//step 6.2 perform shortest path algorithm
			m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_EarlyDepartureTime,pTrain->m_DestinationNodeID , pTrain->m_AllowableSlackAtDeparture, pTrain->m_CostPerUnitTimeStopped );
			//step 6.3 fetch the train path  solution
			float TripPrice  = 0;
			pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_EarlyDepartureTime, pTrain->m_DestinationNodeID,pTrain,TripPrice);
			TotalTripPrice += TripPrice;
			TotalTravelTime += pTrain->m_ActualTripTime;


			//find the link no along the path

			for (int i=1; i< pTrain->m_NodeSize ; i++)
			{
				CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
				ASSERT(pLink!=NULL);
				pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

			}


		}

			float TotalResourcePrice = 0;
			for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			{
				int t;
				for(t=0; t< OptimizationHorizon; t++)
				{
					TotalResourcePrice += (*iLink)->m_ResourceAry[t].Price ;
				}

			}

			g_LogFile << "Iteration: " << LR_Iteration << ",Total Travel Time:" << TotalTravelTime << "Total Trip Price:"<< TotalTripPrice << ",Total Resource Price: " << TotalResourcePrice << ",Total Price:" << TotalTripPrice-TotalResourcePrice << endl;

		//final export to log file at last iteration
		if(	LR_Iteration+1 >= g_MaxNumberOfLRIterations)
		{
		int infeasibility_count  = 0;

			for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			{
				int t;
				for(t=0; t< OptimizationHorizon; t++)
				{
					if((*iLink)->m_ResourceAry[t].UsageCount > (*iLink)->m_LinkCapacity)
					{
						g_LogFile << "Capacity is exceeded: Link" << (*iLink)->m_FromNodeNumber << "->" <<  (*iLink)->m_ToNodeNumber << ",Time: " << t <<", Usage Counter = " <<(*iLink)->m_ResourceAry[t].UsageCount << ", Capacity = " << (*iLink)->m_LinkCapacity<< endl;
						infeasibility_count++;
					}
				}
			}

			float TotalPrice = 0;
			for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			{
				int t;
				for(t=0; t< OptimizationHorizon; t++)
				{
					if((*iLink)->m_ResourceAry[t].Price > 0)
					{
						g_LogFile << "Price > 0 : arc" << (*iLink)->m_FromNodeNumber << "->" <<  (*iLink)->m_ToNodeNumber << ",Time: " << t << ", Price: " <<(*iLink)->m_ResourceAry[t].Price  <<", Usage Counter = " <<(*iLink)->m_ResourceAry[t].UsageCount << ", Capacity = " << (*iLink)->m_LinkCapacity<< endl;

					}
				}
			}

						g_LogFile << "Total Lagrangian price = " << TotalPrice << endl;
						g_LogFile << "Total Count of Infeasibility = " << infeasibility_count << endl;

		
		if(infeasibility_count ==0)
		{
			g_LogFile << "Feasible solution is found. Search ends!" << endl;
			break;
		}
		
		}



	}


	if(m_pNetwork !=NULL)     // m_pNetwork is used to calculate time-dependent generalized least cost path 
		delete m_pNetwork;

	return true;
}


bool g_TimetableOptimization_Priority_Rule()
{
	// this algorithm schedules a train at a time, and record the use of resource so that the following trains cannot use the previously consumped resource, which leads to a feasible solution
	// the priority of trains are assumed to be given from the train sequence of the input file timetabl.csv

	int OptimizationHorizon = _MAX_OPTIMIZATION_HORIZON;  // we need to dynamically determine the optimization 

	NetworkForSP* m_pNetwork =NULL;
		//only allocate once
			m_pNetwork = new NetworkForSP(g_NodeSet.size(), g_LinkSet.size(), OptimizationHorizon, 1);  

	float TotalTripPrice  = 0;


	std::set<CLink*>::iterator iLink;
	for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
	{
		// reset resource usage counter for each timestamp
		(*iLink)->ResetResourceAry(OptimizationHorizon);
	}

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
		for(v = 0; v<g_SimpleTrainDataVector.size(); v++)
		{
			CTrain* pTrain = g_TrainVector[v];

			// step 2.1. build time-dependent network with resource price

			// here we allocate OptimizationHorizon time and cost labels for each node

			//step 2.1 find time-dependent shortest path with resource price label
			m_pNetwork->BuildSpaceTimeNetworkForTimetabling(&g_NodeSet, &g_LinkSet, pTrain->m_TrainType );
			//step 2.2 perform shortest path algorithm
			m_pNetwork->OptimalTDLabelCorrecting_DoubleQueue(pTrain->m_OriginNodeID , pTrain->m_EarlyDepartureTime,pTrain->m_DestinationNodeID , pTrain->m_AllowableSlackAtDeparture, pTrain->m_CostPerUnitTimeStopped );
			//step 3.3 fetch the train path  solution
			float TripPrice = 0;
			pTrain->m_NodeSize = m_pNetwork->FindOptimalSolution(pTrain->m_OriginNodeID , pTrain->m_EarlyDepartureTime, pTrain->m_DestinationNodeID,pTrain,TripPrice);
			TotalTripPrice+=TripPrice;


			//find the link no along the path

			for (int i=1; i< pTrain->m_NodeSize ; i++)
			{
				CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i-1].NodeID , pTrain->m_aryTN[i].NodeID  );
				ASSERT(pLink!=NULL);
				pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

			}


			// step 3 record resource usage
			// start from n=1, as only elements from n=1 to m_NodeSize hold link information, the first node element has no link info
			for(int n = 1; n< pTrain->m_NodeSize; n++)
			{
				CLink* pLink = g_LinkIDMap[pTrain->m_aryTN[n].LinkID];

				if(g_CellBasedNetworkFlag == 1)   // cell based representation, only apply to node departure time
				{
					// inside loop for each link traveled by each tran
					for(int t = pTrain->m_aryTN[n-1].NodeArrivalTimestamp-g_SafetyHeadway; t<= pTrain->m_aryTN[n-1].NodeArrivalTimestamp + g_SafetyHeadway; t++)
					{
						if(t>=0 && t<OptimizationHorizon)
						{
							pLink->m_ResourceAry[t].UsageCount+=1;
							pLink->m_ResourceAry[t].LastUseIterationNo = 0;

							if(pLink->m_ResourceAry[t].UsageCount >= pLink->m_LinkCapacity)  //over capacity
								{
									pLink->m_ResourceAry[t].Price  = MAX_SPLABEL;  // set the maximum price so the followers cannot use this time
								}

						}

					}
				}

				if(g_CellBasedNetworkFlag == 0)  // general capacity constraint
				{
					// inside loop for each link traveled by each tran
					for(int t = pTrain->m_aryTN[n-1].NodeArrivalTimestamp; t<= pTrain->m_aryTN[n].NodeArrivalTimestamp; t++)
					{
						if(t>=0 && t<OptimizationHorizon)
						{
							pLink->m_ResourceAry[t].UsageCount+=1;
							pLink->m_ResourceAry[t].LastUseIterationNo = 0;

							if(pLink->m_ResourceAry[t].UsageCount >= pLink->m_LinkCapacity)  //over capacity
								{
									pLink->m_ResourceAry[t].Price  = MAX_SPLABEL;  // set the maximum price so the followers cannot use this time
								}

						}

					}
				}

			}
		



		}

	

		//final export to log file at last iteration

			for (iLink = g_LinkSet.begin(); iLink != g_LinkSet.end(); iLink++)
			{
				int t;
				for(t=0; t< OptimizationHorizon; t++)
				{
					if((*iLink)->m_ResourceAry[t].UsageCount > (*iLink)->m_LinkCapacity)
					{
						g_LogFile << "Error still exist in priority rule-based method: Capacity is exceeded: arc" << (*iLink)->m_FromNodeNumber << "->" <<  (*iLink)->m_ToNodeNumber << ",Time: " << t << endl;
					}
				}
			}

				g_LogFile << "Total Trip Price (Upper bound) = " << TotalTripPrice << endl;



	if(m_pNetwork !=NULL)     // m_pNetwork is used to calculate time-dependent generalized least cost path 
		delete m_pNetwork;

	return true;

}





