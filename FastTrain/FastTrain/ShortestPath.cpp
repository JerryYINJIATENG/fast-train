//  Portions Copyright 2010 Xuesong Zhou (xzhou99@gmail.com)

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

//shortest path calculation

// note that the current implementation is only suitable for time-dependent minimum time shortest path on FIFO network, rather than time-dependent minimum cost shortest path
// the key reference (1) Shortest Path Algorithms in Transportation Models http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.51.5192
// (2) most efficient time-dependent minimum cost shortest path algorithm for all departure times
// Time-dependent, shortest-path algorithm for real-time intelligent vehicle highway system applications&quot;, Transportation Research Record 1408 ?Ziliaskopoulos, Mahmassani - 1993

#include "stdafx.h"
#include "Network.h"
#include "FastTrain.h"




int  NetworkForSP:: GetLinkNoByNodeIndex(int usn_index, int dsn_index)
{
	int LinkNo = -1;
	for(int i=0; i < m_OutboundSizeAry[usn_index]; i++)
	{

		if(m_OutboundNodeAry[usn_index][i] == dsn_index)
		{
			LinkNo = m_OutboundLinkAry[usn_index][i];
			return LinkNo;
		}
	}

	cout << " Error in GetLinkNoByNodeIndex " ;
	//<< g_NodeIDtoNameMap[usn_index] << "-> " << g_NodeIDtoNameMap[dsn_index];

	g_ProgramStop();

	return -1;
}




bool NetworkForSP::SimplifiedTDLabelCorrecting_DoubleQueue(int origin, int departure_time, int vehicle_type)   // Pointer to previous node (node)
// time -dependent label correcting algorithm with deque implementation
{

	int i;
	int debug_flag = 0;

	if(m_OutboundSizeAry[origin]== 0)
		return false;

	for(i=0; i <m_NodeSize; i++) // Initialization for all nodes
	{
		NodePredAry[i]  = -1;
		NodeStatusAry[i] = 0;

		LabelTimeAry[i] = MAX_SPLABEL;
		LabelCostAry[i] = MAX_SPLABEL;

	}

	// Initialization for origin node
	LabelTimeAry[origin] = float(departure_time);
	LabelCostAry[origin] = 0;

	SEList_clear();
	SEList_push_front(origin);

	int FromID, LinkNo, ToID;


	float NewTime, NewCost;
	while(!SEList_empty())
	{
		FromID  = SEList_front();
		SEList_pop_front();

		if(debug_flag)
			TRACE("\nScan from node %d",FromID);

		NodeStatusAry[FromID] = 2;        //scaned

		for(i=0; i<m_OutboundSizeAry[FromID];  i++)  // for each arc (i,j) belong A(j)
		{
			LinkNo = m_OutboundLinkAry[FromID][i];
			ToID = m_OutboundNodeAry[FromID][i];

			if(ToID == origin)
				continue;


			//					  TRACE("\n   to node %d",ToID);
			// need to check here to make sure  LabelTimeAry[FromID] is feasible.


			int link_entering_time_interval = int(LabelTimeAry[FromID])/m_OptimizationTimeInveral;
			if(link_entering_time_interval >= m_OptimizationIntervalSize)  // limit the size
				link_entering_time_interval = m_OptimizationIntervalSize-1;

			if(link_entering_time_interval < 0)  // limit the size
				link_entering_time_interval = 0;

			NewTime	 = LabelTimeAry[FromID] + m_LinkTDTimeAry[LinkNo][link_entering_time_interval];  // time-dependent travel times come from simulator
			NewCost    = LabelCostAry[FromID] + m_LinkTDCostAry[LinkNo][link_entering_time_interval];       // costs come from time-dependent tolls, VMS, information provisions

			if(NewCost < LabelCostAry[ToID] ) // be careful here: we only compare cost not time
			{

				//					       TRACE("\n         UPDATE to %f, link travel time %f", NewCost, m_LinkTDCostAry[LinkNo][link_entering_time_interval]);

				if(NewTime > m_OptimizationHorizon -1)
					NewTime = float(m_OptimizationHorizon-1);

				LabelTimeAry[ToID] = NewTime;
				LabelCostAry[ToID] = NewCost;
				NodePredAry[ToID]   = FromID;

				// Dequeue implementation
				//
				if(NodeStatusAry[ToID]==2) // in the SEList_TD before
				{
					SEList_push_front(ToID);
					NodeStatusAry[ToID] = 1;
				}
				if(NodeStatusAry[ToID]==0)  // not be reached
				{
					SEList_push_back(ToID);
					NodeStatusAry[ToID] = 1;
				}

				//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
			}

		}      // end of for each link

	} // end of while
	return true;
}





void NetworkForSP::BuildSpaceTimeNetworkForTimetabling(std::set<CNode*>* p_NodeSet, std::set<CLink*>* p_LinkSet, int TrainType)
{
	std::list<CNode*>::iterator iterNode;
	std::set<CLink*>::iterator iterLink;

	m_NodeSize = p_NodeSet->size();

	int FromID, ToID;

	int i,t;

	for(i=0; i< m_NodeSize; i++)
	{
		m_OutboundSizeAry[i] = 0;
		m_InboundSizeAry[i] = 0;

	}

	// add physical links or virtucal cell only

	for(iterLink = p_LinkSet->begin(); iterLink != p_LinkSet->end(); iterLink++)
	{

		if(   (g_CellBasedNetworkFlag == 0&&(*iterLink)->m_PhysicalLinkFlag ==true)
			|| (g_CellBasedNetworkFlag == 1&&(*iterLink)->m_PhysicalLinkFlag == false ))
		{

		FromID = (*iterLink)->m_FromNodeID;
		ToID   = (*iterLink)->m_ToNodeID;

		m_FromIDAry[(*iterLink)->m_LinkID] = FromID;
		m_ToIDAry[(*iterLink)->m_LinkID]   = ToID;

		m_OutboundNodeAry[FromID][m_OutboundSizeAry[FromID]] = ToID;
		m_OutboundLinkAry[FromID][m_OutboundSizeAry[FromID]] = (*iterLink)->m_LinkID ;
		m_OutboundSizeAry[FromID] +=1;

		m_InboundLinkAry[ToID][m_InboundSizeAry[ToID]] = (*iterLink)->m_LinkID  ;
		m_InboundSizeAry[ToID] +=1;


//		TRACE("------Link %d->%d:\n",FromID,ToID);
		ASSERT(m_AdjLinkSize > m_OutboundSizeAry[FromID]);

		m_LinkTDMaxWaitingTimeAry[(*iterLink)->m_LinkID] = (*iterLink)->GetTrainMaxWaitingTime (TrainType);  // in the future, we can extend it to time-dependent running time

		for(t=0; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
		{
			m_LinkTDTimeAry[(*iterLink)->m_LinkID][t] = (*iterLink)->GetTrainRunningTime(TrainType);  // in the future, we can extend it to time-dependent running time
			m_LinkTDCostAry[(*iterLink)->m_LinkID][t]=  (*iterLink)->m_ResourceAry[t].Price;  // for all train types


		//TRACE("Time %d, Travel Time %f, Cost %f\n", t,m_LinkTDTimeAry[(*iterLink)->m_LinkID][t] ,m_LinkTDCostAry[(*iterLink)->m_LinkID][t]);

			// use travel time now, should use cost later
		}

		}
	}

	m_LinkSize = p_LinkSet->size();
}


bool NetworkForSP::OptimalTDLabelCorrecting_DoubleQueue(int origin, int departure_time, int destination, int AllowableSlackAtDeparture, int CostPerUnitTimeStopped)
// time -dependent label correcting algorithm with deque implementation
{

	int i;
	int debug_flag = 0;  // set 1 to debug the detail information
	if(debug_flag)
				TRACE("\nCompute shortest path from %d at time %d",g_NodeIDToNumberMap[origin], departure_time);

    bool bFeasiblePathFlag  = false;

	if(m_OutboundSizeAry[origin]== 0)
		return false;

	for(i=0; i <m_NodeSize; i++) // Initialization for all nodes
	{
		NodeStatusAry[i] = 0;

		for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
		{
			TD_LabelCostAry[i][t] = MAX_SPLABEL;
			TD_NodePredAry[i][t] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
			TD_TimePredAry[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
		}

	}

	//	TD_LabelCostAry[origin][departure_time] = 0;

	// Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node

	//+1 in "departure_time + 1+ MaxAllowedStopTime" is to allow feasible value for t = departure time
	for(int t=departure_time; t < departure_time + 1+ AllowableSlackAtDeparture; t+=m_OptimizationTimeInveral)
	{
		TD_LabelCostAry[origin][t]= t-departure_time;
	}

	SEList_clear();
	SEList_push_front(origin);


	while(!SEList_empty())
	{
		int FromID  = SEList_front();
		SEList_pop_front();  // remove current node FromID from the SE list


		NodeStatusAry[FromID] = 2;        //scaned

		//scan all outbound nodes of the current node
		for(i=0; i<m_OutboundSizeAry[FromID];  i++)  // for each arc (i,j) belong A(i)
		{
			int LinkNo = m_OutboundLinkAry[FromID][i];
			int ToID = m_OutboundNodeAry[FromID][i];

			if(ToID == origin)  // remove possible loop back to the origin
				continue;


			if(debug_flag)
				TRACE("\nScan from node %d to node %d",g_NodeIDToNumberMap[FromID],g_NodeIDToNumberMap[ToID]);

					int MaxAllowedStopTime = m_LinkTDMaxWaitingTimeAry[LinkNo];


			// for each time step, starting from the departure time
			for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
			{
				if(TD_LabelCostAry[FromID][t]<MAX_SPLABEL-1)  // for feasible time-space point only
				{   
					for(int time_stopped = 0; time_stopped <= MaxAllowedStopTime; time_stopped++)
					{
						int NewToNodeArrivalTime	 = (int)(t + time_stopped + m_LinkTDTimeAry[LinkNo][t]);  // time-dependent travel times for different train type
						float NewCost  =  TD_LabelCostAry[FromID][t] + m_LinkTDTimeAry[LinkNo][t] + m_LinkTDCostAry[LinkNo][t] + time_stopped*CostPerUnitTimeStopped;
						// costs come from time-dependent resource price or road toll

						if(NewToNodeArrivalTime > (m_OptimizationHorizon -1))  // prevent out of bound error
							NewToNodeArrivalTime = (m_OptimizationHorizon-1);

						if(NewCost < TD_LabelCostAry[ToID][NewToNodeArrivalTime] ) // we only compare cost at the downstream node ToID at the new arrival time t
						{

							if(ToID == destination)
							bFeasiblePathFlag = true; 


							if(debug_flag)
								TRACE("\n         UPDATE to %f, link cost %f at time %d", NewCost, m_LinkTDCostAry[LinkNo][t],NewToNodeArrivalTime);

							// update cost label and node/time predecessor

							TD_LabelCostAry[ToID][NewToNodeArrivalTime] = NewCost;
							TD_NodePredAry[ToID][NewToNodeArrivalTime] = FromID;  // pointer to previous NODE INDEX from the current label at current node and time
							TD_TimePredAry[ToID][NewToNodeArrivalTime] = t;  // pointer to previous TIME INDEX from the current label at current node and time

							// Dequeue implementation
							if(NodeStatusAry[ToID]==2) // in the SEList_TD before
							{
								SEList_push_front(ToID);
								NodeStatusAry[ToID] = 1;
							}
							if(NodeStatusAry[ToID]==0)  // not be reached
							{
								SEList_push_back(ToID);
								NodeStatusAry[ToID] = 1;
							}

						}
					}
				}
				//another condition: in the SELite now: there is no need to put this node to the SEList, since it is already there.
			}

		}      // end of for each link

	}	// end of while


	ASSERT(bFeasiblePathFlag);

	return bFeasiblePathFlag;
}


int NetworkForSP::FindOptimalSolution(int origin, int departure_time, int destination, CTrain* pTrain, float& TripPrice)  // the last pointer is used to get the node array
{

	// step 1: scan all the time label at destination node, consider time cost
	// step 2: backtrace to the origin (based on node and time predecessors)
	// step 3: reverse the backward path
	// return final optimal solution

	// step 1: scan all the time label at destination node, consider time cost
	STrainNode tmp_AryTN[MAX_NODE_SIZE_IN_A_PATH]; //backward temporal solution

	float min_cost = MAX_SPLABEL;
	int min_cost_time_index = -1;

	for(int t=departure_time; t <m_OptimizationHorizon; t+=m_OptimizationTimeInveral)
	{
		if(TD_LabelCostAry[destination][t] < min_cost)
		{
			min_cost = TD_LabelCostAry[destination][t];
			min_cost_time_index = t;
		}

	}
		
	TripPrice = min_cost;  // this is the trip price including travel time and all resource price consumed by this train

	ASSERT(min_cost_time_index>0); // if min_cost_time_index ==-1, then no feasible path if founded

	// step 2: backtrace to the origin (based on node and time predecessors)

	int	NodeSize = 0;

	//record the first node backward, destination node
	tmp_AryTN[NodeSize].NodeID = destination;
	tmp_AryTN[NodeSize].NodeArrivalTimestamp = min_cost_time_index;

	NodeSize++;

	int PredTime = TD_TimePredAry[destination][min_cost_time_index];
	int PredNode = TD_NodePredAry[destination][min_cost_time_index];

	while(PredNode != origin && PredNode!=-1 && NodeSize< MAX_NODE_SIZE_IN_A_PATH) // scan backward in the predessor array of the shortest path calculation results
	{
		ASSERT(NodeSize< MAX_NODE_SIZE_IN_A_PATH-1);

		tmp_AryTN[NodeSize].NodeID = PredNode;
		tmp_AryTN[NodeSize].NodeArrivalTimestamp = PredTime;

		NodeSize++;

		//record current values of node and time predecessors, and update PredNode and PredTime
		int PredTime_cur = PredTime;
		int PredNode_cur = PredNode;

		PredNode = TD_NodePredAry[PredNode_cur][PredTime_cur];
		PredTime = TD_TimePredAry[PredNode_cur][PredTime_cur];

	}

	tmp_AryTN[NodeSize].NodeID = origin;
	tmp_AryTN[NodeSize].NodeArrivalTimestamp = departure_time;
	NodeSize++;

	// step 3: reverse the backward solution

	if(pTrain->m_aryTN !=NULL)
		delete pTrain->m_aryTN;

	pTrain->m_aryTN = new STrainNode[NodeSize];


	int i;
	for(i = 0; i< NodeSize; i++)
	{
		pTrain->m_aryTN[i].NodeID			= tmp_AryTN[NodeSize-1-i].NodeID;
		pTrain->m_aryTN[i].NodeArrivalTimestamp	= tmp_AryTN[NodeSize-1-i].NodeArrivalTimestamp;
	}

	for(i = 0; i< NodeSize; i++)
	{
		if(i == NodeSize-1)  // destination
			pTrain->m_aryTN[i].NodeDepartureTimestamp	= pTrain->m_aryTN[i].NodeArrivalTimestamp;
		else
		{
			CLink* pLink = g_FindLinkWithNodeIDs(pTrain->m_aryTN[i].NodeID , pTrain->m_aryTN[i+1].NodeID  );
			ASSERT(pLink!=NULL);
			pTrain->m_aryTN[i].LinkID  = pLink->m_LinkID ;

			int LinkTravelTime = pLink->GetTrainRunningTime ( pTrain->m_TrainType );
			pTrain->m_aryTN[i].NodeDepartureTimestamp	= pTrain->m_aryTN[i+1].NodeArrivalTimestamp - LinkTravelTime ;
		}

	}

	pTrain->m_ActualTripTime = pTrain->m_aryTN[NodeSize-1].NodeArrivalTimestamp - pTrain->m_EarlyDepartureTime ;

	return NodeSize;
}

