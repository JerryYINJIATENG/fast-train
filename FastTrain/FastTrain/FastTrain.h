//  Portions Copyright 2010 Xuesong Zhou (xzhou99@gmail.com)

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html


//    This file is part of NeXTA Version 3 (Open-source).

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
extern std::set<CNode*>		g_NodeSet;
extern std::map<int, CNode*> g_NodeIDMap;
extern std::map<int, int> g_NodeNumbertoIDMap;
extern std::map<int, int> g_NodeIDToNumberMap;

extern	std::map<unsigned long, CLink*> g_NodeIDtoLinkMap;


extern std::set<CLink*>		g_LinkSet;
extern std::map<int, CLink*> g_LinkIDMap;
extern std::vector<CTrain*> g_TrainVector;
extern std::vector<CSimpleTrainData> g_SimpleTrainDataVector;  // for STL sorting


extern int g_OptimizationHorizon;
extern int g_MaxNumberOfLRIterations;
extern int g_MaxNumberOfFeasibilityPenaltyIterations;
extern int g_SafetyHeadway;
extern int g_NumberOfTrainTypes;
extern int g_CellBasedNetworkFlag;



extern ofstream g_LogFile;

int g_read_integer(FILE* f);
int g_read_integer_with_char_O(FILE* f);

float g_read_float(FILE *f);

unsigned long GetLinkKey(int FromNodeID, int ToNodeID);
CLink* g_FindLinkWithNodeNumbers(int FromNodeNumber, int ToNodeNumber);
CLink* g_FindLinkWithNodeIDs(int FromNodeID, int ToNodeID);
int g_GetPrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename);
float g_GetPrivateProfileFloat( LPCTSTR section, LPCTSTR key, float def_value, LPCTSTR filename);
void g_ReadTrainProfileCSVFile();
bool g_ReadTimetableCVSFile();
bool g_TimetableOptimization_Lagrangian_Method_with_PriorityRules();
bool g_TimetableOptimization_Lagrangian_Method();
bool g_ExportTimetableDataToCSVFile();
bool g_TimetableOptimization_Priority_Rule();
void g_ExpandNetworkForDoubleTracks();
