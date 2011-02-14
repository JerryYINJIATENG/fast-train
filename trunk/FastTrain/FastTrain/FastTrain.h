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
extern std::map<int, CNode*> g_NodeMap;
extern std::map<int, int> g_NodeIDtoNameMap;
extern std::map<int, int> g_NodeNametoIDMap;
extern	std::map<unsigned long, CLink*> g_NodeIDtoLinkMap;


extern std::set<CLink*>		g_LinkSet;
extern std::map<int, CLink*> g_LinkIDMap;
extern std::vector<CTrain*> g_TrainVector;



extern ofstream g_LogFile;

int g_read_integer(FILE* f);
int g_read_integer_with_char_O(FILE* f);

float g_read_float(FILE *f);

CLink* g_FindLinkWithNodeNumbers(int FromNodeNumber, int ToNodeNumber);
CLink* g_FindLinkWithNodeIDs(int FromNodeID, int ToNodeID);
int g_GetPrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename);
float g_GetPrivateProfileFloat( LPCTSTR section, LPCTSTR key, float def_value, LPCTSTR filename);
