
// Becasese the functions below might relate to file interfaces with other proprietary software packages, no copyright or GPL statement is made here.

// Utility.cpp : Utility functions used for reading and outputing

#include "stdafx.h"
#include "math.h"
#include "Network.h"
#include "FastTrain.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;

unsigned long GetLinkKey(int FromNodeID, int ToNodeID)
	{
		int MaxNodeKey = 60000;  // max: unsigned short 65,535;
		unsigned long LinkKey = FromNodeID*MaxNodeKey+ToNodeID;
		return LinkKey;
}

CLink* g_FindLinkWithNodeNumbers(int FromNodeNumber, int ToNodeNumber)
{
		int FromNodeID = g_NodeNumbertoIDMap[FromNodeNumber];
		int ToNodeID = g_NodeNumbertoIDMap[ToNodeNumber];

		unsigned long LinkKey = GetLinkKey( FromNodeID, ToNodeID);
		return g_NodeIDtoLinkMap[LinkKey];
}

CLink* g_FindLinkWithNodeIDs(int FromNodeID, int ToNodeID)
{

		unsigned long LinkKey = GetLinkKey( FromNodeID, ToNodeID);
		return g_NodeIDtoLinkMap[LinkKey];
}

float g_GetRandomRatio()
{
	//		g_RandomSeed = (g_LCG_a * g_RandomSeed + g_LCG_c) % g_LCG_M;  //m_RandomSeed is automatically updated.

	//		return float(g_RandomSeed)/g_LCG_M;

	return float(rand())/RAND_MAX;
}
// polar form of the Box-Muller transformation to get two random numbers that follow a standard normal distribution 
float g_RNNOF()
{
    float x1, x2, w, y1, y2;
 
         do {
                 x1 = 2.0f * g_GetRandomRatio() - 1.0f;
                 x2 = 2.0f * g_GetRandomRatio()- 1.0f;
                 w = x1 * x1 + x2 * x2;
         } while ( w >= 1.0f );

         w = sqrt( (-2.0f * log( w ) ) / w );
         y1 = x1 * w;
         y2 = x2 * w;

	 return y1;  // we only use one random number
}


int g_read_integer_with_char_O(FILE* f)
   // read an integer from the current pointer of the file, skip all spaces, if read "O", return 0;
{
   char ch, buf[ 32 ];
   int i = 0;
   int flag = 1;
   /* returns -1 if end of file is reached */

   while(true)
      {
      ch = getc( f );
      if( ch == EOF ) return -1;
      if( ch == 'O' ) return 0;  // special handling

      if (isdigit(ch))
         break;
      if (ch == '-')
         flag = -1;
      else
         flag = 1;
      };
   if( ch == EOF ) return -1;
   while( isdigit( ch )) {
      buf[ i++ ] = ch;
      ch = fgetc( f );
      }
   buf[ i ] = 0;


   return atoi( buf ) * flag;

}

void g_ProgramStop()
{
	getchar();
	exit(0);
};

int g_read_integer(FILE* f)
   // read an integer from the current pointer of the file, skip all spaces
{
   char ch, buf[ 32 ];
   int i = 0;
   int flag = 1;
   /* returns -1 if end of file is reached */

   while(true)
      {
      ch = getc( f );
      if( ch == EOF ) return -1;
      if (isdigit(ch))
         break;
      if (ch == '-')
         flag = -1;
      else
         flag = 1;
      };
   if( ch == EOF ) return -1;
   while( isdigit( ch )) {
      buf[ i++ ] = ch;
      ch = fgetc( f );
      }
   buf[ i ] = 0;


   return atoi( buf ) * flag;

}

float g_read_float(FILE *f)
   //read a floating point number from the current pointer of the file,
   //skip all spaces

{
   char ch, buf[ 32 ];
   int i = 0;
   int flag = 1;

   /* returns -1 if end of file is reached */

   while(true)
      {
      ch = getc( f );
      if( ch == EOF ) return -1;
      if (isdigit(ch))
         break;

      if (ch == '-')
         flag = -1;
      else
         flag = 1;

      };
   if( ch == EOF ) return -1;
   while( isdigit( ch ) || ch == '.' ) {
      buf[ i++ ] = ch;
      ch = fgetc( f );

      }
   buf[ i ] = 0;

   /* atof function converts a character string (char *) into a doubleing
      pointer equivalent, and if the string is not a floting point number,
      a zero will be return.
      */

   return (float)(atof( buf ) * flag);

}


int g_GetPrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename) 
{
   char lpbuffer[64];
   int value = def_value;
   if(GetPrivateProfileString(section,key,"",lpbuffer,sizeof(lpbuffer),filename)) 
   {
	   value =  atoi(lpbuffer); 
   }

   if(value == def_value)  //  the parameter might not exist
   {
   sprintf_s(lpbuffer,"%d",def_value);
   WritePrivateProfileString(section,key,lpbuffer,filename);
   }
	   return value; 
}

float g_GetPrivateProfileFloat( LPCTSTR section, LPCTSTR key, float def_value, LPCTSTR filename) 
{ 
   char lpbuffer[64];
   float value = def_value;
   if(GetPrivateProfileString(section,key,"",lpbuffer,sizeof(lpbuffer),filename)) 
   {
	   value =  (float)(atof(lpbuffer)); 
   }

   if(value == def_value)  //  the parameter might not exist
   {
   sprintf_s(lpbuffer,"%5.2f",def_value);
   WritePrivateProfileString(section,key,lpbuffer,filename);
   }

	   return value; 
} 
