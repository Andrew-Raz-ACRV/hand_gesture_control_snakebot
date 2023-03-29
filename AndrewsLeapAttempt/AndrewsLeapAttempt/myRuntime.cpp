/***************************************************************************/
/*  File: runtime.cpp - Dynamic link functions.                            */
/*                                                                         */
/***************************************************************************/

//#ifdef _MSC
  #include "stdafx.h"

#include "myRuntime.h"

//************************************************************************
// Function instants for runtime linking
//************************************************************************

OPENPMACDEVICE                  OpenPmacDevice;
CLOSEPMACDEVICE                 ClosePmacDevice;
PMACDOWNLOADFILE                PmacDownloadFile;
PMACDOWNLOADA                   PmacDownload;
PMACSENDLINEA                   PmacSendLine;
PMACGETLINEA                    PmacGetLine;
PMACSENDCHARA                   PmacSendChar;
PMACGETRESPONSEA                PmacGetResponse;
DOWNLOAD_SHOWPROGRESS           PmacDownloadProgress;
PMACSELECT                      PmacSelect;
DRVNUMBEROFDEVICES              DrvNumberOfDevices;
PMACGETPMACTYPE                 PmacGetPmacType;
GETROMDATEA                     PmacGetRomDate;
GETROMVERSIONA                  PmacGetRomVersion;
PMACGETCONTROLRESPONSEA         PmacGetControlResponse;
PMACGETVARIABLE                 PmacGetVariable;

// DPR Functions
PMACDPRSETMEM                   PmacDPRSetMem;
PMACDPRGETMEM                   PmacDPRGetMem;
PMACDPRAVAILABLE		            PmacDPRAvailable;
PMACDPRGETFLOAT			            PmacDPRGetFloat;
PMACDPRSETFLOAT			            PmacDPRSetFloat;
PMACDPRFLOAT			              PmacDPRFloat;
PMACDPRLFIXED			              PmacDPRLFixed;
PMACDPRGETGLOBALSTATUS	        PmacDPRGetGlobalStatus;
PMACDPRGETGLOBALSTATUSTURBO	    PmacDPRGetGlobalStatusTurbo;
PMACDPRREALTIMEEX               PmacDPRRealTimeEx;
PMACDPRBACKGROUNDEX             PmacDPRBackgroundEx;
PMACDPRPOSITION                 PmacDPRPosition;
// Interrupt Functions
PMACINTRTERMINAGE      PmacINTRTerminate;
PMACINTRFUNCINIT       PmacINTRFuncCallInit;


static HINSTANCE hLib;
HINSTANCE OpenRuntimeLink()
{
  int i;

  // Get handle to PComm32.DLL
  hLib = LoadLibrary("PComm32");
  if ( hLib != NULL ) 
  {
    for (i = 0 ; i < 1 ; i++)
    {
      OpenPmacDevice = (OPENPMACDEVICE)GetProcAddress(
                           hLib,"OpenPmacDevice");
      if (OpenPmacDevice == NULL) break;
      ClosePmacDevice = (CLOSEPMACDEVICE)GetProcAddress(
                           hLib,"ClosePmacDevice");
      if (ClosePmacDevice == NULL) break;
      PmacDownloadFile = (PMACDOWNLOADFILE)GetProcAddress(
                           hLib,"PmacDownloadFile");
      if (PmacDownloadFile == NULL) break;
      PmacDownload = (PMACDOWNLOADA)GetProcAddress(
                           hLib,"PmacDownloadA");
      if (PmacDownload == NULL) break;
      PmacSendLine = (PMACSENDLINEA)GetProcAddress(
                           hLib,"PmacSendLineA");
      if (PmacSendLine == NULL) break;
      PmacGetLine = (PMACGETLINEA)GetProcAddress(
                           hLib,"PmacGetLineA");
      if (PmacGetLine == NULL) break;
      PmacGetResponse = (PMACGETRESPONSEA)GetProcAddress(
                           hLib,"PmacGetResponseA");
      if (PmacGetResponse == NULL) break;
      PmacGetControlResponse = (PMACGETCONTROLRESPONSEA)GetProcAddress(hLib,"PmacGetControlResponseA");
    	if(PmacGetControlResponse == NULL) break;  
      PmacDownloadProgress = (DOWNLOAD_SHOWPROGRESS)GetProcAddress(
                           hLib,"PmacDownloadProgress");
      if (PmacDownloadProgress == NULL) break;
      PmacSelect = (PMACSELECT)GetProcAddress(hLib,"PmacSelect");
      if (PmacSelect == NULL) break;
      DrvNumberOfDevices = (DRVNUMBEROFDEVICES)GetProcAddress(
                           hLib,"DrvNumberOfDevices");
      if (DrvNumberOfDevices == NULL) break;
      PmacGetPmacType = (PMACGETPMACTYPE)GetProcAddress(
                           hLib,"PmacGetPmacType");
      if (PmacGetPmacType == NULL) break;
      PmacGetRomDate = (GETROMDATEA)GetProcAddress(
                           hLib,"PmacGetRomDateA");
      if (PmacGetRomDate == NULL) break;
      PmacGetRomVersion = (GETROMVERSIONA)GetProcAddress(
                           hLib,"PmacGetRomVersionA");
      if (PmacGetRomVersion == NULL) break;

      //DPR Stuff
      PmacDPRSetMem = (PMACDPRSETMEM)GetProcAddress(
                           hLib,"PmacDPRSetMem");
      if (PmacDPRSetMem == NULL) break;
      PmacDPRGetMem = (PMACDPRGETMEM)GetProcAddress(
                           hLib,"PmacDPRGetMem");
      if (PmacDPRGetMem == NULL) break;
	    PmacDPRAvailable = (PMACDPRAVAILABLE)GetProcAddress(
						   hLib, "PmacDPRAvailable");
	    if (PmacDPRAvailable == NULL) break;
	    PmacDPRGetFloat = (PMACDPRGETFLOAT)GetProcAddress(
						   hLib,"PmacDPRGetFloat");
	    if (PmacDPRGetFloat == NULL) break;
	    PmacDPRSetFloat = (PMACDPRSETFLOAT)GetProcAddress(
						   hLib,"PmacDPRSetFloat");
	    if (PmacDPRSetFloat == NULL) break;
      PmacDPRFloat = (PMACDPRFLOAT)GetProcAddress(
						   hLib,"PmacDPRFloat");
	    if (PmacDPRFloat == NULL) break;
	    PmacDPRLFixed = (PMACDPRLFIXED)GetProcAddress(
						   hLib,"PmacDPRLFixed");
	    if (PmacDPRLFixed == NULL) break;
	    PmacDPRGetGlobalStatus = (PMACDPRGETGLOBALSTATUS)GetProcAddress(
						   hLib,"PmacDPRGetGlobalStatus");
	    if (PmacDPRGetGlobalStatus == NULL) break;
	    PmacDPRGetGlobalStatusTurbo = (PMACDPRGETGLOBALSTATUSTURBO)GetProcAddress(
						   hLib,"PmacDPRGetGlobalStatusTurbo");
	    if (PmacDPRGetGlobalStatusTurbo == NULL) break;
      PmacDPRRealTimeEx = (PMACDPRREALTIMEEX)GetProcAddress(
						   hLib,"PmacDPRRealTimeEx");
	    if (PmacDPRRealTimeEx == NULL) break;
      PmacDPRBackgroundEx = (PMACDPRBACKGROUNDEX)GetProcAddress(
						   hLib,"PmacDPRBackgroundEx");
	    if (PmacDPRBackgroundEx == NULL) break;
      PmacDPRPosition = (PMACDPRPOSITION)GetProcAddress(
               hLib,"PmacDPRPosition");
      if (PmacDPRPosition == NULL) break;


	  }
    if(i==0)                         // Check validity of procedure addresses
    {
      FreeLibrary(hLib);
      hLib = NULL;
    }
  }

  return hLib;
}
//---------------------------------------------------------------------------

void CloseRuntimeLink()
{
  if(hLib)
    FreeLibrary(hLib);
}