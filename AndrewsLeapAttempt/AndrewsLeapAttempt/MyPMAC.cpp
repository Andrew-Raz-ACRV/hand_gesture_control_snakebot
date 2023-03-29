// MyPMAC.cpp: implementation of the CMyPMAC class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
//#include "Controller.h"
//#include "6DoFCTR.h"
#include "MyPMAC.h"
#include "myRuntime.h"
//#include "ControllerDlg.h"
//#include "6DoFCTRDlg.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CMyPMAC::CMyPMAC()
{
	const DWORD dwDevice = 0;
}

CMyPMAC::~CMyPMAC()
{

}

void CMyPMAC::PmacInit()
{
//	CMy6DoFCTRDlg *pMy6DoFCTRDlg=(CMy6DoFCTRDlg *)AfxGetMainWnd();
	dwDevice = 0;
	if((OpenRuntimeLink())==NULL){
//		MessageBeep(MB_ICONEXCLAMATION);
//		pMy6DoFCTRDlg->MessageBox("打开PMAC RuntimeLink失败!","警告",MB_OK|MB_ICONQUESTION);
		printf("打开PMAC RuntimeLink失败!\n\n");
		return;}
	if(!OpenPmacDevice(dwDevice)){
//		MessageBeep(MB_ICONEXCLAMATION);
//		pMy6DoFCTRDlg->MessageBox("打开PMAC失败!","警告",MB_OK|MB_ICONQUESTION);
		printf("打开PMAC失败!\n\n");
		return;}

}

void CMyPMAC::PmacProcess(LPTSTR wParam, LPCTSTR lParam)
{
	PmacGetResponse(dwDevice,wParam,256,lParam);
}
