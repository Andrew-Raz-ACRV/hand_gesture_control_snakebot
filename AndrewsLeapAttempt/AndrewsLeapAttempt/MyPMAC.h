// MyPMAC.h: interface for the CMyPMAC class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MYPMAC_H__70835225_9532_48BA_8416_F39710B6EE75__INCLUDED_)
#define AFX_MYPMAC_H__70835225_9532_48BA_8416_F39710B6EE75__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class CMyPMAC  
{
public:
	void PmacProcess(LPTSTR wParam, LPCTSTR lParam);
	void PmacInit();
	CMyPMAC();
	virtual ~CMyPMAC();

private:
	DWORD dwDevice;
};

#endif // !defined(AFX_MYPMAC_H__70835225_9532_48BA_8416_F39710B6EE75__INCLUDED_)
