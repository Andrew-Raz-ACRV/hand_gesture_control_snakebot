#ifndef DLLEXPORT
#define DLLEXPORT extern "C" _declspec(dllimport)
#endif

DLLEXPORT void _stdcall GetData(double (*pd)[3], int cnt);

DLLEXPORT void _stdcall SetData(double (*pd)[3], int cnt);

DLLEXPORT void _stdcall GetdstData(double *dis);

DLLEXPORT void _stdcall SetdstData(double dis);

DLLEXPORT void _stdcall GetlenData(double *len1,double *len2, double *len3);

DLLEXPORT void _stdcall SetlenData(double len1, double len2, double len3);

DLLEXPORT void _stdcall SetFirstButtonStatus(int fbd);

DLLEXPORT void _stdcall GetFirstButtonStatus(int *fbd);

DLLEXPORT void _stdcall SetSecondButtonStatus (int sbd);

DLLEXPORT void _stdcall GetSecondButtonStatus (int *sbd);

DLLEXPORT void _stdcall SetMotionData(double motion1, double motion2, double motion3);

DLLEXPORT void _stdcall GetMotionData(double *motion1, double *motion2, double *motion3);

DLLEXPORT void _stdcall SetEncoderData(double encoder1, double encoder2, double encoder3, double encoder4, double encoder5, double encoder6);

DLLEXPORT void _stdcall GetEncoderData(double *encoder1, double *encoder2, double *encoder3, double *encoder4, double *encoder5, double *encoder6);

DLLEXPORT void _stdcall SetVisualServoingStatus(int vss);

DLLEXPORT void _stdcall GetVisualServoingStatus(int *vss);

DLLEXPORT void _stdcall SetDeltaAnchorPosition(double dAnchor1, double dAnchor2, double dAnchor3);

DLLEXPORT void _stdcall GetDeltaAnchorPosition(double *dAnchor1, double *dAnchor2, double *dAnchor3);

DLLEXPORT void _stdcall GetResult(double *rs, int ct);

DLLEXPORT void _stdcall SetResult(double *rs, int ct);

DLLEXPORT void _stdcall Get(double *p, int c);

DLLEXPORT void _stdcall Set(double *p, int c);