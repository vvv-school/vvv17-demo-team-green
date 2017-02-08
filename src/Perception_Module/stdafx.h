// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

//#include "targetver.h"
//#include <windows.h>
#include <stdio.h>
//#include <tchar.h>

// TODO: reference additional headers your program requires here
#ifndef __fastcall
#define __fastcall
#endif

void Sleep(int ms);

#ifndef byte
typedef unsigned char byte;
#endif

// TODO: reference additional headers your program requires here
void MessageBox(void *hdl,char *Msg,char* Caption,int Button);

#ifndef MB_OK
#define MB_OK 1
#endif
