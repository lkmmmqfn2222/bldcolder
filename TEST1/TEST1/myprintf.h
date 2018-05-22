#ifndef _MYPRINTF_H
#define _MYPRINTF_H

#include "bsp.h"

void    myprint(char* fmt, ...);
void    printch(char ch);
void    printdec(int dec);
void    printflt(double flt);
void    printbin(int bin);
void    printhex(int hex);
void    printstr(char* str);


#endif

