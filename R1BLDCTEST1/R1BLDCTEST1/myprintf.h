#ifndef		__MYPRINT_H_
#define		__MYPRINT_H_

#include <stm32f0xx_usart.h>



void    print(char* fmt, ...);
void    printch(char ch);
void    printdec(int dec);
void    printflt(double flt);
void    printbin(int bin);
void    printhex(int hex);
void    printstr(char* str);

#endif