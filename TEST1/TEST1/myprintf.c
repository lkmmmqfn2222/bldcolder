
#include "myprintf.h"

void myprint(char* fmt, ...)
{
	double vargflt = 0;
	int  vargint = 0;
	char* vargpch = NULL;
	char vargch = 0;
	char* pfmt = NULL;
	va_list vp;

	va_start(vp, fmt);
	pfmt = fmt;

	while (*pfmt)
	{
		if (*pfmt == '%')
		{
			switch (*(++pfmt))
			{

			case 'c':
				vargch = va_arg(vp, int);
				/*    va_arg(ap, type), if type is narrow type (char, short, float) an error is given in strict ANSI
				mode, or a warning otherwise.In non-strict ANSI mode, 'type' is allowed to be any expression. */
				printch(vargch);
				break;
			case 'd':
			case 'i':
				vargint = va_arg(vp, int);
				printdec(vargint);
				break;
			case 'f':
				vargflt = va_arg(vp, double);
				/*    va_arg(ap, type), if type is narrow type (char, short, float) an error is given in strict ANSI
				mode, or a warning otherwise.In non-strict ANSI mode, 'type' is allowed to be any expression. */
				printflt(vargflt);
				break;
			case 's':
				vargpch = va_arg(vp, char*);
				printstr(vargpch);
				break;
			case 'b':
			case 'B':
				vargint = va_arg(vp, int);
				printbin(vargint);
				break;
			case 'x':
			case 'X':
				vargint = va_arg(vp, int);
				printhex(vargint);
				break;
			case '%':
				printch('%');
				break;
			default:
				break;
			}
			pfmt++;
		}
		else
		{
			printch(*pfmt++);
		}
	}
	va_end(vp);
}

void printch(char ch)
{
	//	return;
		USART_SendData(USART1, ch);
	while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TC))
		;
}

void printdec(int dec)
{
	if (dec == 0)
	{
		printch('0');
		return;
	}
	if (dec < 0)
	{
		dec = -dec;
		printch('-');
	}

	int num = 0;
	int tmp = dec;
	while (tmp >= 10)
	{
		num += 1;
		tmp /= 10;
	}

	for (int i = num; i >= 0; i--)
	{
		int tmp = dec;
		for (int j = 0; j < i; j++)
			tmp /= 10;
		tmp = tmp % 10;
		printch((char)(tmp + '0'));
	}
	
}

void printflt(double flt)
{
	int icnt = 0;
	int tmpint = 0;

	tmpint = (int)flt;
	printdec(tmpint);
	printch('.');
	flt = flt - tmpint;
	tmpint = (int)(flt * 1000000);
	printdec(tmpint);
}

void printstr(char* str)
{
	while (*str)
	{
		printch(*str++);
	}
}

void printbin(int bin)
{
	if (bin == 0)
	{
		printstr("0b");
		return;
	}
	printbin(bin / 2);
	printch((char)(bin % 2 + '0'));
}

void printhex(int hex)
{
	if (hex == 0)
	{
		printstr("0x");
		return;
	}
	printhex(hex / 16);
	if (hex < 10)
	{
		printch((char)(hex % 16 + '0'));
	}
	else
	{
		printch((char)(hex % 16 - 10 + 'a'));
	}
}