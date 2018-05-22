#include "delay.h"
#include "myprintf.h"
#include "bsp.h"

int main()
{
	uart1_init(115200);
	delay_init();
	while (1)
	{
		myprint("this is a test program!\r\n");
		delay_ms(800);
	}
}