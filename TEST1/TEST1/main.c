
#include "myprintf.h"
#include "bsp.h"
#include "bldc.h"

motor_t motor;

int main()
{
	uart1_init(115200);
	bldc_drv_init();
	hall_config();
	tim16_init();
	

	bldc_init(&motor);
	bldc_enable(&motor);


	
	while(1);
}