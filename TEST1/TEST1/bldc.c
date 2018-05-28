#include "bldc.h"
#include "bsp.h"

#define  STEP0() do{phase_enable(0,0,0);}while(0)
#define  STEP1(DUTY) do{phase_enable(1,1,0);PWM(DUTY,0,0);}while(0)
#define  STEP2(DUTY) do{phase_enable(1,0,1);PWM(DUTY,0,0);}while(0)
#define  STEP3(DUTY) do{phase_enable(0,1,1);PWM(0,DUTY,0);}while(0)
#define  STEP4(DUTY) do{phase_enable(1,1,0);PWM(0,DUTY,0);}while(0)
#define  STEP5(DUTY) do{phase_enable(1,0,1);PWM(0,0,DUTY);}while(0)
#define  STEP6(DUTY) do{phase_enable(0,1,1);PWM(0,0,DUTY);}while(0)


void bldc_default_para(motor_t *self)
{
	self->poles = 8;
	self->hall_tbl[1] = 2;
	self->hall_tbl[2] = 4;
	self->hall_tbl[3] = 3;
	self->hall_tbl[4] = 6;
	self->hall_tbl[5] = 1;
	self->hall_tbl[6] = 5;
	self->hall_tbl_rt[1] = 5;
	self->hall_tbl_rt[2] = 1;
	self->hall_tbl_rt[3] = 6;
	self->hall_tbl_rt[4] = 3;
	self->hall_tbl_rt[5] = 4;
	self->hall_tbl_rt[6] = 2;


	self->parameter.kp = 80 * 1000;
	self->parameter.ki = 3.5 * 1000;
	self->parameter.watchdog_time = 500;
	self->parameter.overcurrent_time = 1000;
	self->parameter.overheat_time = 1000;
	self->parameter.overtemp = 1000;  //1000 for 50'C
	self->parameter.stall_time = 1000;
//	     self->parameter.kp = 80 * 1000;
//	     self->parameter.ki = 3.5 * 1000;
//	     self->parameter = para->bldc;
	self->pid.ki = self->parameter.ki / 1000.0;
	self->pid.kp = self->parameter.kp / 1000.0;
}

void pid_init(pid_contorller_t * self)
{
	self->err = 0;
	self->d_err = 0;
	self->ui = 0;
	self->ud = 0;
	self->kd = 0;
	self->kp = 30;
	self->output = 0;
	self->imax = MAX_PWM_DUTY;
	self->imin = -1 * MAX_PWM_DUTY;
}


void pid_reset(pid_contorller_t *self)
{
	self->err = 0;
	self->ui = 0;
}




void bldc_init(motor_t *self)
{
	pid_init(&self->pid);
	self->poles = 8;
	self->watchdog = 0;
	self->fbk_pos = 0;
	self->state.data = 0;
	bldc_default_para(self);
	bldc_disable(self);
//	self->parameter.overtemp = GET_TEM_BLDC() - 350;
}



void bldc_switch(motor_t *self)
{
	if (self->__enabled == 0)
		return;
	int16_t duty = self->duty;
	uint16_t next_step = 0;
	uint16_t phase = self->now_hall;
	if (duty >= 0)
		next_step = (self->hall_tbl[phase] % 6) + 1;
	else
	{
		duty = -duty;
		next_step = (self->hall_tbl_rt[phase] % 6) + 1;
	}
	
	if (duty > MAX_PWM_DUTY)
		duty = MAX_PWM_DUTY;
	
	switch (next_step)
	{
	case 1:
		/*  AH -- BL  1-2*/
		STEP1(duty); break; 
	case 2:
		/*  AH -- CL  1-3 */
		STEP2(duty); break;
	case 3:
		/*  BH -- CL  2-3 */
		STEP3(duty); break;
	case 4:
		/*  BH -- AL  2-1 */
		STEP4(duty); break;
	case 5:
		/*  CH -- AL  3-1 */
		STEP5(duty); break;
	case 6:
		/*  CH -- BL  3-2*/
		STEP6(duty); break;
	default:
		//		print("Hall error! %d\r\n", next_step);
				break;
	}
}


void bldc_disable(motor_t *self)
{
	STEP0();
	self->duty = 0;
	self->__enabled = 0;
	self->demand_spd = 0;
	pid_reset(&self->pid);
	self->state.bit.target_reached = 0;
}


void bldc_hall_trigger(motor_t *self,u8 hall,u32 delta_usx10)
{
	if (delta_usx10>20000)
	{
		self->__fbk_spd = 0;
	}
	self->now_hall = hall;
	u32 time = 0;
	self->time_stamp[self->time_pt++ % SPD_CAL_FILTER_LEN] = delta_usx10;
	if ((hall<1)||(hall>6))
	{
		self->hall_err_cnt++;
		if (self->hall_err_cnt>3)
		{
			self->state.bit.hall = 1;
			bldc_disable(self);
			self->hall_err_cnt = 0;
		}
	}
	else
	{
		self->hall_err_cnt = 0;
		int spd_dir = 0;
		int pose = self->hall_tbl[hall] - 1;
		if (pose==(self->last_hall+1)%6)
		{
			self->fbk_pos++;
			spd_dir = 1;
		}
		if (self->last_hall==(pose+1)%6)
		{
			self->fbk_pos--;
			spd_dir = -1;
		}
		self->last_hall = pose;
		for (int i = 0; i < SPD_CAL_FILTER_LEN; i++)
		{
			time += self->time_stamp[i];
		}
		u16	spd = 1000000.0 * 6.0 / (time*self->poles * 6) * 100;
		if (spd_dir<0)
		{
			spd = -spd;
		}
		self->duty = 2000;
		bldc_switch(self);
	}
}


void bldc_enable(motor_t *self)
{
	if (self->state.data)
		return;
	self->state.bit.target_reached = 0;
	//pid_reset(&self->pid);
	self->__tar_spd = 0;
	self->demand_spd = 0;
	self->overcurrent_tick = 0;
	self->stall_tick = 0;
	self->overheat_tick = 0;
	pid_reset(&self->pid);

	self->duty = 0;
	self->__enabled = 1;
	bldc_switch(self);
}