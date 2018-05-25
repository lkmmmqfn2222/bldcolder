#ifndef _BLDC_H
#define _BLDC_H

#include "include_config.h"

#define  SPD_CAL_FILTER_LEN		6


/************************************
 *PID peremeter 
 *
 *
 **/
typedef struct
{
	float err;
	float d_err;
	float last_err;
	float ui;
	float ud;
	float imax;
	float imin;
	float kp;
	float ki;
	float kd;
	float output;

}pid_contorller_t;


typedef struct {
	uint32_t kp;
	uint32_t ki;
	uint16_t watchdog_time;
	uint16_t overtemp;
	uint16_t overheat_time;
	uint16_t overcurrent_time;
	uint16_t stall_time;
} bldc_parameter_t;


/****************************
 *
 *motor peremeter
 *
 *
 **/
typedef struct	
{
	uint16_t hall_tbl[8];
	uint16_t hall_tbl_rt[8];
	uint16_t poles;
	uint16_t __enabled;    // = 1 for control, 0 for disable
	int16_t duty;

	
	volatile int16_t  __tar_spd; /* with 100x unit*/
	volatile int16_t  __fbk_spd; /* with 100x unit*/
	float	 demand_spd;
	int32_t  fbk_pos;

	int16_t hall_err_cnt;
	int16_t last_hall;

	pid_contorller_t pid;

	uint16_t watchdog;
	uint16_t stall_tick;
	uint16_t overcurrent_tick;
	uint32_t overheat_tick;

	uint32_t time_stamp[SPD_CAL_FILTER_LEN];
	uint16_t time_pt;
	uint16_t time_pt_last;
	uint8_t	 now_hall;
	union 
	{
		u8 data;
		struct 
		{
			char hall : 1;
			char overheat : 1;
			char overcurrent : 1;
			char stall : 1;
			char vibration : 1;
			char target_reached : 1;
		}bit;
	}state;
	bldc_parameter_t parameter;
}motor_t;

void bldc_default_para(motor_t *self);
void pid_reset(pid_contorller_t *self);
void bldc_switch(motor_t *self);
void bldc_disable(motor_t *self);
void hall_interrupt(void);
void bldc_init(motor_t *self);
void bldc_enable(motor_t *self);
void bldc_hall_trigger(motor_t *self, u8 hall, u32 delta_usx10);


#endif // !_BLDC_H
