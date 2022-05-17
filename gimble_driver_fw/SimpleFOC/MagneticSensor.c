#include "main.h"
#include "spi.h"

/******************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/
//TLE5012B
#define READ_STATUS       0x8001 //8000
#define READ_ANGLE_VALUE  0x8021 //8020
#define TLE5012B_CPR      32767
/******************************************************************************/
unsigned short ReadTLE5012B_1(unsigned short Comm)
{
	uint16_t data[2], command=READ_ANGLE_VALUE;
	
	HAL_GPIO_WritePin(TLE_CS_GPIO_Port, TLE_CS_Pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1, (uint8_t *)(&command), 1, 1000);
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	HAL_SPI_Receive(&hspi1, (uint8_t *)(&data), 2, 1000);
	
	HAL_GPIO_WritePin(TLE_CS_GPIO_Port, TLE_CS_Pin, GPIO_PIN_SET);
	return data[0];
}
/******************************************************************************/
//#endif

/******************************************************************************/
void MagneticSensor_Init(void)
{
	cpr=TLE5012B_CPR;
	angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;

	full_rotation_offset = 0;
	velocity_calc_timestamp=0;
}
/******************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
	angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*cpr) ) 
		full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev = angle_data;
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return  (full_rotation_offset + ( angle_data / (float)cpr) * _2PI) ;
}
/******************************************************************************/
// Shaft velocity calculation
float getVelocity(void)
{
	unsigned long now_us;
	float Ts, angle_c, vel;

	// calculate sample time
	now_us = SysTick->VAL; //_micros();
	if(now_us<velocity_calc_timestamp)
		Ts = (float)(velocity_calc_timestamp - now_us)/9*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/9*1e-6;
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) 
		Ts = 1e-3;

	// current angle
	angle_c = getAngle();
	// velocity calculation
	vel = (angle_c - angle_prev)/Ts;

	// save variables for future pass
	angle_prev = angle_c;
	velocity_calc_timestamp = now_us;
	return vel;
}
/******************************************************************************/
