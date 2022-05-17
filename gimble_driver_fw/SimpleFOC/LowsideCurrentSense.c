#include "main.h"
#include "adc.h"
#include "MP6540A_ASS.h"

/******************************************************************************/
uint32_t pinA,pinB,pinC;
float gain_a,gain_b,gain_c;
float offset_ia,offset_ib,offset_ic;
/******************************************************************************/
void Current_calibrateOffsets(void);
/******************************************************************************/
void LowsideCurrentSense(float _shunt_resistor, float _gain, uint32_t _pinA, uint32_t _pinB, uint32_t _pinC)
{
//	float volts_to_amps_ratio;
	
	pinA = _pinA;
	pinB = _pinB;
	pinC = _pinC;
	
//	volts_to_amps_ratio = 1.0 /_shunt_resistor / _gain; // volts to amps
	
	gain_a = 1/1.1461;
	gain_b =-1/1.0363;
	gain_c = 1/1.1494;
	
//	printf("gain_a:%.2f,gain_b:%.2f,gain_c:%.2f.\r\n",gain_a,gain_b,gain_c);
}
/******************************************************************************/
void LowsideCurrentSense_Init(void)
{
//	configureADCInline(pinA,pinB,pinC);
	Current_calibrateOffsets();   //检测偏置电压，也就是电流0A时的运放输出电压值，理论值=1.65V
}
/******************************************************************************/
// Function finding zero offsets of the ADC
void Current_calibrateOffsets(void)
{
	int i;
	
	offset_ia=0;
	offset_ib=0;
	offset_ic=0;
	// read the adc voltage 1000 times ( arbitrary number )
	for(i=0; i<500; i++)
	{
		offset_ia += _readADCVoltage(pinA);
		offset_ib += _readADCVoltage(pinB);
#if(_isset(pinC)) 
		offset_ic += _readADCVoltage(pinC);
#endif
		HAL_Delay(1);
	}
	// calculate the mean offsets
	offset_ia = offset_ia/500;
	offset_ib = offset_ib/500;
#if(_isset(pinC)) 
	offset_ic = offset_ic/500;
#endif	
//	printf("offset_ia:%.4f,offset_ib:%.4f,offset_ic:%.4f.\r\n",offset_ia,offset_ib,offset_ic);
}
/******************************************************************************/
// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s getPhaseCurrents(void)
{
	PhaseCurrent_s current;
	
	current.a = (_readADCVoltage(pinA) - offset_ia)*SOA_Ratio[temperature]; // amps
	current.b = (_readADCVoltage(pinB) - offset_ib)*-SOB_Ratio[temperature];// amps
#if(_isset(pinC)) 
	current.c = (_readADCVoltage(pinC) - offset_ic)*SOC_Ratio[temperature]; // amps
#else
	current.c = 0.0;
#endif	
	return current;
}
/******************************************************************************/


