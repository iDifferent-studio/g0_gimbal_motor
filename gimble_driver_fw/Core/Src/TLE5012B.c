#include "TLE5012B.h"
#include "spi.h"

float read_TLE5012B(void)
{
	uint16_t data[2], command=0x8021;
	float angle;
	
	HAL_GPIO_WritePin(TLE_CS_GPIO_Port, TLE_CS_Pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1, (uint8_t *)(&command), 1, 1000);
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	HAL_SPI_Receive(&hspi1, (uint8_t *)(&data), 2, 1000);
	
	HAL_GPIO_WritePin(TLE_CS_GPIO_Port, TLE_CS_Pin, GPIO_PIN_SET);
	
	angle = (data[0] & 0x7fff) / (0x7fff / 360.0);
	
	return angle;
}
