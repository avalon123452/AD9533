/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include "ad5933.h"
#include <math.h>

/******************************************************************************/
/************************** Constants Definitions *****************************/
/******************************************************************************/
const int32_t pow_2_27 = 134217728ul;      // 2 to the power of 27

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
void ad5933_init(ad5933_t *dev,I2C_HandleTypeDef *i2c, uint32_t sys_clk, uint8_t clk_source, uint8_t gain, uint8_t range) {
	dev->hi2c = i2c;
	dev->current_sys_clk = sys_clk;
	dev->current_clock_source = clk_source;
	dev->current_gain = gain;
	dev->current_range = range;
	ad5933_setSysClk(dev, clk_source, sys_clk);
	ad5933_setRangeNGain(dev, range, gain);
	ad5933_reset(dev);
}

void ad5933_setReg(ad5933_t *dev, uint8_t regAddr, uint32_t val, uint8_t bytesLength){
	uint8_t writeData[2]={0};
	for (uint8_t i= 0; i < bytesLength; i++) {
		writeData[0]= regAddr + bytesLength - i -1;
		writeData[1]= (uint8_t)((val>> (i *8))& 0xFF);
		HAL_I2C_Master_Transmit(dev->hi2c, AD5933_ADDRESS << 1, writeData, 2, AD5933_TIMEOUT);
	}
}

uint32_t ad5933_getReg(ad5933_t *dev, uint8_t regAddr, uint8_t bytesLength){
	uint32_t val=0;
	uint8_t writeData[2]={0};
	uint8_t readData=0xFF;
	for (uint8_t i= 0; i < bytesLength; i++) {
		writeData[0]= AD5933_ADDR_POINTER;
		writeData[1]= regAddr + i;
		HAL_I2C_Master_Transmit(dev->hi2c, AD5933_ADDRESS << 1, writeData, 2, AD5933_TIMEOUT);
		HAL_I2C_Master_Receive(dev->hi2c, AD5933_ADDRESS << 1, &readData, 1, AD5933_TIMEOUT);
		val = val <<8;
		val |= readData;
	}
	return val;
}

void ad5933_reset(ad5933_t *dev) {
	ad5933_setReg(dev,AD5933_REG_CONTROL_LB,AD5933_CONTROL_RESET | dev->current_clock_source,1);
}

void ad5933_setSysClk(ad5933_t *dev, uint8_t clkSrc,uint32_t extClkFreq) {
	dev->current_clock_source = clkSrc;
	if (clkSrc == AD5933_CONTROL_EXT_SYSCLK) {
		dev->current_sys_clk = extClkFreq;
	}
	else {
		dev->current_sys_clk = AD5933_INTERNAL_SYS_CLK;
	}
	ad5933_setReg(dev,AD5933_REG_CONTROL_LB,dev->current_clock_source,1);
}

void ad5933_setRangeNGain(ad5933_t *dev,uint8_t range,uint8_t gain) {
	ad5933_setReg(dev,
				  AD5933_REG_CONTROL_HB,
				  AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_NOP) |
				  AD5933_CONTROL_RANGE(range) |
				  AD5933_CONTROL_PGA_GAIN(gain),
				  1);
	/* Store the last settings made to range and gain. */
	dev->current_range = range;
	dev->current_gain = gain;
}

float ad5933_getTemperature(ad5933_t *dev)
{
	float temperature = 0;
	uint8_t status = 0;

	ad5933_setReg(dev,
				  AD5933_REG_CONTROL_HB,
				  AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_MEASURE_TEMP) |
				  AD5933_CONTROL_RANGE(dev->current_range) |
				  AD5933_CONTROL_PGA_GAIN(dev->current_gain),
				  1);
	while((status & AD5933_STAT_TEMP_VALID) == 0) {
		status = ad5933_getReg(dev,
						   AD5933_REG_STATUS,
						   1);
	}

	temperature = ad5933_getReg(dev,
						AD5933_REG_TEMP_DATA,
						2);
	if(temperature < 8192) {
		temperature /= 32;
	} else {
		temperature -= 16384;
		temperature /= 32;
	}
	return temperature;
}

void ad5933_configSweep(ad5933_t *dev,
			 uint32_t  start_freq,
			 uint32_t  inc_freq,
			 uint16_t inc_num)
{
	uint32_t start_freq_reg = 0;
	uint32_t inc_freq_reg = 0;
	uint16_t inc_num_reg = 0;

	/* Ensure that incNum is a valid data. */
	if(inc_num > AD5933_MAX_INC_NUM) {
		inc_num_reg = AD5933_MAX_INC_NUM;
	} else {
		inc_num_reg = inc_num;
	}

	/* Convert users start frequency to binary code. */
	start_freq_reg = (uint32_t)((double)start_freq * 4 / dev->current_sys_clk *
				    pow_2_27);

	/* Convert users increment frequency to binary code. */
	inc_freq_reg = (uint32_t)((double)inc_freq * 4 / dev->current_sys_clk *
				  pow_2_27);

	/* Configure the device with the sweep parameters. */
	ad5933_setReg(dev,
				  AD5933_REG_FREQ_START,
				  start_freq_reg,
				  3);
	ad5933_setReg(dev,
				  AD5933_REG_FREQ_INC,
				  inc_freq_reg,
				  3);
	ad5933_setReg(dev,
				  AD5933_REG_INC_NUM,
				  inc_num_reg,
				  2);
}

void ad5933_startSweep(ad5933_t *dev)
{
	ad5933_control(dev, AD5933_FUNCTION_STANDBY);
	ad5933_control(dev, AD5933_FUNCTION_INIT_START_FREQ);
	ad5933_control(dev, AD5933_FUNCTION_START_SWEEP);
}

void ad5933_getData(ad5933_t *dev,
		     short *imag_data,
		     short *real_data)
{
	ad5933_wait(dev, AD5933_STAT_DATA_VALID);

	*real_data = ad5933_getReg(dev,
					       AD5933_REG_REAL_DATA,
					       2);
	*imag_data = ad5933_getReg(dev,
					       AD5933_REG_IMAG_DATA,
					       2);
}

void ad5933_freqSweep(ad5933_t *dev, int16_t imag[], int16_t real[],uint16_t increment)
{
	uint8_t sweepDone=0, i=0;
	ad5933_startSweep(dev);
	while(sweepDone==0)
	{
		if(i>=increment) {
			break;
		}
		ad5933_getData(dev, &imag[i], &real[i]);
		i++;
		sweepDone = ad5933_getReg(dev,AD5933_REG_STATUS,1) & AD5933_STAT_SWEEP_DONE;
		ad5933_control(dev, AD5933_FUNCTION_INC_FREQ);
	}
	ad5933_control(dev, AD5933_FUNCTION_STANDBY);
}

void ad5933_control(ad5933_t *dev,uint8_t function)
{
	ad5933_setReg(dev,
				  AD5933_REG_CONTROL_HB,
				  AD5933_CONTROL_FUNCTION(function) |
				  AD5933_CONTROL_RANGE(dev->current_range) |
				  AD5933_CONTROL_PGA_GAIN(dev->current_gain),
				  1);
}

double ad5933_calculateGainFactor(ad5933_t *dev,
				    uint32_t calibration_impedance)
{
	double gain_factor = 0;
	double magnitude = 0;
	int16_t real_data=0, imag_data=0;

	ad5933_getData(dev, &imag_data, &real_data);
	magnitude = sqrt((real_data * real_data) + (imag_data * imag_data));
	gain_factor = 1 / (magnitude * calibration_impedance);

	return gain_factor;
}

void ad5933_calculateImpedance(ad5933_t *dev,
				  double gain_factor,
				  int16_t *imag_data,
				  int16_t *real_data,
				  double *impedance)
{
	double magnitude = 0;

	ad5933_getData(dev, imag_data, real_data);

	magnitude = sqrt((*real_data * *real_data) + (*imag_data * *imag_data));
	*impedance =  1 / (magnitude * gain_factor);
}

void ad5933_setSettlingTime(ad5933_t *dev,
			      uint8_t multiplier,
			      uint16_t number_cycles)
{
	if ((multiplier != AD5933_SETTLING_X2) && (multiplier != AD5933_SETTLING_X4))
		multiplier = AD5933_SETTLING_X1;

	ad5933_setReg(dev,
				  AD5933_REG_SETTLING_CYCLES,
				  number_cycles | (multiplier << 9),
				  2);
}

void ad5933_wait(ad5933_t *dev,uint8_t mask)
{
	uint8_t status=0;
	while((status & mask) == 0) {
		status = ad5933_getReg(dev,
						   AD5933_REG_STATUS,
						   1);
	}
}

void ad5933_repeatFreq(ad5933_t *dev,
		  int16_t *imag_data,
		  int16_t *real_data)
{
	ad5933_control(dev, AD5933_FUNCTION_REPEAT_FREQ);
	ad5933_getData(dev, imag_data, real_data);
}
