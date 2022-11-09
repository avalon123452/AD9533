#ifndef __AD5933_H__
#define __AD5933_H__


#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
/******************************************************************************/
/************************** AD5933 Definitions ********************************/
/******************************************************************************/

/* AD5933 Registers */
#define AD5933_REG_CONTROL_HB       0x80    // HB of the Control register
#define AD5933_REG_CONTROL_LB       0x81    // LB of the Control register
#define AD5933_REG_FREQ_START       0x82    // Start frequency
#define AD5933_REG_FREQ_INC         0x85    // Frequency increment
#define AD5933_REG_INC_NUM          0x88    // Number of increments
#define AD5933_REG_SETTLING_CYCLES  0x8A    // Number of settling time cycles
#define AD5933_REG_STATUS           0x8F    // Status
#define AD5933_REG_TEMP_DATA        0x92    // Temperature data
#define AD5933_REG_REAL_DATA        0x94    // Real data
#define AD5933_REG_IMAG_DATA        0x96    // Imaginary data

/* AD5933_REG_CONTROL_HB Bits */
#define AD5933_CONTROL_FUNCTION(x)  ((x) << 4)
#define AD5933_CONTROL_RANGE(x)     ((x) << 1)
#define AD5933_CONTROL_PGA_GAIN(x)  ((x) << 0)

/* AD5933_REG_CONTROL_LB Bits */
#define AD5933_CONTROL_RESET        (0x1 << 4)
#define AD5933_CONTROL_INT_SYSCLK   (0x0 << 3)
#define AD5933_CONTROL_EXT_SYSCLK   (0x1 << 3)

/* AD5933_CONTROL_FUNCTION(x) options */
#define AD5933_FUNCTION_NOP                 0x0
#define AD5933_FUNCTION_INIT_START_FREQ     0x1
#define AD5933_FUNCTION_START_SWEEP         0x2
#define AD5933_FUNCTION_INC_FREQ            0x3
#define AD5933_FUNCTION_REPEAT_FREQ         0x4
#define AD5933_FUNCTION_MEASURE_TEMP        0x9
#define AD5933_FUNCTION_POWER_DOWN          0xA
#define AD5933_FUNCTION_STANDBY             0xB

/* AD5933_CONTROL_RANGE(x) options */
#define AD5933_RANGE_2000mVpp       0x0
#define AD5933_RANGE_200mVpp        0x1
#define AD5933_RANGE_400mVpp        0x2
#define AD5933_RANGE_1000mVpp       0x3

/* AD5933_CONTROL_PGA_GAIN(x) options */
#define AD5933_GAIN_X5              0
#define AD5933_GAIN_X1              1

/* AD5933 Default number of settling cycles */
#define AD5933_15_CYCLES			15

/* AD5933 settling cycles mulitiplier */
#define AD5933_SETTLING_X1			0
#define AD5933_SETTLING_X2			1
#define AD5933_SETTLING_X4			3

/* AD5933_REG_STATUS Bits */
#define AD5933_STAT_TEMP_VALID      (0x1 << 0)
#define AD5933_STAT_DATA_VALID      (0x1 << 1)
#define AD5933_STAT_SWEEP_DONE      (0x1 << 2)

/* AD5933 Address */
#define AD5933_ADDRESS              0x0D

/* AD5933 Block Commands */
#define AD5933_BLOCK_WRITE          0xA0
#define AD5933_BLOCK_READ           0xA1
#define AD5933_ADDR_POINTER         0xB0

/* AD5933 Specifications */
#define AD5933_INTERNAL_SYS_CLK     16000000ul      // 16MHz
#define AD5933_MAX_INC_NUM          511             // Maximum increment number


#define AD5933_TIMEOUT				HAL_MAX_DELAY
/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
 typedef struct {
 	/* I2C */
 	I2C_HandleTypeDef *hi2c;
 	/* Device Settings */
 	uint32_t current_sys_clk;
 	uint8_t current_clock_source;
 	uint8_t current_gain;
 	uint8_t current_range;
 }ad5933_t ;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
 void ad5933_init(ad5933_t *dev,I2C_HandleTypeDef *i2c, uint32_t sys_clk, uint8_t clk_source, uint8_t gain, uint8_t range);
 void ad5933_setReg(ad5933_t *dev, uint8_t regAddr, uint32_t val, uint8_t bytesLength);
 uint32_t ad5933_getReg(ad5933_t *dev, uint8_t regAddr, uint8_t bytesLength);
 void ad5933_reset(ad5933_t *dev);
 void ad5933_setSysClk(ad5933_t *dev, uint8_t clkSrc,uint32_t extClkFreq);
 void ad5933_setRangeNGain(ad5933_t *dev,uint8_t range,uint8_t gain);
 float ad5933_getTemperature(ad5933_t *dev);
 void ad5933_configSweep(ad5933_t *dev,
 			 uint32_t  start_freq,
 			 uint32_t  inc_freq,
 			 uint16_t inc_num);
 void ad5933_startSweep(ad5933_t *dev);
 void ad5933_getData(ad5933_t *dev,
 		     short *imag_data,
 		     short *real_data);
 double ad5933_calculateGainFactor(ad5933_t *dev,
 				    uint32_t calibration_impedance);

 void ad5933_calculateImpedance(ad5933_t *dev,
 				  double gain_factor,
 				  int16_t *imag_data,
 				  int16_t *real_data,
 				  double *impedance);
 void ad5933_setSettlingTime(ad5933_t *dev,
 			      uint8_t multiplier,
 			      uint16_t number_cycles);
 void ad5933_control(ad5933_t *dev,uint8_t function);
 void ad5933_wait(ad5933_t *dev,uint8_t mask);
 void ad5933_freqSweep(ad5933_t *dev, int16_t imag[], int16_t real[],uint16_t increment);
 void ad5933_repeatFreq(ad5933_t *dev,
 		  int16_t *imag_data,
 		  int16_t *real_data);
#ifdef __cplusplus
}
#endif
#endif /* __AD5933_H__ */
