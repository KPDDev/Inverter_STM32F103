/*
 * inverter.h
 *
 *  Created on: Aug 15, 2023
 *      Author: big01
 */

#ifndef INC_INVERTER_H_
#define INC_INVERTER_H_

#define LCD							0
//================= SYSTEM
#define HARDWARE_CHECK				1		// demo check hardware
#define PURESINE_INVTR				2		// Verify PWM

#define OPEN_LOOP					1
#define CLOSED_VOLTAGE   			2
#define CLOSED_CURRENT				3
#define CLOSED_VOLTAGE_CURRENT 	4

#define CONV_MODE					PURESINE_INVTR
#define CONTROLLEVEL 				CLOSED_CURRENT


//================= PWM, INVERTER
#define TIM_CLOCK_DIVIDER           	((uint8_t)1)         	// Master Clock / 1 = 56 MHz
#define PWM_PERIOD_CYCLES          	((uint16_t)3500)  	// CenterAligned = 1750 (3500 * (1/56,000,000)) = 16 KHz
#define REP_COUNTER                  	((uint8_t)1)			// Update Event Cycle by Cycle
#define DEAD_TIME_COUNTS            	((uint8_t)60)   		// Center Aligned = 30 ( 30 * (1/56,000,000)) = 535 nS
#define CAPTURE_POINT               	((uint16_t)600)       // ADC Trigger point

#define DUTY1                        	(uint16_t)PWM_PERIOD_CYCLES/4
#define DUTY2                        	(uint16_t)PWM_PERIOD_CYCLES/8

#define PWM_PERIOD                     ((uint32_t)PWM_PERIOD_CYCLES/2)
#define PWM_PERIOD_MIN					((uint32_t)(PWM_PERIOD_CYCLES *  2)/100)
#define PWM_PERIOD_MAX					((uint32_t)(PWM_PERIOD_CYCLES * 98)/100)

#define MIN_DUTY                       ((uint32_t)(PWM_PERIOD * 10)/100)
#define MAX_DUTY                       ((uint32_t)(PWM_PERIOD * 95)/100)
#define SAT_LIMIT					 	((uint32_t)(PWM_PERIOD * 100)/100)

//================= ADCS
#define VOUT_TARGET 					((uint16_t)5000)
#define REAL_3V3						((uint16_t)3300)

// Rlow/(Rlow + Rhigh) *10000;
// VIN = (7.5/(120+7.5)) * 10000 = 588
#define VIN_RESISTOR_RATIO			((uint16_t)588)
#define VOUT_RESISTOR_RATIO			((uint16_t)588)

#define NEGATIVE                      	0
#define POSITIVE						1

#define INACTIVE						0
#define ACTIVE							1

#define	LOW								0
#define	HIGH							1

//================= COUNTER REF 8KHz 125 uS
//	ref 1 KHz
#define	_5MS							((uint8_t)5)
#define	_10MS							((uint8_t)10)
#define	_100MS							((uint8_t)100)
#define	_200MS							((uint8_t)200)
#define	_500MS							((uint16_t)500)

#define	_1SEC							((uint16_t)1000)
#define	_2SEC							((uint16_t)2000)
#define	_3SEC							((uint16_t)3000)
#define	_4SEC							((uint16_t)4000)
#define	_5SEC							((uint16_t)5000)

#define	_10SEC							((uint16_t)10000)
#define	_15SEC							((uint16_t)15000)
#define	_20SEC							((uint16_t)20000)
#define	_30SEC							((uint16_t)30000)

#define	_1MIN							((uint32_t)60000)
#define	_2MIN							((uint32_t)120000)

//  ref 8 KHz
#define	CNT_1MS							((uint8_t)8)			//  1 mS
#define	CNT_2MS							((uint8_t)16)
#define	CNT_5MS							((uint8_t)40)
#define	CNT_10MS						((uint8_t)80)
#define	CNT_15MS						((uint8_t)120)
#define	CNT_20MS						((uint8_t)160)
#define	CNT_30MS						((uint16_t)240)
#define	CNT_50MS						((uint16_t)400)
#define	CNT_100MS						((uint16_t)800)
#define	CNT_200MS						((uint16_t)1600)
#define	CNT_500MS						((uint16_t)4000)
#define	CNT_1S							((uint16_t)8000)
#define	CNT_2S							((uint16_t)16000)
#define	CNT_3S							((uint16_t)24000)

#define INPUT_STBY_CURRENT				((uint16_t)100)			//  1 Amp
#define INPUT_OVER_CURRENT				((uint16_t)5000)		// 50 Amp

#define INPUT_OVER_VOLTAGE_CUTOUT		((uint16_t)3200)		// 32 VDC
#define INPUT_OVER_VOLTAGE_CUTIN		((uint16_t)2800)		// 28 VDC
#define INPUT_UNDER_VOLTAGE_CUTIN		((uint16_t)2000)		// 20 VDC
#define INPUT_UNDER_VOLTAGE_CUTOUT	((uint16_t)1800)		// 18 VDC


#define OUTPUT_VOLTAGE					((uint16_t)22000)		// 220 V AC
#define OUTPUT_OVER_VOLTAGE			((uint16_t)28000)		// 280 V AC
#define OUTPUT_STBY_CURRENT			((uint16_t)200)			// 0.2 A AC
#define OUTPUT_OVER_CURRENT			((uint16_t)4000)		//   4 A AC

#define OUTPUT_OVER_POWER 				((uint16_t)1200)		// 1200	WATTS

#define TEMP_30C						((uint16_t)3000)
#define TEMP_35C						((uint16_t)3500)
#define TEMP_40C						((uint16_t)4000)
#define TEMP_45C						((uint16_t)4500)
#define TEMP_50C						((uint16_t)5000)
#define TEMP_55C						((uint16_t)5500)
#define TEMP_60C						((uint16_t)6000)
#define TEMP_70C						((uint16_t)7000)
#define TEMP_80C						((uint16_t)8000)

extern volatile uint8_t 	convCompleted;;    	// Complete Conversion flag
extern volatile uint32_t 	adcPair[3];
extern volatile uint32_t 	adcVal[3];
extern volatile uint16_t	adc1[3];
extern volatile uint16_t 	adc2[3];

typedef struct
{
	uint16_t percentMod;
	uint32_t pwm_deadTime;
	uint16_t minPeriod;
	uint32_t maxPeriod;
  	uint32_t currentDuty;

}SPWM_HandleTypeDef;

typedef struct
{
	int32_t kp;
	int32_t ki;
	int32_t IntegralTerm;
	int32_t ProportionalTerm;
	int32_t error;
	int32_t xx;

}PI_HandleTypeDef;

typedef struct
{
	uint8_t rxHeader;

	uint8_t invState;
	uint8_t faultState;

	uint8_t	 vinByteHigh;
	uint8_t	 vinByteLow;

	uint8_t	 iinByteHigh;
	uint8_t	 iinByteLow;

	uint8_t	 voutByteHigh;
	uint8_t	 voutByteLow;

	uint8_t	 ioutByteHigh;
	uint8_t	 ioutByteLow;

	uint8_t	 mosfetTempByteHigh;
	uint8_t	 mosfetTempByteLow;

	uint8_t	 ambtTempByteHigh;
	uint8_t	 ambtTempByteLow;

	uint8_t inputPower_H;
	uint8_t inputPower_L;

	uint8_t outputPower_H;
	uint8_t outputPower_L;

	uint8_t chkSum;

}DISPLAY_HandleTypedef;

extern volatile uint16_t adcOffset;
extern volatile uint16_t adcVRef;
extern volatile uint16_t invVinInst;
extern volatile uint16_t invIinInst;
extern volatile uint16_t invVoutInst;
extern volatile uint16_t invIoutInst;
extern volatile uint16_t invMosfetTemp;
extern volatile uint16_t invTransfTemp;

static inline void  readADCSignals(volatile uint16_t *offset,			// for AC Current, Voltage
									 volatile uint16_t *vRef,			// ADC VoltageRef
									 volatile uint16_t *vin_inst,		// instant input voltage
									 volatile uint16_t *iin_inst,		// instant input current
									 volatile uint16_t *vout_inst,		//
									 volatile uint16_t *iout_inst		//
									 )
{
	(*offset) 		= 1650;
	(*vRef) 		= 3300;
	(*vin_inst) 	= adc1[0];
	(*iin_inst)	 	= adc2[0];
	(*vout_inst) 	= adc1[1];
	(*iout_inst) 	= adc2[1];

}

typedef struct
{
	uint16_t Value;

	uint8_t  Mode;
	uint8_t  Enter;
	uint8_t  Up;
	uint8_t  Down;
	uint8_t  Power;

}keyStatus_HandleTypedef;

typedef struct
{
	uint16_t key_Mode;
	uint16_t key_Enter;
	uint16_t key_Up;
	uint16_t key_Dwn;
	uint16_t key_Power;

	uint32_t exitMenu;		// counter for automatic exit from menu

}keyCounter_HandleTypedef;

typedef struct
{
	uint8_t  key_Mode;
	uint8_t  key_Enter;
	uint8_t  key_Up;
	uint8_t  key_Dwn;
	uint8_t  key_Power;

}keyFlag_HandleTypedef;

//---- Protection

typedef struct
{
	uint16_t Clear;

	uint16_t InputOverCurrent;

	uint16_t InputUnderVoltage;
	uint16_t InputOverVoltage;

	uint16_t OutputOverVoltage;
	uint16_t OutputOverCurrent;
	uint16_t OutputOverPower;

	uint16_t MosfetOverTemp;
	uint16_t TransOverTemp;

}protectionCounter_HandleTypedef;

typedef struct
{
	uint8_t Clear;

	uint8_t InputOverCurrent;
	uint8_t InputUnderVoltage;
	uint8_t InputOverVoltage;

	uint8_t OutputOverVoltage;
	uint8_t OutputOverCurrent;
	uint8_t OutputOverPower;	// reserved

	uint8_t MosfetOverTemp;
	uint8_t TransOverTemp;

}protectionFlag_HandleTypedef;

typedef struct
{
	uint16_t i_in_over;
	uint16_t v_in_under;
	uint16_t v_in_over;
	uint16_t v_out_over;
	uint16_t i_out_over;
	uint16_t p_out_over;	// reserved
	uint16_t t_mos_over;
	uint16_t t_trans_over;

}Fault_HandleTypedef;		// record error time

typedef struct
{
	uint8_t i_in_over;
	uint8_t v_in_under;
	uint8_t v_in_over;
	uint8_t v_out_over;
	uint8_t i_out_over;
	uint8_t p_out_over;		// reserved
	uint8_t t_mos_over;
	uint8_t t_trans_over;

}flagFault_HandleTypedef;		// flag record error time


typedef struct
{
	int16_t		mosfet;
	int16_t	  	transformer;
	uint16_t 	cnt_mosfet;
	uint16_t	cnt_transformer;

}temperature_HandleTypedef;

#endif /* INC_INVERTER_H_ */
