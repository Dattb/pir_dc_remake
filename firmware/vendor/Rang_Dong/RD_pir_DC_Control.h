/*
 * RD_pir_DC_Control.h
 *
 *  Created on: Apr 22, 2022
 *      Author: PC5
 */



#include "proj/tl_common.h"
#include "vendor/mesh/app.h"
#include "vendor/mesh_lpn/app.h"
#include "vendor/mesh_provision/app.h"
#include "vendor/mesh_switch/app.h"
#include "vendor/common/sensors_model.h"
#include "proj_lib/mesh_crypto/sha256_telink.h"
#include "vendor/common/app_heartbeat.h"
#include "vendor/common/scene.h"

#define SUART_DEBUG				0


#define SYS_32K_TICK_15S			(get_32k_tick()/(32767*15))
#define SYS_32K_TICK_S			(get_32k_tick()/32767)
#define SYS_32K_TICK_100MS			(get_32k_tick()/3276.7)
#define SYS_32K_TICK_MS			(get_32k_tick()/32.767)



#define PIR_DC_FLAG_TIMEOUT		5 	 //s
#define PIR_THESHOLD			90 	 // lan

#define PIR_DC_HIGH		GPIO_PB4
#define PIR_DC_LOW		GPIO_PB1

typedef struct{
	union{
		unsigned char data;
		struct{
			unsigned char store					:3;//LSB (Low significan bit)
			unsigned char pirHighFlag			:1;// pir flag high detect flag
			unsigned char pirLowFlag			:1;// pir flag low detect flag
			unsigned char motionOneTime			:1;// to disable pad wakeup
			unsigned char motionFlagOld			:1;// pir flag motion flag old
			unsigned char motionFlag			:1;// MSB (most significan bit)
		};
	};
}pirFlagTypeDef;


enum{
	pirHigh = 0,
	pirLow = 0,
};

typedef struct{
	unsigned char pirHighState;
	unsigned char pirLowState;
}pirStateTypeDef;






typedef struct {
	GPIO_PinTypeDef pinHigh;
	GPIO_PinTypeDef pinLow;
	pirStateTypeDef state;
	pirFlagTypeDef flag;
	unsigned int  timeOut;
}pirPinTypeDef;


void pirDCInit(pirPinTypeDef *pir);
void pirInitFlag(pirFlagTypeDef *flag);
unsigned short pirGetTimeoutReg();
unsigned char pirGetFlagReg();
unsigned char getMotionTime();
void setMotionTime(unsigned char time);
unsigned char getPollTime();
void setPollTime(unsigned char time);
unsigned char getPowerSaveReg();
void setPowerSaveReg(unsigned char data);
unsigned long lightgetLuxReg();
void lightSetLuxReg(unsigned long lux);
void pirSaveFlagReg(unsigned char data);
void pirSaveTimeoutReg(unsigned short data);
void saveFlagToAnalog(unsigned char pirHigh, unsigned char pirLow, unsigned char motionFlag);
unsigned char pirDetect(pirPinTypeDef *pin);
void RD_ADC_init (unsigned int gpio);
unsigned int RD_power_read();
void ledShowProvState();
void pirMotionCOntrol(pirFlagTypeDef *flag,unsigned char pirTime);
void readPower();
unsigned char getResetCnt();
void setResetCnt(unsigned char data);
