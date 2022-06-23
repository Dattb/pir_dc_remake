/*
 * RD_pir_DC_Control.c
 *
 *  Created on: Apr 22, 2022
 *      Author: PC5
 */

#include "RD_pir_DC_Control.h"
#include "SoftUart.h"
#include "RD_Light_Sensor.h"
#include "RD_control.h"


extern unsigned char uartBuff[64];

extern u16 rd_box_addr;
extern u16 rd_node_addr;


unsigned char MotionTimeLine = 0;
//unsigned char pirDetect(){
//
//}

pirPinTypeDef rdPir;


void pirDCInit(pirPinTypeDef *pir){
	pir->pinHigh = PIR_DC_HIGH;
	pir->pinLow = PIR_DC_LOW;
	pir->timeOut = pirGetTimeoutReg();
	pir->flag.data = pirGetFlagReg();
	rdPrintf("init: %d - %d\n",pir->flag.motionFlag,pir->flag.motionFlagOld);
}


unsigned char pirDetect(pirPinTypeDef *pin){
	if(pin->timeOut <= SYS_32K_TICK_S){
		if(SYS_32K_TICK_S - pin->timeOut >= PIR_DC_FLAG_TIMEOUT ){
			#if(SUART_DEBUG)
				rdPrintf("clear %d\n",SYS_32K_TICK_S);
			#endif
			pin->timeOut = SYS_32K_TICK_S;
			pirSaveTimeoutReg(pin->timeOut);
			pin->flag.pirHighFlag = 0;
			pin->flag.pirLowFlag = 0;
			pirSaveFlagReg(pin->flag.data);
		}
	}
	else{
		if((0xffffffff - pin->timeOut + SYS_32K_TICK_S) >= PIR_DC_FLAG_TIMEOUT){
			pin->timeOut = SYS_32K_TICK_S;
			pirSaveTimeoutReg(pin->timeOut);
			pin->flag.pirHighFlag = 0;
			pin->flag.pirLowFlag = 0;
			pirSaveFlagReg(pin->flag.data);
		}
	}

	if(pin->state.pirHighState){
		unsigned char checkPullupTime = 0;
		for(unsigned short i=0;i<100;i++){
			sleep_us(50);
			if(gpio_read(pin->pinHigh)){
				checkPullupTime++;
				if(checkPullupTime >= PIR_THESHOLD){
					pin ->flag.pirHighFlag = 1;
					pin->timeOut = SYS_32K_TICK_S;
					pirSaveFlagReg(pin ->flag.data);
					pirSaveTimeoutReg(pin->timeOut);
					break;
				}
			}
			else {
				pin ->flag.pirHighFlag = 0;
				pirSaveFlagReg(pin->flag.data);
				checkPullupTime = 0;
				break;
			}
		}
	}

	if(pin->state.pirLowState){
		unsigned char checkPullupTime = 0;
		for(unsigned short i=0;i<100;i++){
			sleep_us(50);
			if(gpio_read(pin->pinLow)){
				checkPullupTime++;
				if(checkPullupTime >= PIR_THESHOLD){
					pin ->flag.pirLowFlag = 1;
					pin->timeOut = SYS_32K_TICK_S;
					pirSaveFlagReg(pin->flag.data);
					pirSaveTimeoutReg(pin->timeOut);
					break;
				}
			}
			else {
				pin ->flag.pirLowFlag = 0;
				pirSaveFlagReg(pin->flag.data);
				checkPullupTime = 0;
				break;
			}
		}
	}

	if(pin->flag.pirHighFlag && pin->flag.pirLowFlag){
		pin->flag.motionFlag = 1;
		pin->flag.motionOneTime = 1;
	}

	#if(SUART_DEBUG)
		rdPrintf("hi:%d - low:%d - mo:%d\n",pin->flag.pirHighFlag,pin->flag.pirLowFlag,pin->flag.motionFlag);
	#endif
	return (pin->flag.motionFlag);
}


void sendMotion(unsigned char motionEdge){
	if(motionEdge){
		unsigned char pir_send_buff[2] = {0x0c,0x01};
		mesh_tx_cmd2normal(0xe5,(u8 *) pir_send_buff,2,rd_node_addr,rd_box_addr,2);
	}
}

void pirSaveTimeoutReg(unsigned short data){
	analog_write(PIR_TIMEOUT_REG0,(unsigned char)(data>>8));
	analog_write(PIR_TIMEOUT_REG1,(unsigned char)(data));
}



unsigned short pirGetTimeoutReg(){
	 unsigned long rdTick_32k = 0;
	 rdTick_32k  = analog_read(PIR_TIMEOUT_REG0)<<8 |analog_read(PIR_TIMEOUT_REG1);
	 return rdTick_32k;
}


void lightSetLuxReg(unsigned long lux){
	analog_write(lUX_SAVE_REG0,(unsigned char)(lux>>8));
	analog_write(lUX_SAVE_REG1,(unsigned char)(lux));
}

unsigned long lightgetLuxReg(){
	unsigned long luxData;
	luxData = analog_read(lUX_SAVE_REG0)<<8|analog_read(lUX_SAVE_REG1);
	return luxData;
}

void pirSaveFlagReg(unsigned char data){
	analog_write(PIR_FLAG_SAVE_REG,data);
}

unsigned char pirGetFlagReg(){
	return analog_read(PIR_FLAG_SAVE_REG);
}

unsigned char getMotionTime(){
	return analog_read(MOTION_TIME);
}

void setMotionTime(unsigned char time){
	analog_write(MOTION_TIME,time);
}

unsigned char getPollTime(){
	return analog_read(POLL_TIME);
}

void setPollTime(unsigned char time){
	analog_write(POLL_TIME,time);
}

unsigned char getPowerSaveReg(){
	//return analog_read(POWER_SAVE_REG);
	return 0;
}



void setPowerSaveReg(unsigned char data){
	 //analog_write(POWER_SAVE_REG,data);
}

void setResetCnt(unsigned char data){
	//analog_write(RESET_CNT_REG,data);
}

unsigned char getResetCnt(){
	//return analog_read(RESET_CNT_REG);
	return 0;
}

void readPower()
{
	//u16 Power_Data = RD_power_read();
}

void RD_ADC_init (unsigned int gpio)
{
	adc_init();
	adc_base_init(gpio);
	adc_power_on_sar_adc(1);
}

unsigned int RD_power_read()
{
    RD_ADC_init (GPIO_PC4);
	unsigned int power_read;
	float power_persent = 0;
    //sleep_us(2000);
	power_read = adc_sample_and_get_result();
//	if(power_read<=1070)
//	{
//		power_persent=  power_read*0.0513-45.82;   // %pin = vol*0.0513-45.82
//	}
//	else if (power_read<=1174)
//	{
//		power_persent=  power_read*0.2449-254.62; //y = 0.2449x - 254.62
//	}
//	else if (power_read<=1404)
//	{
//		power_persent=  power_read*0.3084-333.94;          //y = 0.3084x - 333.94
//	}
//	else if(power_read<1480) {
//		power_persent=  power_read*0.1273-89.77;  	//y = 0.1273x - 89.77
//	}
//	else power_persent=100;
	power_persent = (2.0*power_read-2200)/(3200-2200)*100;
	power_persent = (unsigned int) power_persent;
	return power_persent;
	//return power_read;

}


void ledShowProvState(){
	if(is_provision_success()){
		gpio_write(LED_RED_PIN,1);
		gpio_write(LED_BLUE_PIN,0);
	}else{
		gpio_write(LED_RED_PIN,0);
		gpio_write(LED_BLUE_PIN,1);
	}
}


void pirMotionCOntrol(pirFlagTypeDef *flag,unsigned char pirTime){
	MotionTimeLine = getMotionTime();
	if(flag->motionFlag){
		//rdPrintf("count\n");
		if( MotionTimeLine <= SYS_32K_TICK_15S ){
			rdPrintf("count1:%d time: \n",SYS_32K_TICK_15S - MotionTimeLine,pirTime/15);
			//rdPrintf("time0 %d\n",SYS_32K_TICK_15S);
			if(SYS_32K_TICK_15S - MotionTimeLine >= pirTime/15 ){
				#if(SUART_DEBUG)
					rdPrintf("clear %d\n",SYS_32K_TICK_S);
				#endif
				flag->motionFlag = 0;
				rdPrintf("reset1 %d-%d\n",MotionTimeLine,SYS_32K_TICK_15S - MotionTimeLine);
				MotionTimeLine = SYS_32K_TICK_15S;
				setMotionTime(MotionTimeLine);
			}
		}
		else{
			unsigned char rdTick = SYS_32K_TICK_15S;
			if(0xff - MotionTimeLine + rdTick >= pirTime){
				#if(SUART_DEBUG)
					rdPrintf("clear %d\n",SYS_32K_TICK_S);
				#endif
				flag->motionFlag = 0;
				rdPrintf("reset2 %d-%d\n",MotionTimeLine,SYS_32K_TICK_S);
				MotionTimeLine = SYS_32K_TICK_15S;
				setMotionTime(MotionTimeLine);
			}
		}
	}
}
