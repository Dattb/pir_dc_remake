/*
 * RD_Light_Sensor.c
 *
 *  Created on: May 10, 2021
 *      Author: Dat_UTC
 */




#include "RD_Light_Sensor.h"
#include "RD_pir_DC_Control.h"
#include "SoftUart.h"
float Kp = 90;
extern u16 rd_node_addr;
extern u16 rd_box_addr;


u8 RD_i2c_rx_buff[2] = {0};
u8 RD_i2c_tx_buff[2] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};
u8 light_sensor_to_gw_tx_buff[6]= {0};

extern u16 Lux_val_new;

unsigned int CalculateLux(unsigned int rsp_lux)
{
	unsigned int lux_LSB = 0;
	unsigned char lux_MSB = 0;
	unsigned int lux_Value = 0;
	unsigned int pow = 1;
	unsigned char i;
	lux_LSB = rsp_lux & 0x0FFF;
	lux_MSB = ((rsp_lux>>12) & 0x0F);
	//Lux_Value = 0.01 * pow(2,Lux_MSB) * Lux_LSB; //don't use
	for(i=0;i<lux_MSB;i++){
		pow=pow*2;
	}
	lux_Value = 0.01 * pow * lux_LSB;
	// RD_EDIT: calib lux
	#if	LUX_UART_DEBUG
		char UART_TempSend[128];
		sprintf(UART_TempSend,"Lux befor calib %d \n",lux_Value);
		uart_CSend(UART_TempSend);
	#endif
	if(lux_Value > 60){
		int calib_lux = 1.7339 * lux_Value - 39.868;
		lux_Value = calib_lux;
	}
	return lux_Value;
}


u16 light_sensor_read_cnt=0;
u32 time_send_lux;
unsigned int  real_lux_new = 0;
unsigned long luxOld = 0;
unsigned char luxLoop(unsigned long loopTime){
	if(pirGetTimeoutReg() % loopTime == 0){
		#if(SUART_DEBUG)
				rdPrintf("send lux\n");
		#endif
		return 1;
	}
	return 0;
}

unsigned char LuxDetect(unsigned int threshold){
	unsigned char change_lux_flag = 0;
	luxOld = lightgetLuxReg();
	//rdPrintf("L1:%d\n",luxOld);
	real_lux_new = CalculateLux(RD_Light_Sensor_read_raw_val());
	//rdPrintf("L2:%d\n",real_lux_new);
	if(luxOld != real_lux_new){
		if(luxOld >= real_lux_new){
			if(luxOld - real_lux_new >= threshold){
				change_lux_flag = 1;
				luxOld = real_lux_new;
				lightSetLuxReg(luxOld);
				#if(SUART_DEBUG)
					rdPrintf("snd_L\n",real_lux_new);
				#endif
			}
		}
		else {
			if((real_lux_new - luxOld >= threshold)){
				change_lux_flag = 1;
				luxOld = real_lux_new;
				lightSetLuxReg(luxOld);
				#if(SUART_DEBUG)
					rdPrintf("snd_L\n",real_lux_new);
				#endif
			}
		}
	}
	return change_lux_flag;
}

void lightSendData(unsigned char power){

	#if(SUART_DEBUG)
		rdPrintf("send_L\n");
	#endif
	unsigned char lux_send_buff[4] = {0};
	lux_send_buff[0] = 0x0d;
	lux_send_buff[1] = luxOld;
	lux_send_buff[2] = luxOld >>8;
	lux_send_buff[3] = power;
	mesh_tx_cmd2normal(0xe5,(u8 *) lux_send_buff,4, rd_node_addr ,rd_box_addr, 2);

}

unsigned int RD_Light_Sensor_read_raw_val() {
	u16  lux_read_data;
	u8 RD_i2c_rx_buff[3] = {0};
	u8 RD_i2c_tx_buff[2] = {OPT3001_CONFIG_REG_HIGH,OPT3001_CONFIG_REG_LOW};

	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));

	i2c_write_series(OPT3001_CONFIG_REGISTER,OPT3001_CONFIG_REGISTER_LEN,(u8 *)RD_i2c_tx_buff, 2);
	i2c_read_series(OPT3001_RESULT_REGISTER,OPT3001_RESULT_REGISTER_LEN, (u8 *)RD_i2c_rx_buff, 3);

	lux_read_data = (RD_i2c_rx_buff[0]<<8) | RD_i2c_rx_buff[1];
	return lux_read_data;
}


void RD_Send_raw_Lux(u16 Lux_raw_val,unsigned int lux_real)
{
	*(light_sensor_to_gw_tx_buff)   = 0x04;
	*(light_sensor_to_gw_tx_buff+1) = 0x00;
	*(light_sensor_to_gw_tx_buff+2)   = (u8)(Lux_raw_val>>8);
	*(light_sensor_to_gw_tx_buff+3) = (u8)(Lux_raw_val);
	*(light_sensor_to_gw_tx_buff+4)   = (u8)(lux_real>>8);
	*(light_sensor_to_gw_tx_buff+5) = (u8)(lux_real);
	mesh_tx_cmd2normal_primary(SENSOR_STATUS, (u8 *)light_sensor_to_gw_tx_buff, 6, 0x0001, 2);
}

void Light_sensor_i2c_init(){
	i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
	i2c_master_init(SLAVE_DEVICE_ADDR,(unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLK_SPEED)));
}
unsigned int Lux_clock_time_read = 0;
void Lux_loop (unsigned int Lux_loop_time){
	if(clock_time_ms() - Lux_clock_time_read >= Lux_loop_time){
		Lux_clock_time_read = clock_time_ms();
		//Lux_send(LUX_THRESH_HOLD);
	}
}
