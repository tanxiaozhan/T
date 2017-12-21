/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"

//#include "esp8266/ets_sys.h"
#include "c_types.h"
#include "uart.h"

#include "esp_libc.h"
#include "user_config.h"
#include "../include/gpio.h"


// 中断GPIO脚定义，在此使用GPIO5
#define INTR_PIN         GPIO_Pin_5
#define INTR_PIN_FUNC        FUNC_GPIO5
#define INTR_PIN_MUX         PERIPHS_IO_MUX_GPIO5_U

//连接继电器的GPIO管脚，在此为GPIO4
#define SWITCH_CONTROL_GPIO_PIN 4
#define SWITCH_CONTROL_PIN_FUNC FUNC_GPIO4
#define SWITCH_CONTROL_PIN_MUX  PERIPHS_IO_MUX_GPIO4_U


//设置AP SSID为esp8266_XXXXXX,X为MAC地址后三个地址
#define ESP_AP_SSID      "esp8266_%02x%02x%02x"
#define ESP_AP_PASSWORD  "12345678"

#define SOFTAP_IF       0x01

#define ALARM_SECTOR 100
//把定时数据保存到flash的第几扇区，每扇区4KB，flash读写必须4字节对齐

#define ALARM_ON_FLASH_ADDRESS ALARM_SECTOR * 4 * 1024 + 0
/*在flash中保存的定时开的首地址，在flash中保存着5组定时数据，
每组4bytes，格式：时，分，秒，间隔,定时信息中的启用状态enable与小时合用一个字节，
小时字节中的最高位保存启用状态：1-启用，0-关闭
*/
#define ALARM_OFF_FLASH_ADDRESS ALARM_ON_FLASH_ADDRESS + 40  //在flash中保存的定时关的首地址，在flash中保存着5组定时数据，每组4bytes，格式：时，分，秒，保留

#define ALARM_NUM     5    //最大定时组数量

es_date_time es_now;         //当前时间
alarm_time alarm_on[ALARM_NUM];       //五组定时
es_time alarm_off[ALARM_NUM];
uint8 interval[ALARM_NUM];      //定时器间重复隔天数，0-每天重复，1-每二天重复

alarm_switch switch_state;      //继电器开关状态

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


/********************************************************
 设置ESP8266为AP模式，并设置IP及DHCP服务器地址池
参数：无
返回：无
********************************************************/
bool wifi_softap_setup( void ){

    bool ap_init_result;   //记录wifi_softap初始化结果

	ap_init_result = wifi_set_opmode(SOFTAP_MODE);

    ap_init_result &= wifi_softap_dhcps_stop();

    struct ip_info info;
    IP4_ADDR(&info.ip, 192, 168, 10, 254);  // set IP
    IP4_ADDR(&info.gw, 192, 168, 10, 254);    // set gateway
    IP4_ADDR(&info.netmask, 255, 255, 255, 0); // set netmask
    ap_init_result &= wifi_set_ip_info(SOFTAP_IF, &info);

    struct dhcps_lease esp_dhcps_lease;
    IP4_ADDR(&esp_dhcps_lease.start_ip,192,168,10,1);
    IP4_ADDR(&esp_dhcps_lease.end_ip,192,168,10,5);
    ap_init_result &= wifi_softap_set_dhcps_lease(&esp_dhcps_lease);

    ap_init_result &= wifi_softap_dhcps_start();


    char softap_mac[6];
    wifi_get_macaddr(STATION_IF, softap_mac);

    struct softap_config *config = (struct softap_config *)os_zalloc(sizeof(struct softap_config ));
    wifi_softap_get_config(config); // Get soft-AP config first.
    sprintf(config->ssid, ESP_AP_SSID,softap_mac[3],softap_mac[4],softap_mac[5]);
    sprintf(config->password, ESP_AP_PASSWORD);
    config->authmode = AUTH_WPA2_PSK;
    config->ssid_len = 0;        // or its actual SSID length
    config->max_connection = 2;

    ap_init_result &= wifi_softap_set_config(config); // Set ESP8266 soft-AP config

    os_free(config);


    if(ap_init_result)
    	os_printf("softap setup success!\n");
    else
    	os_printf("softap setup fail!\n");

    return ap_init_result;
}


/******************************************************************************
 *
 *检测定时时间
 *
*******************************************************************************/
void check_alarm(void){
	uint8 i;
	for(i=0;i<ALARM_NUM;i++){
		if(alarm_on[i].enable){
			if (alarm_on[i].alarm_time.hour == es_now.time.hour &&
				alarm_on[i].alarm_time.minute == es_now.time.minute &&
				alarm_on[i].alarm_time.second == es_now.time.second){
					turn_on_off(1);   //打开继电器
			}

			if(switch_state == ON){
				if(alarm_off[i].hour==es_now.time.hour &&
				   alarm_off[i].minute==es_now.time.minute &&
				   alarm_off[i].second==es_now.time.second){
						turn_on_off(0);    //关闭继电器
				}
			}
		}
	}
}


/******************************************************************************
 *
 *使用I2C总线从DS3231实时钟芯片获取实时钟
 *
*******************************************************************************/
void DS3231_read_time(void){


}


/******************************************************************************
 *
 *打开/关闭继电器
 *
*******************************************************************************/
void turn_on_off(uint8 control){
	if(control==1){
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SWITCH_CONTROL_GPIO_PIN),1);  //GPIO4输出高电平，继电器通电
		switch_state=ON;
	}
	else
	{
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SWITCH_CONTROL_GPIO_PIN),0);  //GPIO4输出低电平，继电器关闭
		switch_state=OFF;
	}

}


/******************************************************************************
 *
 *DS3231秒中断处理函数
 *
*******************************************************************************/
void GPIO_intr_handler(void){
	u32 pin_status=0;
	pin_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );      // 读取GPIO中断状态可以判断是那个端口的中断

/*
	ETS_GPIO_INTR_DISABLE();                            // 关闭GPIO中断
    if ( pin_status & BIT(INTR_PIN) )      //GPIO5的中断
    {
    	DS3231_read_time();
    	check_alarm();

    }
    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, pin_status );   // 清除GPIO中断标志
    ETS_GPIO_INTR_ENABLE();                               // 开启GPIO中断
*/

    _xt_isr_mask(1<<ETS_GPIO_INUM);    //disable interrupt

    if ( pin_status & INTR_PIN )      //GPIO5的中断
    {
    	DS3231_read_time();
    	check_alarm();
    }

	GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, pin_status ); //clear interrupt mask
    _xt_isr_unmask(1 << ETS_GPIO_INUM); //Enable the GPIO interrupt

  /* 特别留意：在中断回调函数中，
         在disable interrupt之后，如果有中断信号触发中断引脚，即使还未使能中断，但interrupt mask仍然会被置为中断标志，
    因此，为避免Enable the GPIO interrupt之后，由于受之前中断位的影响而马上再次中断，故在_xt_isr_unmask(1 << ETS_GPIO_INUM);   //Enable the GPIO interrupt之前务必加上
       * GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, TRUE ); //clear interrupt mask
    */


}


/******************************************************************************
 *
 *设置PIO5为外部中断端口，低电平触发方式，响应DS3231实时钟芯片产生的秒中断信号
 *
*******************************************************************************/
void GPIO_intr_init(void){
	/*
	PIN_FUNC_SELECT(INTR_PIN_MUX, INTR_PIN_FUNC);    //定义管教功能  作GPIO使用
	GPIO_DIS_OUTPUT(INTR_PIN_FUNC);  //设置GPIO5为输入状态
    PIN_PULLUP_EN(INTR_PIN_MUX);        //使能上拉

	ETS_GPIO_INTR_DISABLE();     //关闭中断

	ETS_GPIO_INTR_ATTACH(&GPIO_intr_handler, NULL );     //设置中断函数
    gpio_pin_intr_state_set( GPIO_ID_PIN(INTR_PIN_NUM), GPIO_PIN_INTR_LOLEVEL );      //设置低电平中断触发方式
    //gpio_output_set(0,0,0,BIT5);                  //使能输入

    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS,  BIT(INTR_PIN_NUM));           //清除该引脚的GPIO中断标志
    ETS_GPIO_INTR_ENABLE();                          //中断使能
    */

    GPIO_ConfigTypeDef gpio_in_cfg;    //Define GPIO Init Structure
    gpio_in_cfg.GPIO_IntrType = GPIO_PIN_INTR_NEGEDGE;    //下降沿触发
    gpio_in_cfg.GPIO_Mode = GPIO_Mode_Input;    //Input mode
    gpio_in_cfg.GPIO_Pullup = GPIO_PullUp_EN;
    gpio_in_cfg.GPIO_Pin = INTR_PIN;    // Enable GPIO
    gpio_config(&gpio_in_cfg);    //Initialization function

    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, INTR_PIN);
    gpio_intr_handler_register(GPIO_intr_handler, NULL); // Register the interrupt function
    _xt_isr_unmask(1 << ETS_GPIO_INUM);    //Enable the GPIO interrupt
}


/******************************************************************************
 *
 *每次上电，从FLASH读取定时时间
 *
*******************************************************************************/
void read_flash_alarm(void){
	SpiFlashOpResult read_result;
	uint32 alarm_data[ALARM_NUM];
	int i;
	uint8 temp;
	//从flash读取定时开启数据
	read_result=spi_flash_read(ALARM_ON_FLASH_ADDRESS, alarm_data, ALARM_NUM * 4);
	if(read_result==SPI_FLASH_RESULT_OK){
		//判断从flash读取的定时数据的有效性
		for(i=0;i<5;i++){
			temp = (uint8)(alarm_data[i]>>24);
			if((temp & 0x80)==0)
				alarm_on[i].enable=0;
			else
				alarm_on[i].enable=1;

			temp = temp & 0x7F;
			if(temp<24)   //小时字节中最高位为定时启用状态：1-启用，0-不启用
				alarm_on[i].alarm_time.hour=temp;

			temp = (uint8)((alarm_data[i]>>16) & 0x00000000FF);
			if(temp<60)
				alarm_on[i].alarm_time.minute=temp;

			temp = (uint8)((alarm_data[i]>>8) & 0x00000000FF);
			if(temp<60)
				alarm_on[i].alarm_time.second=temp;

			alarm_on[i].interval=(uint8)(alarm_data[i] & 0x00000000FF);
		}
	}

	//从flash读取定时关数据
	read_result=spi_flash_read(ALARM_OFF_FLASH_ADDRESS, alarm_data, ALARM_NUM * 4);
	if(read_result==SPI_FLASH_RESULT_OK){
		//判断从flash读取的定时数据的有效性
		for(i=0;i<5;i++){
			temp=(uint8)(alarm_data[i]>>24);
			if(temp<24)
				alarm_off[i].hour = temp;

			temp=(uint8)((alarm_data[i]>>16) & 0x00000000FF);
			if(temp<60)
				alarm_off[i].minute = temp;

			temp=(uint8)((alarm_data[i]>>8) & 0x00000000FF);
			if(temp<60)
				alarm_off[i].second = temp;
		}
	}
}


/******************************************************************************
 *
 *把定时数据写入FLASH
 *
*******************************************************************************/
void write_flash_alarm(void){
	SpiFlashOpResult write_result;
	uint32 alarm_data[ALARM_NUM];
	int i;
	write_result=spi_flash_erase_sector(ALARM_SECTOR);   //写数据前，须先删除
	if(write_result != SPI_FLASH_RESULT_OK)
		return;

	for(i=0;i<5;i++){
		if(alarm_on[i].enable)
			alarm_data[i]=alarm_on[i].alarm_time.hour | 0x80;
		else
			alarm_data[i]=(uint32)(alarm_on[i].alarm_time.hour & 0x7F);
		alarm_data[i]<<=24;

		alarm_data[i] |= (uint32)alarm_on[i].alarm_time.minute << 16 ;
		alarm_data[i] |= (uint32)alarm_on[i].alarm_time.second << 8 ;
		alarm_data[i] |= alarm_on[i].interval;
	}
	write_result=spi_flash_write(ALARM_ON_FLASH_ADDRESS,alarm_data,ALARM_NUM*4);

	for(i=0;i<5;i++){
		alarm_data[i] = (uint32)alarm_off[i].hour <<24 ;
		alarm_data[i] |= (uint32)alarm_off[i].minute << 16 ;
		alarm_data[i] |= (uint32)alarm_off[i].second << 8 ;
	}
	write_result=spi_flash_write(ALARM_OFF_FLASH_ADDRESS,alarm_data,ALARM_NUM*4);

}

/******************************************************************************
 *
 *初始化继电器控制端口
 *
*******************************************************************************/
void GPIO_control_init(void){
	PIN_FUNC_SELECT(SWITCH_CONTROL_PIN_MUX,SWITCH_CONTROL_PIN_FUNC);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(SWITCH_CONTROL_GPIO_PIN),0);
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
	uart_init_new();
	printf("Ai-Thinker Technology Co. Ltd.\r\n%s %s\r\n", __DATE__, __TIME__);
	printf("Hello,World!\r\n");

    printf("SDK version:%s\n", system_get_sdk_version());

    //初始化wifi
    wifi_softap_setup();

    //从FLASH读取定时时间
    read_flash_alarm();

    esp_tcp_server();

    GPIO_intr_init();
    GPIO_control_init();

}

