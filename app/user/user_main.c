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


// �ж�GPIO�Ŷ��壬�ڴ�ʹ��GPIO5
#define INTR_PIN         GPIO_Pin_5
#define INTR_PIN_FUNC        FUNC_GPIO5
#define INTR_PIN_MUX         PERIPHS_IO_MUX_GPIO5_U

//���Ӽ̵�����GPIO�ܽţ��ڴ�ΪGPIO4
#define SWITCH_CONTROL_GPIO_PIN 4
#define SWITCH_CONTROL_PIN_FUNC FUNC_GPIO4
#define SWITCH_CONTROL_PIN_MUX  PERIPHS_IO_MUX_GPIO4_U


//����AP SSIDΪesp8266_XXXXXX,XΪMAC��ַ��������ַ
#define ESP_AP_SSID      "esp8266_%02x%02x%02x"
#define ESP_AP_PASSWORD  "12345678"

#define SOFTAP_IF       0x01

#define ALARM_SECTOR 100
//�Ѷ�ʱ���ݱ��浽flash�ĵڼ�������ÿ����4KB��flash��д����4�ֽڶ���

#define ALARM_ON_FLASH_ADDRESS ALARM_SECTOR * 4 * 1024 + 0
/*��flash�б���Ķ�ʱ�����׵�ַ����flash�б�����5�鶨ʱ���ݣ�
ÿ��4bytes����ʽ��ʱ���֣��룬���,��ʱ��Ϣ�е�����״̬enable��Сʱ����һ���ֽڣ�
Сʱ�ֽ��е����λ��������״̬��1-���ã�0-�ر�
*/
#define ALARM_OFF_FLASH_ADDRESS ALARM_ON_FLASH_ADDRESS + 40  //��flash�б���Ķ�ʱ�ص��׵�ַ����flash�б�����5�鶨ʱ���ݣ�ÿ��4bytes����ʽ��ʱ���֣��룬����

#define ALARM_NUM     5    //���ʱ������

es_date_time es_now;         //��ǰʱ��
alarm_time alarm_on[ALARM_NUM];       //���鶨ʱ
es_time alarm_off[ALARM_NUM];
uint8 interval[ALARM_NUM];      //��ʱ�����ظ���������0-ÿ���ظ���1-ÿ�����ظ�

alarm_switch switch_state;      //�̵�������״̬

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
 ����ESP8266ΪAPģʽ��������IP��DHCP��������ַ��
��������
���أ���
********************************************************/
bool wifi_softap_setup( void ){

    bool ap_init_result;   //��¼wifi_softap��ʼ�����

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
 *��ⶨʱʱ��
 *
*******************************************************************************/
void check_alarm(void){
	uint8 i;
	for(i=0;i<ALARM_NUM;i++){
		if(alarm_on[i].enable){
			if (alarm_on[i].alarm_time.hour == es_now.time.hour &&
				alarm_on[i].alarm_time.minute == es_now.time.minute &&
				alarm_on[i].alarm_time.second == es_now.time.second){
					turn_on_off(1);   //�򿪼̵���
			}

			if(switch_state == ON){
				if(alarm_off[i].hour==es_now.time.hour &&
				   alarm_off[i].minute==es_now.time.minute &&
				   alarm_off[i].second==es_now.time.second){
						turn_on_off(0);    //�رռ̵���
				}
			}
		}
	}
}


/******************************************************************************
 *
 *ʹ��I2C���ߴ�DS3231ʵʱ��оƬ��ȡʵʱ��
 *
*******************************************************************************/
void DS3231_read_time(void){


}


/******************************************************************************
 *
 *��/�رռ̵���
 *
*******************************************************************************/
void turn_on_off(uint8 control){
	if(control==1){
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SWITCH_CONTROL_GPIO_PIN),1);  //GPIO4����ߵ�ƽ���̵���ͨ��
		switch_state=ON;
	}
	else
	{
		GPIO_OUTPUT_SET(GPIO_ID_PIN(SWITCH_CONTROL_GPIO_PIN),0);  //GPIO4����͵�ƽ���̵����ر�
		switch_state=OFF;
	}

}


/******************************************************************************
 *
 *DS3231���жϴ�����
 *
*******************************************************************************/
void GPIO_intr_handler(void){
	u32 pin_status=0;
	pin_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );      // ��ȡGPIO�ж�״̬�����ж����Ǹ��˿ڵ��ж�

/*
	ETS_GPIO_INTR_DISABLE();                            // �ر�GPIO�ж�
    if ( pin_status & BIT(INTR_PIN) )      //GPIO5���ж�
    {
    	DS3231_read_time();
    	check_alarm();

    }
    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, pin_status );   // ���GPIO�жϱ�־
    ETS_GPIO_INTR_ENABLE();                               // ����GPIO�ж�
*/

    _xt_isr_mask(1<<ETS_GPIO_INUM);    //disable interrupt

    if ( pin_status & INTR_PIN )      //GPIO5���ж�
    {
    	DS3231_read_time();
    	check_alarm();
    }

	GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, pin_status ); //clear interrupt mask
    _xt_isr_unmask(1 << ETS_GPIO_INUM); //Enable the GPIO interrupt

  /* �ر����⣺���жϻص������У�
         ��disable interrupt֮��������ж��źŴ����ж����ţ���ʹ��δʹ���жϣ���interrupt mask��Ȼ�ᱻ��Ϊ�жϱ�־��
    ��ˣ�Ϊ����Enable the GPIO interrupt֮��������֮ǰ�ж�λ��Ӱ��������ٴ��жϣ�����_xt_isr_unmask(1 << ETS_GPIO_INUM);   //Enable the GPIO interrupt֮ǰ��ؼ���
       * GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, TRUE ); //clear interrupt mask
    */


}


/******************************************************************************
 *
 *����PIO5Ϊ�ⲿ�ж϶˿ڣ��͵�ƽ������ʽ����ӦDS3231ʵʱ��оƬ���������ж��ź�
 *
*******************************************************************************/
void GPIO_intr_init(void){
	/*
	PIN_FUNC_SELECT(INTR_PIN_MUX, INTR_PIN_FUNC);    //����̹ܽ���  ��GPIOʹ��
	GPIO_DIS_OUTPUT(INTR_PIN_FUNC);  //����GPIO5Ϊ����״̬
    PIN_PULLUP_EN(INTR_PIN_MUX);        //ʹ������

	ETS_GPIO_INTR_DISABLE();     //�ر��ж�

	ETS_GPIO_INTR_ATTACH(&GPIO_intr_handler, NULL );     //�����жϺ���
    gpio_pin_intr_state_set( GPIO_ID_PIN(INTR_PIN_NUM), GPIO_PIN_INTR_LOLEVEL );      //���õ͵�ƽ�жϴ�����ʽ
    //gpio_output_set(0,0,0,BIT5);                  //ʹ������

    GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS,  BIT(INTR_PIN_NUM));           //��������ŵ�GPIO�жϱ�־
    ETS_GPIO_INTR_ENABLE();                          //�ж�ʹ��
    */

    GPIO_ConfigTypeDef gpio_in_cfg;    //Define GPIO Init Structure
    gpio_in_cfg.GPIO_IntrType = GPIO_PIN_INTR_NEGEDGE;    //�½��ش���
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
 *ÿ���ϵ磬��FLASH��ȡ��ʱʱ��
 *
*******************************************************************************/
void read_flash_alarm(void){
	SpiFlashOpResult read_result;
	uint32 alarm_data[ALARM_NUM];
	int i;
	uint8 temp;
	//��flash��ȡ��ʱ��������
	read_result=spi_flash_read(ALARM_ON_FLASH_ADDRESS, alarm_data, ALARM_NUM * 4);
	if(read_result==SPI_FLASH_RESULT_OK){
		//�жϴ�flash��ȡ�Ķ�ʱ���ݵ���Ч��
		for(i=0;i<5;i++){
			temp = (uint8)(alarm_data[i]>>24);
			if((temp & 0x80)==0)
				alarm_on[i].enable=0;
			else
				alarm_on[i].enable=1;

			temp = temp & 0x7F;
			if(temp<24)   //Сʱ�ֽ������λΪ��ʱ����״̬��1-���ã�0-������
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

	//��flash��ȡ��ʱ������
	read_result=spi_flash_read(ALARM_OFF_FLASH_ADDRESS, alarm_data, ALARM_NUM * 4);
	if(read_result==SPI_FLASH_RESULT_OK){
		//�жϴ�flash��ȡ�Ķ�ʱ���ݵ���Ч��
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
 *�Ѷ�ʱ����д��FLASH
 *
*******************************************************************************/
void write_flash_alarm(void){
	SpiFlashOpResult write_result;
	uint32 alarm_data[ALARM_NUM];
	int i;
	write_result=spi_flash_erase_sector(ALARM_SECTOR);   //д����ǰ������ɾ��
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
 *��ʼ���̵������ƶ˿�
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

    //��ʼ��wifi
    wifi_softap_setup();

    //��FLASH��ȡ��ʱʱ��
    read_flash_alarm();

    esp_tcp_server();

    GPIO_intr_init();
    GPIO_control_init();

}

