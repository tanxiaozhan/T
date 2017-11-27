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

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#endif



typedef enum {
	OFF,ON
} alarm_switch;      //�̵�������״̬

typedef struct {
	unsigned int yy;
	unsigned int mm;
	unsigned int dd;
} es_date;
typedef struct {
	unsigned int hour;
	unsigned int minute;
	unsigned int second;
} es_time;

typedef struct {
	es_date date;
	es_time time;
} es_date_time;

typedef struct {
	bool enable;      //����״̬��true-����
	es_time alarm_time;
	unsigned int interval;   //�ظ����������0-ÿ��һ��
} alarm_time;


void GPIO_intr_init(void);   //��ʼ���ж϶˿�
void GPIO_control_init(void);       //��ʼ�����Ƽ̵����˿�

void i2c_setup(void);
//DS3231ʵʱ�����ڡ�ʱ���д����
void DS3231_read_time(void);
void DS3231_write_time(void);
void DS3231_read_date(void);
void DS3231_write_date(void);

void read_flash_alarm(void);
void write_flash_alarm(void);
void turn_on_off(uint8 control);        //����GPIO4�����/�͵�ѹ����/�رռ̵���

void check_alarm(void);
