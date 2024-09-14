/*
 * uart_frame_ptc.c
 *
 *  Created on: Sep 8, 2024
 *      Author: Jewel James
 */

#include "uart_frame_ptc.h"

#define RX_BUFF_LEN			100
#define MAX_PARAM_COUNT		100
#define HEADER_NO_BYTES		3
#define DEVICE_ID_1			0xAB
#define DEVICE_ID_2			0xCD

#define TIMEOUT				5000

#define SCALE_DWN_FACTOR	0.1
#define SCALE_PARAM_COUNT	3

uint8_t rx_buffer[RX_BUFF_LEN];
float param_arr[RX_BUFF_LEN/2];
uint8_t rx_idx, rx_fail, byte_idx, rx_cplt;
uint8_t no_of_params;
uint8_t scale_id_arr[] = {1, 4, 2};

uint32_t uart_frame_timer;

uint16_t _crc16(const uint8_t *, uint8_t);
void _clearbuffer(uint8_t *);
/**
 * Frame format:
 * [DEVID 1] [DEVID 2] [PARAM COUNT] [P1 HB] [P1 LB] ... [Pn LB] [CRC LB] [CRC HB]
 */

/**
 * @brief	byte check--add in UART callback
 * @param	byte
 */
void uart_frame_check(uint8_t byte) {
	static uint8_t crc_low = 0, crc_high = 0;
	switch(rx_idx) {
	case 0:
		if(byte == DEVICE_ID_1) rx_idx++;
		break;
	case 1:
		if(byte == DEVICE_ID_2)
			rx_idx++;
		else
			rx_idx = 0;
		break;
	case 2:
		no_of_params = byte;
		rx_idx++;
		break;
	case 3:
		rx_buffer[byte_idx] = byte;
		if(byte_idx > no_of_params * 2) {
			byte_idx = 0;
			rx_idx++;
		}
		else byte_idx++;
		break;
	case 5:
		crc_low = byte;
		rx_idx++;
		break;
	case 6:
		crc_high = byte;
		rx_idx++;
		/* calculate */
		uint16_t crc_calc = \
				_crc16(rx_buffer, (2 + 1 + (no_of_params * 2)));
		/* if crc matches */
		if(crc_calc == (((crc_high << 8) & 0xFF00) & crc_low)) {
			rx_cplt = 1;
			rx_fail = 0;
			/* process data and load into param_arr */
			for(size_t i = 0; i < no_of_params; i++) {
				uint16_t val16 = ((rx_buffer[i*2 + HEADER_NO_BYTES] << 8) & 0xFF00) \
						& rx_buffer[i*2 + HEADER_NO_BYTES + 1];
				for(size_t j = 0; j < SCALE_PARAM_COUNT; j++) {
					if(i == scale_id_arr[j]) {
						param_arr[i] = (float)val16 * SCALE_DWN_FACTOR;
					}
					else param_arr[i] = (float)val16;
				}
			}
		}
		else {
			rx_idx = 0;
			rx_fail = 1;
			rx_cplt = 0;
		}
		_clearbuffer(rx_buffer);
		break;
	default:
		rx_idx = 0;
	}
}


/**
 * @brief	timeout--to be included in the milliseconds timer callback
 * @param
 */
void uart_timeout() {
	if(uart_frame_timer > TIMEOUT) {
		rx_fail = 1;
		rx_cplt = 0;
		_clearbuffer(rx_buffer);
	}
}

float uart_frame_getparam(uint8_t id) {
	return param_arr[id];
}

void _clearbuffer(uint8_t *buff) {
	for(size_t i = 0; i < RX_BUFF_LEN; i++) {
		buff[i] = 0;
		buff++;
	}
}

uint16_t _crc16(const uint8_t *data, uint8_t length) {
	// See 6.2.2 in https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
	// for this algorithm
	uint16_t _crc = 0xFFFF;
	for(uint8_t i = 0; i < length; i++)
	{
		_crc = _crc ^ (uint16_t)data[i];
		for(uint8_t j = 0; j < 8; ++j)
		{
			if (_crc & 0x0001)
			_crc = (_crc >> 1) ^ 0xA001;
			else
			_crc >>= 1;
		}
	}
	return _crc;
}
