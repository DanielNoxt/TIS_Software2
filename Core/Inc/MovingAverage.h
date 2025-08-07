/*
 * Ringbuffer.h
 *
 *  Created on: Jun 27, 2024
 *      Author: D.Claassen
 */

#ifndef INC_MOVINGAVERAGE_H_
#define INC_MOVINGAVERAGE_H_

#include "main.h"

#define BUFFER_SIZE AUTOZERO_MOVING_AVERAGE_SIZE

//Abgespeckter Ringbuffer zur Mittelwertbildung
typedef struct {
	float buffer[BUFFER_SIZE];
	uint8_t head;
} circular_buf_t;

void CircularBuf_init(circular_buf_t *cb, float val);
void CircularBuf_addValue(circular_buf_t *cb, float value);
float CircularBuf_average(circular_buf_t *cb);

#endif /* INC_MOVINGAVERAGE_H_ */
