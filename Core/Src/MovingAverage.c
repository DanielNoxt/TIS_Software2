/*
 * Ringbuffer.c
 *
 *  Created on: Jun 27, 2024
 *      Author: D.Claassen
 */


#include <MovingAverage.h>


// Funktion zum Initialisieren des Ringbuffers
void CircularBuf_init(circular_buf_t *cb, float val){
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        cb->buffer[i] = val;
    }
    cb->head = 0;
}

// Funktion zum Hinzufügen eines neuen Wertes zum Ringbuffer
void CircularBuf_addValue(circular_buf_t *cb, float value) {
    cb->buffer[cb->head] = value;
    cb->head = (cb->head + 1) % BUFFER_SIZE;
}

// Funktion zur Berechnung des Durchschnitts des Ringbuffer
float CircularBuf_average(circular_buf_t *cb) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        sum += cb->buffer[i];
    }
    return sum / BUFFER_SIZE;
}
