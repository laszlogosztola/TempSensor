/*
 * am2302.h
 *
 * Created: 2013.12.17. 5:35:33
 *  Author: Ghost
 */ 


#ifndef AM2302_H_
#define AM2302_H_

#include "config.h"

void power_up_sensor(void);
void power_down_sensor(void);

//Read data - returns 1 if successful read happened
uint8_t read_data(float* temperature, float* humidity);


#endif /* AM2302_H_ */