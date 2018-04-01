/*
 * delay_us.h
 *
 *  Created on: 28 апр. 2016 г.
 *      Author: Чуфырев Андрей
 */

#ifndef UDP_PID__DELAY_US_H_
#define UDP_PID__DELAY_US_H_


#include <stdbool.h>
#include <stdint.h>

#include "driverlib/sysctl.h"

extern void _delay_us(uint32_t us);


#endif /* UDP_PID__DELAY_US_H_ */
