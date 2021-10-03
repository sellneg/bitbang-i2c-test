/******************************************************************************
  Filename:       softI2c.h
  Revised:        $Date: 2018-05-21 08:38:22  $
  Revision:       $Revision: 00001 $

  Description:    GPIO port soft i2c
******************************************************************************/
#include <unistd.h>

#include <libsoc_i2c.h>
#include <libsoc_gpio.h>     /* GPIO library */
#include <libsoc_debug.h>

#ifndef SOFTI2C_H
#define SOFTI2C_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned int gpio_scl_pin;
    gpio *gpio_i2c_scl;
    unsigned int gpio_sda_pin;
    gpio *gpio_i2c_sda;
} soft_i2c_t;

soft_i2c_t * softI2cInit(unsigned int gpio_scl, unsigned int gpio_sda);
int8_t softi2cSendString(soft_i2c_t * soft_i2c,uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint16_t dataLen);
int8_t softi2cRecvString(soft_i2c_t * soft_i2c,uint8_t slaveAddr,uint8_t regAddr,uint8_t *buff,uint16_t buffSize);

#ifdef __cplusplus
}
#endif


#endif
