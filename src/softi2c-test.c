#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "include/softI2c.h"

#define GPIO_SCL  193
#define GPIO_SDA  194

int main(void) {
    uint8_t config[1] = {0xF5};
    uint8_t data[2] = {0};

    soft_i2c_t *soft_i2c = softI2cInit(GPIO_SCL, GPIO_SDA);

	// Send humidity measurement command(0xF5)
    softi2cSendString(soft_i2c,0x40,0,config,1);

	// Read 2 bytes of humidity data
	// humidity msb, humidity lsb
    softi2cRecvString(soft_i2c,0x40,0,data,2);    
    printf("%X%X\n", data[0], data[0]);
	// Convert the data
	float humidity = (((data[0] * 256 + data[1]) * 125.0) / 65536.0) - 6;

	// Output data to screen
	printf("Relative Humidity : %.2f RH \n", humidity);

	// Send temperature measurement command(0xF3)
	config[0] = 0xE0; // 0xF3;
    softi2cSendString(soft_i2c,0x40,0,config,1);
	sleep(1);

	// Read 2 bytes of temperature data
	// temp msb, temp lsb
    softi2cRecvString(soft_i2c,0x40,0,data,2);
    printf("%X%X\n", data[0], data[1]);
	// Convert the data
	float cTemp = (((data[0] * 256 + data[1]) * 175.72) / 65536.0) - 46.85;
	float fTemp = cTemp * 1.8 + 32;

	// Output data to screen
	printf("Temperature in Celsius : %.2f C \n", cTemp);
	printf("Temperature in Fahrenheit : %.2f F \n", fTemp);
}