/******************************************************************************
  Filename:       softI2c.c
  Revised:        $Date: 2018-05-21 08:38:22  $
  Revision:       $Revision: 00001 $

  Description:    
******************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>

#include <libsoc_i2c.h>
#include <libsoc_gpio.h>     /* GPIO library */
#include <libsoc_debug.h>

#include "include/softI2c.h"

/*********************************************************************
 * @fn         i2cInit
 *
 * @brief      Config I2C clock,pins,pins input and output,io clock 
 *            
 *       
 * @param     none 
 *
 * @return     none
 */
soft_i2c_t * softI2cInit(unsigned int gpio_scl, unsigned int gpio_sda)
{	
  // Create both gpio pointers
  // gpio *gpio_i2c_sda, *gpio_i2c_scl;
  soft_i2c_t * soft_i2c = malloc(sizeof(soft_i2c_t));
  soft_i2c->gpio_scl_pin = gpio_scl;
  soft_i2c->gpio_sda_pin = gpio_sda;

  // Enable debug output
  // libsoc_set_debug(1);

  // Request gpios
  soft_i2c->gpio_i2c_scl = libsoc_gpio_request(gpio_scl, LS_SHARED);
  soft_i2c->gpio_i2c_sda = libsoc_gpio_request(gpio_sda, LS_SHARED);

  // Ensure both gpio were successfully requested
  if (soft_i2c->gpio_i2c_scl == NULL || soft_i2c->gpio_i2c_sda == NULL)
  {
    goto fail;
  }
  // Set direction to OUTPUT
  libsoc_gpio_set_direction(soft_i2c->gpio_i2c_scl, OUTPUT);
  
  // Check the direction
  if (libsoc_gpio_get_direction(soft_i2c->gpio_i2c_scl) != OUTPUT)
  {
    printf("Failed to set direction to OUTPUT\n");
    goto fail;
  }
  // Set direction to OUTPUT
  libsoc_gpio_set_direction(soft_i2c->gpio_i2c_sda, OUTPUT);
  
  // Check the direction
  if (libsoc_gpio_get_direction(soft_i2c->gpio_i2c_sda) != OUTPUT)
  {
    printf("Failed to set direction to OUTPUT\n");
    goto fail;
  }
  // Set level HIGH and check level in software and hardware
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);
  
  if (libsoc_gpio_get_level(soft_i2c->gpio_i2c_scl) != HIGH)
  {
    printf("Failed setting gpio level HIGH\n");
    goto fail;
  }
  // Set level HIGH and check level in software and hardware
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, HIGH);
  
  if (libsoc_gpio_get_level(soft_i2c->gpio_i2c_sda) != HIGH)
  {
    printf("Failed setting gpio level HIGH\n");
    goto fail;
  }

  fail:
  
  // If gpio_request was successful
  //if (soft_i2c->gpio_i2c_scl)
  //{
    // Free gpio request memory
  //  libsoc_gpio_free(soft_i2c->gpio_i2c_scl);
  //}
  
  //if (soft_i2c->gpio_i2c_sda)
  //{
    // Free gpio request memory
  //  libsoc_gpio_free(soft_i2c->gpio_i2c_sda);
  //}
  
  return soft_i2c;
}


/******************************************************************************
* FunctionName   : Start_I2c()
* Description    : I2C start bit
* EntryParameter : none
* ReturnValue    : none
******************************************************************************/
static void i2c_start(soft_i2c_t * soft_i2c)
{
  // Set level HIGH and check level in software and hardware
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);
  // Set level HIGH and check level in software and hardware
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, HIGH);
  // Send the data signal of the start condition
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, HIGH);
  // Start condition establishment time is greater than 4.7us, delay
  usleep(10);

  libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, LOW); // Send start signal
  // Start condition establishment time is greater than 4.7us, delay
  usleep(10);

  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, LOW);    // Clamp the I2C bus and prepare to send or receive data
}

/******************************************************************************
* FunctionName   : Stop_I2c()
* Description    : stop bit
* EntryParameter : none
* ReturnValue    : None
******************************************************************************/
static void i2c_stop(soft_i2c_t * soft_i2c)
{
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, LOW);    // Send end condition data signal
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);    // End condition establishment time is greater than 4μ
  usleep(10);

  libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, HIGH);    // Send I2C bus end signal
  usleep(10);
}

/******************************************************************************
* FunctionName   : I2C_SendACK()
* Description    : Send a response
* EntryParameter : 0：ack,1:noAck
* ReturnValue    : None
******************************************************************************/
static void i2c_send_ack(soft_i2c_t * soft_i2c, uint8_t ack)
{
    if(ack)
		{
			libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, HIGH);
		}
		else
		{
			libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, LOW);
		}
  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);          // Pull up the clock
  usleep(10);

  libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, LOW);    // Send I2C bus end signal
  usleep(10);
}

/******************************************************************************
* FunctionName   : SendByte()
* Description    : send 1 byte
* EntryParameter : send byte
* ReturnValue    : None
******************************************************************************/
static void  i2c_send_byte(soft_i2c_t * soft_i2c, uint8_t byte)
{
    uint8_t BitCnt, time_out = 200;
                                          // The length of data to be
    for(BitCnt = 0; BitCnt < 8; BitCnt++) // transmitted is 8 bits.
    {
        if((byte << BitCnt) & 0x80)         // Judge the send bit
        {
            libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, HIGH);
        }
        else
        {
            libsoc_gpio_set_level(soft_i2c->gpio_i2c_sda, LOW);
        }
                                   // Set the clock line high to inform the
        libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);               // controller to start receiving data bits.
        // Ensure that the clock high period is greater than 4μ
        usleep(10);

        libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, LOW);
        usleep(10);
    }
	
	  libsoc_gpio_set_direction(soft_i2c->gpio_i2c_sda, INPUT);
    usleep(5);
    libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);
    usleep(5);
    while(libsoc_gpio_get_level(soft_i2c->gpio_i2c_sda) == HIGH)
    {
        if(time_out--)
        {
            break;
        }

        usleep(5);
    }

    libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, LOW);
    usleep(5);
	  libsoc_gpio_set_direction(soft_i2c->gpio_i2c_sda, OUTPUT);
}

/******************************************************************************
* FunctionName   : I2C_RcvByte
* Description    : recive 1 byte
* EntryParameter : none
* ReturnValue    : recive byte
******************************************************************************/
static uint8_t i2c_recv_byte(soft_i2c_t * soft_i2c)
{
    uint8_t retc = 0;

    libsoc_gpio_set_direction(soft_i2c->gpio_i2c_sda, INPUT);              // Set the data line as input mode

    for(uint8_t i = 0; i < 8; i++)
    {                      // Set the clock line high
        libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, HIGH);       // to make the data on the data line valid.
        usleep(5);
				
		    retc <<= 1;

        if(libsoc_gpio_get_level(soft_i2c->gpio_i2c_sda) == HIGH)
        {                 // Read the data bit,
            retc |= 0x01; // the received data bit is placed in the retec
        }

        libsoc_gpio_set_level(soft_i2c->gpio_i2c_scl, LOW);       //Set the clock line low, ready to receive data bits
        usleep(10);			//Clock low period is greater than 4.7us
    }

    libsoc_gpio_set_direction(soft_i2c->gpio_i2c_sda, OUTPUT);
    usleep(10);

    return retc;
}

/******************************************************************************
* FunctionName   : i2cSendString
* Description    : string send
* EntryParameter : slaveAddr:slave device I2C addr,string:send string,
*                       len:string lenth
* ReturnValue    : -1:fail,0:success
******************************************************************************/
int8_t softi2cSendString(soft_i2c_t * soft_i2c,uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint16_t dataLen)
{
    if(dataLen == 0 || data == NULL)
    {
        return -1;
    }

    i2c_start(soft_i2c);               	           //Start bus
    i2c_send_byte(soft_i2c, slaveAddr << 1);       //Sending device address
	
	  //i2c_send_byte(soft_i2c, regAddr);

    for(uint8_t cnt = 0; cnt < dataLen; cnt++)
    {
        i2c_send_byte(soft_i2c, data[cnt]);
    }

    i2c_stop(soft_i2c);                  //End bus

    return 0;
}

/******************************************************************************
* FunctionName   : i2cRecvString
* Description    : string recive
* EntryParameter : slave_addr:slave device I2C addr,
*                       string:recive string,len:string lenth
* ReturnValue    : -1:fail,not -1:success
******************************************************************************/
int8_t softi2cRecvString(soft_i2c_t * soft_i2c,uint8_t slaveAddr,uint8_t regAddr,uint8_t *buff,uint16_t buffSize)
{	
    if(buffSize == 0 || buff == NULL)
    {
        return -1;
    }

    i2c_start(soft_i2c);                                 //Start bus
	  i2c_send_byte(soft_i2c, slaveAddr << 1);
	  //i2c_send_byte(soft_i2c, regAddr);
	
	  i2c_start(soft_i2c); 
    i2c_send_byte(soft_i2c, (slaveAddr << 1) + 1);            //Sending device address

    for(uint8_t i = 0; i < buffSize; i++)
    {
        buff[i] = i2c_recv_byte(soft_i2c);                   //Receive data
        if(i == (buffSize - 1))
        {
            i2c_send_ack(soft_i2c, 1);
        }
        else
        {
            i2c_send_ack(soft_i2c, 0);                      //Send the answer
        }
    }

    i2c_stop(soft_i2c);                                  //End bus

    usleep(10);

    return 0;
}
