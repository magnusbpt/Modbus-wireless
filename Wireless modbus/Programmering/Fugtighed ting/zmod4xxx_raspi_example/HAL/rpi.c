/*******************************************************************************
 * Copyright (c) 2023 Renesas Electronics Corporation
 * All Rights Reserved.
 *
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.renesas.com/eu/en/document/msc/renesas-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    rpi.c
 * @brief   Raspberry Pi HAL implementation
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */


#include <stdio.h>
#include <string.h>
#include <pigpio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "rpi.h"
#include "hal.h"


#define I2C_BUS 1

static  int     _rpi;
static uint8_t  _i2cCmdBuf [ 1024 ];

// remember hal object for deinitialization
static HAL_t*   _hal = NULL;

static int
_PiGPIOError ( int  error ) {
  int  code = esInterface | resPiGPIO | ( error & 0xffff );
  return code;
}

static int
_I2CError ( int  error ) {
  int  code = esInterface | resI2C | ( error & 0xffff );
  return code;
}


void
_Sleep ( uint32_t  ms ) {
  usleep ( ms * 1000 );
}

static int
_Connect ( ) {
  int errorCode = gpioInitialise ( );
  if ( errorCode < 0 )
    return _PiGPIOError ( errorCode );
  _rpi = i2cOpen ( I2C_BUS, 0, 0 );
  if ( _rpi < 0 )
    return _PiGPIOError ( errorCode );
  
  // initialize reset pin:
  //  Set as input ...
  errorCode = gpioSetMode ( 26, PI_INPUT );
  if ( errorCode ) return _PiGPIOError ( errorCode );

  //  ... with pullup ...
  errorCode = gpioSetPullUpDown ( 26, PI_PUD_UP );
  if ( errorCode ) return _PiGPIOError ( errorCode );

  return ecSuccess;
}

static int
_I2CRead ( uint8_t  slAddr, uint8_t*  rdData, int  rdLen ) {
  uint8_t*  p = _i2cCmdBuf;
  // set address
  *p++ = PI_I2C_ADDR;
  *p++ = slAddr;
  // read command
  *p++ = PI_I2C_READ;
  *p++ = rdLen;
  // no more commands
  *p++ = 0;
  
  int  count = i2cZip ( _rpi, _i2cCmdBuf, p - _i2cCmdBuf, rdData, rdLen );
  if ( count < 0 )
    return _PiGPIOError ( count );
  else if ( count != rdLen )
    return _I2CError ( recI2CLenMismatch );
  return ecSuccess;
}

static int
_I2CWrite ( uint8_t  slAddr, uint8_t*  wrData, int  wrLen ) {
  uint8_t*  p = _i2cCmdBuf;
  uint8_t  dummy [ 10 ];
  // set address
  *p++ = PI_I2C_ADDR;
  *p++ = slAddr;
  // write command
  *p++ = PI_I2C_WRITE;
  *p++ = wrLen;
  memcpy ( p, wrData, wrLen );
  p += wrLen;
  // no more commands
  *p++ = 0;
  
  int  count = i2cZip ( _rpi, _i2cCmdBuf, p - _i2cCmdBuf, dummy, 0 );
  if ( count < 0 )
    return _PiGPIOError ( count );
  else if ( count )
    return _I2CError ( recI2CLenMismatch );
  return ecSuccess;
}

static int
_I2CWriteRead ( uint8_t  slAddr, uint8_t*  wrData, int  wrLen, uint8_t*  rdData, int  rdLen ) {
  uint8_t*  p = _i2cCmdBuf;
  // set address
  *p++ = PI_I2C_ADDR;
  *p++ = slAddr;
  // write command
  *p++ = PI_I2C_WRITE;
  *p++ = wrLen;
  memcpy ( p, wrData, wrLen );
  p += wrLen;
  // read command
  *p++ = PI_I2C_READ;
  *p++ = rdLen;
  // no more commands
  *p++ = 0;
  
  // for some unknown reason the last parameter must be 1 byte more than the actual read count
  int  count = i2cZip ( _rpi, _i2cCmdBuf, p - _i2cCmdBuf, rdData, rdLen + 1 );
  if ( count < 0 )
    return _PiGPIOError ( count );
  else if ( count != rdLen )
    return _I2CError ( recI2CLenMismatch );
  return ecSuccess;
}

static int
_Reset ( ) {

  //  ... Define output value:
  //  for a sensor reset, the pin is temporarily configured as output
  int errorCode = gpioWrite ( 26, 0 );
  if ( errorCode ) return _PiGPIOError ( errorCode );

  // release the reset pin
  _Sleep ( 1 );
  errorCode = gpioSetMode ( 26, PI_INPUT );
  if ( errorCode ) return _PiGPIOError ( errorCode );

  return ecSuccess;  
}

void
_Terminate ( int  sig ) {
  printf ( "Termination requested by user\n" );
  HAL_HandleError ( ecSuccess, NULL );
}



int
HAL_Init ( HAL_t*  hal ) {

  printf ( "Initializing Raspbery Pi HAL\n\n" );
  printf ( "This application can be be terminated at any "
           "time by pressing Ctrl-C\n\n" );

  _hal = hal;

  int errorCode = _Connect ( );

  // register signal handler for Ctlr-C
  // this needs to be called after gpioInitialize (called from _Connect)
  signal ( SIGINT, _Terminate );

  if ( ! errorCode ) {
    hal -> msSleep        = _Sleep;
    hal -> i2cRead        = _I2CRead;
    hal -> i2cWrite       = _I2CWrite;
    hal -> i2cWriteRead   = _I2CWriteRead;
    hal -> reset          = _Reset;
  }
  return errorCode;
}


int
HAL_Deinit ( HAL_t*  hal ) {
  int  errorCode = i2cClose ( _rpi );
  gpioTerminate ( );
  if ( errorCode )
    return _PiGPIOError ( errorCode );
  return ecSuccess;
}


void
HAL_HandleError ( int  errorCode, void const*  contextV ) {
  char const*  context = ( char const* ) contextV;
  if ( errorCode )
    printf ( "ERROR code x%02X received during %s\n", errorCode, context );

  errorCode = HAL_Deinit ( _hal );
  if ( errorCode )
    printf ( "ERROR code x%02X received during interface deinitialization\n", 
             errorCode );

  printf ( "Exiting\n" );
  exit ( errorCode );
}
