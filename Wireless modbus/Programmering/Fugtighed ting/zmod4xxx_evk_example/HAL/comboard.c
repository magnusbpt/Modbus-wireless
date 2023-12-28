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
 * @file    comboard.c
 * @brief   HAL implementation of ESCom/HiCom based code
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */


#include <stdio.h>
#include <signal.h>
#include <windows.h>

#include "hal.h"

#include "escom.h"
#include "hicom.h"



#define  ESCOM_MAX_COUNT  3
#define  HICOM_MAX_COUNT  3

typedef void*  BoardHandle_t;


static ESComInterface_t   _esComList [ ESCOM_MAX_COUNT ];
static ESComInterface_t*  _esComHandle;

static HiComInterface_t   _hiComList [ HICOM_MAX_COUNT ];
static HiComInterface_t*  _hiComHandle;

static BoardHandle_t      _boardHandle;

static int ( *_deinitFunction ) ( HAL_t* );

// remember hal object for deinitialization
static HAL_t*   _hal = NULL;


// wrap the ESCom I2C Read API call in slim HAL API compatible function
static int
_ESComI2CReadWrapper ( uint8_t  slAddr, uint8_t*  data, int  len ) {
  return  ESCom_I2CRead ( _esComHandle, slAddr, data, len );
}

// wrap the ESCom I2C Write API call in slim HAL API compatible function
static int
_ESComI2CWriteWrapper ( uint8_t  slAddr, uint8_t*  data, int  len ) {
  return  ESCom_I2CWrite ( _esComHandle, slAddr, data, len );
}

// wrap the ESCom I2C Write + Read API call in slim HAL API compatible function
static int
_ESComI2CWriteReadWrapper ( uint8_t  slAddr, uint8_t*  wrData, int  wrLen, uint8_t*  rdData, int  rdLen ) {
  return  ESCom_I2CWriteRead ( _esComHandle, slAddr, wrData, wrLen, rdData, rdLen );
}

int
_ESComInit ( HAL_t*  hal, ESComInterface_t**  board ) {
  int  count = ESCOM_MAX_COUNT;

  // scan for ESCom boards connected to the host
  //  up to ESCOM_MAX_COUNT devices are detected 
  int  errorCode = ESCom_Find ( _esComList, &count );
  if ( errorCode )
    return errorCode;

  // iterate over list of ESCom boards and try to connect
  //  the first one that succeeds will be used
  for ( int i = 0; i < count; ++ i ) {
    errorCode = ESCom_Connect ( &_esComList [ i ] );
    if ( errorCode )
      // try next board if we cannot connect to this one
      continue;

    _esComHandle = &_esComList [ i ];
    hal -> i2cRead      = _ESComI2CReadWrapper;
    hal -> i2cWrite     = _ESComI2CWriteWrapper;
    hal -> i2cWriteRead = _ESComI2CWriteReadWrapper;

    *board = & _esComList [ i ];
    return ecSuccess;
  }

  return ieNoInterfaceFound;
}


static int
_ESComDeinit ( ) {
  int errorCode = ESCom_SetPower ( _esComHandle, false );
  // report error, but don't return. We should try to disconnect
  //  regardless of the error
  if ( errorCode )
    printf ( "ERROR code x%02x received during ESCom power down.\n", 
             errorCode );

  return ESCom_Disconnect ( _esComHandle );
}


// wrap the HiCom I2C Read API call in slim HAL API compatible function
static int
_HiComI2CReadWrapper ( uint8_t  slAddr, uint8_t*  data, int  len ) {
  return  HiCom_I2CRead ( _hiComHandle, slAddr, data, len );
}

// wrap the HiCom I2C Write API call in slim HAL API compatible function
static int
_HiComI2CWriteWrapper ( uint8_t  slAddr, uint8_t*  data, int  len ) {
  return  HiCom_I2CWrite ( _hiComHandle, slAddr, data, len );
}

// wrap the HiCom I2C Write + Read API call in slim HAL API compatible function
static int
_HiComI2CWriteReadWrapper ( uint8_t  slAddr, uint8_t*  wrData, int  wrLen, uint8_t*  rdData, int  rdLen ) {
  return  HiCom_I2CWriteRead ( _hiComHandle, slAddr, wrData, wrLen, rdData, rdLen );
}


static int
_HiComInit ( HAL_t*  hal, HiComInterface_t**  board ) {
  int  count = ESCOM_MAX_COUNT;

  // scan for ESCom boards connected to the host
  //  up to ESCOM_MAX_COUNT devices are detected 
  int  errorCode = HiCom_Find ( _hiComList, &count );
  if ( errorCode )
    return errorCode;

  // iterate over list of ESCom boards and try to connect
  //  the first one that succeeds will be used
  for ( int i = 0; i < count; ++ i ) {
    errorCode = HiCom_Connect ( &_hiComList [ i ] );
    if ( errorCode )
      // try next board if we cannot connect to this one
      continue;

    _hiComHandle = &_hiComList [ i ];
    hal -> i2cRead      = _HiComI2CReadWrapper;
    hal -> i2cWrite     = _HiComI2CWriteWrapper;
    hal -> i2cWriteRead = _HiComI2CWriteReadWrapper;

    *board = & _hiComList [ i ];
    return ecSuccess;
  }

  return ieNoInterfaceFound;
}

static int
_HiComDeinit ( ) {
  int errorCode = HiCom_SetPower ( _hiComHandle, false );
  // report error, but don't return. We should try to disconnect
  //  regardless of the error
  if ( errorCode )
    printf ( "ERROR code x%02x received during HiCom power down.\n", 
             errorCode );

  return HiCom_Disconnect ( _hiComHandle );
}


void
_Terminate ( int  sig ) {
  printf ( "Termination requested by user\n" );
  HAL_HandleError ( ecSuccess, NULL );
}

void
_Sleep ( uint32_t  ms ) {
  Sleep ( ms );
}

int
HAL_Init ( HAL_t*  hal ) {
  int errorCode;

  _hal = hal;
  printf ( "This application can be be terminated at any time by Ctrl-C\n\n" );
  printf ( "Looking for ESCom Boards ...\n" );

  // register signal handler for Ctlr-C
  signal ( SIGINT, _Terminate );

  hal -> msSleep = _Sleep;
 
  // Try to connect an ESCom board
  errorCode = _ESComInit ( hal, ( ESComInterface_t** ) &_boardHandle );
  if ( !errorCode ) {
    printf ( "Detected ESCom board\n" );
    printf ( "  USB Serial: %ls\n",  ( ( ESComInterface_t* ) _boardHandle ) -> serial + 2 );
 
    // board is connected - set deinit function 
    _deinitFunction = _ESComDeinit;

    // swtich on sensor voltage
    errorCode = ESCom_SetPower ( _boardHandle, true );
    if ( errorCode ) return errorCode;

    // allow setteling of sensor supply voltage
    hal -> msSleep ( 100 );
    
    // verify the sensor is powered - if not, notify user and
    //  exit demo. Typically this is due to missing jumper
    float  voltage;
    errorCode = ESCom_GetSensorVoltage ( _boardHandle, &voltage );
    if ( errorCode ) return errorCode;
    if ( voltage < 1.8 ) {
      printf ( "  ERROR: Bad sensor supply voltage. Please check that the supply voltage jumpers are set correctly.\n\n" );
      return -1;
    }
    else
      printf ( "  Sensor voltage: %.1f\n\n", voltage );

    // we're connected and powered - return successfully
    return ecSuccess;
  }

  // Try to connect an HiCom board
  printf ( "Looking for HiCom Boards\n" );
  errorCode = _HiComInit ( hal, ( HiComInterface_t** ) &_boardHandle );
  if ( !errorCode ) {
    printf ( "Detected HiCom board\n" );
 
    // board is connected - set deinit function 
    _deinitFunction = _HiComDeinit;

    // swtich on sensor voltage
    errorCode = HiCom_SetPower ( _boardHandle, true );
    if ( errorCode ) return errorCode;

    hal -> msSleep ( 100 );

    // we're connected and powered - return successfully
    return ecSuccess;
  }

  return errorCode;
}

int
HAL_Deinit ( HAL_t*  hal ) {
  memset ( hal, 0, sizeof ( hal ) );
  return _deinitFunction ( hal );
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
