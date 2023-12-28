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
 * @file    escom.c
 * @brief   ESCom board function definitions
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */

#include <stdio.h>
#include "escom.h"

typedef struct {
  struct {
    uint8_t  channel : 3;
    uint8_t  delay   : 1;
    uint8_t  start   : 1;
    uint8_t  read    : 1;
    uint8_t  stop    : 1;
    uint8_t  lAddr   : 1;
  };
  uint8_t    len;
  uint16_t   addr   : 10;
  uint16_t   status :  6;
  uint8_t    data [];
} I2CCtrl_t;


static libusb_context*  _context;
static uint8_t  i2cBuffer [ 1024 ];


static int
_LibUSBError ( int  error ) {
  int  code = esInterface | eesLibUSB | ( error & 0xffff );
  return code;
}

static int
_I2CError ( int  error ) {
  int  code = esInterface | eesI2C | ( error & 0xffff );
  return code;
}

static int
_ESComError ( int  error ) {
  return esInterface | eesCommand | ( error & 0xffff );
}

int
ESCom_Find ( ESComInterface_t*  handle, int*  count ) {

  libusb_device**  device;
  int  deviceIndex = 0;
  
  int errorCode = libusb_init ( &_context );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  ssize_t cnt = libusb_get_device_list ( _context, &device );
  for ( ssize_t i = 0; i < cnt && deviceIndex < *count; ++ i ) {
    handle -> device = device [ i ];
    struct libusb_device_descriptor  desc;
    
    if ( libusb_get_device_descriptor ( handle -> device, &desc ) )
      continue;  // Continue silently?

    if ( desc . idVendor  != 0x45b ) continue;
    if ( desc . idProduct != 0x268 ) continue;

    handle -> handle = 0;
    handle -> inEP  = 0x83;
    handle -> outEP = 0x03;
    ++deviceIndex;
    ++handle;
  }

  *count = deviceIndex;
  return  ecSuccess;
}

int
ESCom_Connect ( ESComInterface_t*  handle ) {
  struct libusb_device_descriptor  desc;
  int errorCode = libusb_get_device_descriptor ( handle -> device, &desc );
  if ( errorCode )
    return errorCode;

  errorCode = libusb_open ( handle -> device, & handle -> handle );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  int  cfg = -1;
  errorCode = libusb_get_configuration ( handle -> handle, &cfg );
  if ( errorCode )
    return _LibUSBError ( errorCode );

  if ( cfg != 1 ) {
    errorCode = libusb_set_configuration ( handle -> handle, 1 );
    if ( errorCode )
      return _LibUSBError ( errorCode );
  }

  int  count = libusb_get_string_descriptor ( handle -> handle, desc . iSerialNumber, 0, handle -> serial, 64 );
  if ( count > 0 ) {
    handle -> serial [ count ] = 0;
    handle -> serial [ count + 1 ] = 0;
  }
  else
    return _LibUSBError ( errorCode );

  errorCode = libusb_claim_interface ( handle -> handle, 2 );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  return ecSuccess;
}

int
ESCom_Disconnect ( ESComInterface_t*  handle ) {
  int errorCode = libusb_release_interface ( handle -> handle, 2 );
  libusb_close ( handle -> handle );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  return ecSuccess;
}

int
ESCom_SetPower ( ESComInterface_t*  handle, bool  on ) {
  uint8_t  data [] = { 4, ( uint8_t ) on };

  int transferred;
  int errorCode = libusb_bulk_transfer ( handle -> handle, handle -> outEP, data, 2, &transferred, 100 );
  if ( errorCode )
    return _LibUSBError ( errorCode);
  
  errorCode = libusb_bulk_transfer ( handle -> handle, handle -> inEP, data, 2, &transferred, 500 );
  if ( errorCode )
    return _LibUSBError ( errorCode);
  
  return ecSuccess;
}

int
ESCom_I2CWrite ( ESComInterface_t*  hal, uint8_t  addr, uint8_t*  data, int  size ) {
  i2cBuffer [ 0 ] = 0;

  I2CCtrl_t*  b = ( I2CCtrl_t* ) ( i2cBuffer + 1 );
  b -> channel = 3;
  b -> start = 1;
  b -> delay = 0;
  b -> read  = 0;
  b -> stop  = 1;
  b -> lAddr = 0;
  b -> len   = size & 0xff;
  b -> addr  = addr;
  memcpy ( b -> data, data, size );
  int transferred;
  int errorCode = libusb_bulk_transfer ( hal -> handle, hal -> outEP, i2cBuffer, 1 + sizeof ( I2CCtrl_t ) + size, &transferred, 100 );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  errorCode = libusb_bulk_transfer ( hal -> handle, hal -> inEP, i2cBuffer, 256, &transferred, 500 );
  if ( errorCode )
    return _LibUSBError ( b -> status );
  
  b = ( I2CCtrl_t* ) i2cBuffer;
  if ( b -> status != 0 )
    return _I2CError ( b -> status );
  return ecSuccess;
}

int
ESCom_I2CRead ( ESComInterface_t*  hal, uint8_t  addr, uint8_t*  data, int  size ) {
  i2cBuffer [ 0 ] = 0;

  I2CCtrl_t*  b = ( I2CCtrl_t* ) ( i2cBuffer + 1 );
  b -> channel = 3;
  b -> start = 1;
  b -> delay = 0;
  b -> read  = 1;
  b -> stop  = 1;
  b -> lAddr = 0;
  b -> len   = size & 0xff;
  b -> addr  = addr;
  int transferred;
  int errorCode = libusb_bulk_transfer ( hal -> handle, hal -> outEP, i2cBuffer, 1 + sizeof ( I2CCtrl_t ), &transferred, 100 );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  errorCode = libusb_bulk_transfer ( hal -> handle, hal -> inEP, i2cBuffer, 256, &transferred, 500 );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  b = ( I2CCtrl_t* ) i2cBuffer;
  if ( b -> status != 0 )
    return _I2CError ( b -> status );
  
  memcpy ( data, b -> data, size );
  return ecSuccess;
}

int
ESCom_I2CWriteRead ( ESComInterface_t*  hal, uint8_t  addr, uint8_t*  wrData, int  wrSize, uint8_t*  rdData, int  rdSize ) {
  i2cBuffer [ 0 ] = 0;

  I2CCtrl_t*  b = ( I2CCtrl_t* ) ( i2cBuffer + 1 );
  b -> channel = 3;
  b -> start = 1;
  b -> delay = 0;
  b -> read  = 0;
  b -> stop  = 0;
  b -> lAddr = 0;
  b -> len   = wrSize & 0xff;
  b -> addr  = addr;
  memcpy ( b -> data, wrData, wrSize );

  b = ( I2CCtrl_t* ) ( b -> data + wrSize );
  b -> channel = 3;
  b -> start = 1;
  b -> delay = 0;
  b -> read  = 1;
  b -> stop  = 1;
  b -> lAddr = 0;
  b -> len   = rdSize & 0xff;
  b -> addr  = addr;

  int transferred;
  int errorCode = libusb_bulk_transfer ( hal -> handle, hal -> outEP, i2cBuffer, 1 + 2 * sizeof ( I2CCtrl_t ) + wrSize, &transferred, 100 );
  if ( errorCode )
    return _LibUSBError ( errorCode );
  
  errorCode = libusb_bulk_transfer ( hal -> handle, hal -> inEP, i2cBuffer, 256, &transferred, 500 );
  if ( errorCode )
    return _LibUSBError ( errorCode );

  b = ( I2CCtrl_t* ) i2cBuffer;
  if ( b -> status != 0 )
    return _I2CError ( b -> status );
  
  if ( ( ++b ) -> status != 0 )
    return _I2CError ( b -> status );
  
  memcpy ( rdData, b -> data, rdSize );
  return ecSuccess;
}

char const*
ESCom_GetDescriptionString ( ESComInterface_t*  ifce ) {
  static char  strBuf [ 100 ];
  sprintf ( strBuf, "ESCom Board, Serial='%ls'", ifce -> serial + 2 );
  return strBuf;
}

int
ESCom_GetSensorVoltage ( ESComInterface_t*  board, float*  voltage ) {
  uint8_t  data [ 10 ] = { 3, };

  int transferred;
  int errorCode = libusb_bulk_transfer ( board -> handle, board -> outEP, data, 1, &transferred, 100 );
  if ( errorCode )
    return _LibUSBError ( errorCode);
  
  errorCode = libusb_bulk_transfer ( board -> handle, board -> inEP, data, 5, &transferred, 500 );
  if ( errorCode )
    return _LibUSBError ( errorCode);
  
  if ( data [ 0 ] )
    return _ESComError ( data [ 0 ] );

  *voltage = * ( float* ) ( data + 1 );

  return ecSuccess;
}
