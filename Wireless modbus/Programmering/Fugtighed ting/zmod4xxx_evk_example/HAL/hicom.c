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
 * @file    hicom.c
 * @brief   HiCom interrface function definitions
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */


#include "hicom.h"



static int
_FTDIError ( int  error ) {
  int  code = esInterface | hesFTDI | ( error & 0xffff );
  return code;
}

static int
_I2CError ( int  error ) {
  int  code = esInterface | hesI2C | ( error & 0xffff );
  return code;
}

int
HiCom_Find ( HiComInterface_t*  handle, int*  count ) {
  HiComStatus_t  errorCode;

  DWORD  deviceCount;

  uint32_t  maxCount = *count;
  *count = 0;

  Init_libMPSSE ( );

  errorCode = I2C_GetNumChannels ( &deviceCount );
  if ( errorCode )
    return _FTDIError ( errorCode );

  if ( deviceCount < 1 )
    return ieNoInterfaceFound;

  // search for HiCom device
  for ( int i = 0; i < deviceCount; ++i ) {
    // ask for device name
    errorCode = I2C_GetChannelInfo ( i, (FT_DEVICE_LIST_INFO_NODE*) handle );
    if ( errorCode )
      continue;

    if ( strcmp ( handle -> node . SerialNumber, "A" ) )
      continue;

    handle -> index = i;

    ++handle;
    *count += 1;
    if ( *count >= maxCount )
      break;
  }

  return ecSuccess;
}

int
HiCom_Connect ( HiComInterface_t*  handle ) {
  HiComStatus_t  errorCode;

  errorCode = I2C_OpenChannel ( handle -> index, &handle -> node . ftHandle );
  if ( errorCode )
    return _FTDIError ( errorCode );

  ChannelConfig  conf = { HICOM_I2C_SPEED, 2, 0 };
  //  initialize channel
  errorCode = I2C_InitChannel ( handle -> node . ftHandle, &conf );
  if ( errorCode )
    return _FTDIError ( errorCode );

  return ecSuccess;
}

int
HiCom_Disconnect ( HiComInterface_t*  handle ) {
  HiComStatus_t errorCode;
  errorCode = I2C_CloseChannel ( handle -> node . ftHandle );
  if ( errorCode )
    _FTDIError ( errorCode );

  return ecSuccess;
}

int
HiCom_SetPower ( HiComInterface_t*  handle, bool  on ) {

  return FT_WriteGPIOLow ( handle -> node . ftHandle, 0x80, on ? 0x80 : 0 );
}

int
HiCom_I2CWrite ( HiComInterface_t*  hal, uint8_t  addr, uint8_t*  data, int  size ) {
  DWORD  transferred;
  HiComStatus_t  errorCode = I2C_DeviceWrite ( hal -> node . ftHandle, addr,
                                               size, data, &transferred, 0x1f );
  if ( errorCode ) return _FTDIError ( errorCode );
  return ecSuccess;
}

int
HiCom_I2CRead ( HiComInterface_t*  hal, uint8_t  addr, uint8_t*  data, int  size ) {
  DWORD  transferred;
  HiComStatus_t  errorCode = I2C_DeviceRead ( hal -> node . ftHandle, addr,
                                               size, data, &transferred, 0x1f );
  if ( errorCode ) return _FTDIError ( errorCode );
  return ecSuccess;
}

int
HiCom_I2CWriteRead ( HiComInterface_t*  hal, uint8_t  addr, uint8_t*  wrData, int  wrSize, uint8_t*  rdData, int  rdSize ) {
  DWORD  transferred;
  HiComStatus_t  errorCode = I2C_DeviceWrite ( hal -> node . ftHandle, addr,
                                               wrSize, wrData, &transferred, 0x1d );
  if ( errorCode ) return _FTDIError ( errorCode );

  errorCode = I2C_DeviceRead ( hal -> node . ftHandle, addr,
                               rdSize, rdData, &transferred, 0x1f );
  if ( errorCode ) return _FTDIError ( errorCode );
  return ecSuccess;
}
