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
 * @file    rpi.h
 * @brief   Raspberry Pi declarations
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */


#ifndef _RPI__H_
#define _RPI__H_

typedef enum {
  resPiGPIO         = 0x310000,
  resI2C            = 0x320000,
  recI2CLenMismatch = 0x320001
} RPiErrorDefs_t;


#endif // _RPI__H_
