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
 * @file    escom.h
 * @brief   ESCom board type and function declarations
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */

#ifndef _ESCOM__H_
#define _ESCOM__H_

#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "libusb.h"

/** ESCom error sub-scopes (or're with HAL scope and acutal error code) */
typedef enum {
  eesLibUSB    = 0x110000,
  eesI2C       = 0x120000,
  eesCommand   = 0x140000
} ESComErrorScope_t;


/** Data structure containing ESCom board information */
typedef struct {
  libusb_device*         device;
  libusb_device_handle*  handle;
  uint8_t                outEP;
  uint8_t                inEP;
  char                   serial [ 66 ];
} ESComInterface_t;

typedef enum { emUSB, emEVK }  ESComMode_t;

/** Enumerate ESCom boards connected to PC
 *
 * Scan available USB interfaces and save those with matching VID/PID in the
 *  `boards` buffer. Value pointed to by `count` serves as input and output.
 *  As input, `count` specifies the size maximum number of boards to be
 *  stored in `boards`. The buffer must provide sufficent space.
 * Before return, this function stores the number of detected EScom boards in
 *  \a count.
 *
 * @param  [in]     boards Pointer to ::ESComInterface_t objects
 * @param  [in,out] count  Maximum number of boards to be stored in `board` [in],
 *                         acutal number of boards stored in `board` [out]
 * @return  0 on success or error code on failure
 */
int  ESCom_Find ( ESComInterface_t*  boards, int*  count );

/** Connect ESCom board
 *
 * This function tries to connect the ESCom board identified by `board` and
 *  allocates the required resources.
 *
 * @param [in] board Pointer to an ::EScomInterface_t instance obtained through
 *                   ESCom_Find()
 * @return   0 on success or error code on failure
 */
int  ESCom_Connect(ESComInterface_t*  board);

/** Disconnect ESCom board and free assciated resources
 *
 * @param [in] board Pointer to an ::EScomInterface_t instance
 * @return   0 on success or error code on failure
 */
int  ESCom_Disconnect(ESComInterface_t*  board);

/** Switch on or off the internal sensor supply voltage
 *
 * @param [in] board Pointer to an ::ESComInsteface_t instance obtained
 * @param [in] on    Boolean indicating whether the sensor supply voltage shall be swtiched on
 * @return   0 on success or error code otherwise
 */
int  ESCom_SetPower(ESComInterface_t*  board, bool  on);

/** Perform an I2C Write
 *
 * This is the ESCom implementation of HAL_t::i2cWiote
 * @param [in] board  Pointer to an ::ESComInsterface_t instance
 * @param [in] slAddr Slave address of target device
 * @param [in] data   Pointer to buffer containing data to be sent
 * @param [in] size   Number of byte to be sent
 * @return   0 on success or error code otherwise
 */
int  ESCom_I2CWrite(ESComInterface_t*  board, uint8_t  slAddr, uint8_t*  data, int  size);

/** Perform an I2C Read
 *
 * This is the ESCom implementation of HAL_t::i2cRead
 * @param [in]  board  Pointer to an ::ESComInsterface_t instance
 * @param [in]  slAddr Slave address of target device
 * @param [out] data   Pointer to buffer where received data is written
 * @param [in]  size   Number of byte to be received
 * @return   0 on success or error code otherwise
 */
int  ESCom_I2CRead(ESComInterface_t*  board, uint8_t  slAddr, uint8_t*  data, int  size );

/** Perform an I2C Write, followed by Read (no stop condition in between)
 *
 * This is the ESCom implementation of HAL_t::i2cWriteRead
 * @param [in]  board    Pointer to an ::ESComInsterface_t instance
 * @param [in]  slAddr   Slave address of target device
 * @param [in]  wrData   Pointer to buffer containing data to be sent
 * @param [in]  wrSize   Number of byte to be sent
 * @param [out] rdData   Pointer to buffer where received data is written
 * @param [in]  rdSize   Number of byte to be received
 * @return   0 on success or error code otherwise
 */
int  ESCom_I2CWriteRead(ESComInterface_t*  board, uint8_t  slAddr, uint8_t*  wrData, int  wrSize, uint8_t*  rdData, int  rdSize );

/** Read out the measured sensor voltage 
 *  @arg [in]  board Pointer to an ::ESComInsteface_t instance
 *  @arg [out] voltage voltage value measured by ESCom board
 */
int  ESCom_GetSensorVoltage(ESComInterface_t*  board, float*  voltage );


#endif // _ESCOM__H_
