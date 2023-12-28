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
 * @file    hicom.h
 * @brief   HiCom board type and function declarations
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */

#ifndef _HICOM__H_
#define _HICOM__H_

#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "libmpsse_i2c.h"


#define HICOM_NAME       "Dual RS232-HS A"
#define HICOM_I2C_SPEED  100000


typedef FT_STATUS HiComStatus_t;
typedef FT_HANDLE HiComHandle_t;

typedef enum {
  hesFTDI    = 0x210000,
  hesI2C     = 0x220000
} HiComErrorScope_t;


typedef struct {
  FT_DEVICE_LIST_INFO_NODE  node;
  int                       index;
} HiComInterface_t;

/**
 * @brief  Enumerate HiCom boards
 * Scans USB ports for connected HiCom boards.
 * 
 * @param board pointer to a buffer storing board information
 * @param count [in] maximum count of boards to be stored
 *              [out] actual number of boards stored
 * @return int  0 on success, error code otherwise
 */
int  HiCom_Find ( HiComInterface_t*  board, int*  count );

/**
 * @brief Connect a HiCom instance
 * Tries to connect to an interface that has been discovered with 
 * HiCom_Find() previously
 * 
 * @param board Pointer to HiCom board discovered by HiCom_Find()
 * @return int  0 on success, error code otherwise
 */
int  HiCom_Connect ( HiComInterface_t* board );

/**
 * @brief Disconnect a HicomBoard
 * 
 * @param board Pointer to HiCom board to be disconnected
 * @return int 
 */
int  HiCom_Disconnect ( HiComInterface_t*  board );

/**
 * @brief Switch sensor power supply on or off
 * 
 * @param board Pointer to HiCom board to be operated
 * @return int  0 on success, error code otherwise
 */
int  HiCom_SetPower ( HiComInterface_t*  board, bool  on );

/** Perform an I2C Write
 *
 * This is the HiCom implementation of HAL_t::i2cWiote
 * @param [in] board  Pointer to an ::HiComInsterface_t instance
 * @param [in] slAddr Slave address of target device
 * @param [in] data   Pointer to buffer containing data to be sent
 * @param [in] size   Number of byte to be sent
 * @return   0 on success or error code otherwise
 */
int  HiCom_I2CWrite ( HiComInterface_t*  board, uint8_t  slAddr, uint8_t*  data, int  size );

/** Perform an I2C Read
 *
 * This is the HiCom implementation of HAL_t::i2cRead
 * @param [in]  board  Pointer to an ::HiComInsterface_t instance
 * @param [in]  slAddr Slave address of target device
 * @param [out] data   Pointer to buffer where received data is written
 * @param [in]  size   Number of byte to be received
 * @return   0 on success or error code otherwise
 */
int  HiCom_I2CRead ( HiComInterface_t*  board, uint8_t  slAddr, uint8_t*  data, int  size );

/** Perform an I2C Write, followed by Read (no stop condition in between)
 *
 * This is the HiCom implementation of HAL_t::i2cWriteRead
 * @param [in]  board    Pointer to an ::HiComInsterface_t instance
 * @param [in]  slAddr   Slave address of target device
 * @param [in]  wrData   Pointer to buffer containing data to be sent
 * @param [in]  wrSize   Number of byte to be sent
 * @param [out] rdData   Pointer to buffer where received data is written
 * @param [in]  rdSize   Number of byte to be received
 * @return   0 on success or error code otherwise
 */
int  HiCom_I2CWriteRead ( HiComInterface_t*  board, uint8_t  slAddr, uint8_t*  wrData, int  wrSize, uint8_t*  rdData, int  rdSize );


#endif // _HICOM__H_
