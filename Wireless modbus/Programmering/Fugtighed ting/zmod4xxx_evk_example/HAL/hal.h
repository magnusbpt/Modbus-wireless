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
 * @file    hal.h
 * @brief   Generic hardware abstraction layer definitions
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 * 
 * This header defines the HAL interface. Functions declared here have to be
 *  re-implemented when code is ported to a new platform.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>


/**
 * @brief Success status code and error scopes
 * 
 */
typedef enum {
  ecSuccess   = 0,            /**< common success code */

  esSensor    = 0x00000000,   /**< Sensor scope */
  esAlgorithm = 0x10000000,   /**< Algorithm scope */
  esInterface = 0x20000000,   /**< Interface scope */
  esHAL       = 0x30000000,   /**< HAL scope */
  esMask      = 0xf0000000    /**< provided for scope filtering */
} ErrorCommon_t;

/**
 * @brief HAL scope error definitions
 * 
 * When sensors are initialized (e.g. init_hardware()), the hal objects is
 *  checked whether all HAL functions required by the sensor are provided.
 *  If a function is missing one of the errors from this enumeration is
 *  returned.
 */
typedef enum {
  heI2CReadRequried = esHAL | 1,  /**< HAL_t::i2cRead not provided */
  heI2CWriteRequried,             /**< HAL_t::i2cWrite not provided */
  heI2CWriteReadRequried,         /**< HAL_t::i2cWriteRead not provided */
  heSleepRequried,                /**< HAL_t::sleepMs not provided */
  heResetRequried                 /**< HAL_t::reset not provided */
} HALError_t;

/**
 * @brief Interface scope error definitions
 */
typedef enum {
  ieNoInterfaceFound = esInterface + 1
} InterfaceError_t;

/**
 * @brief A structure of pointers to hardware specific functions
 */
typedef struct {
  /** Pointer to I2C read implementation 
   * 
   * An implementation must
   *  - Send start bit
   *  - Send (`slAddr` << 1) | 1
   *  - Read `rdSize` bytes into `rdData`
   *  - Send stop bit
   */
  int  ( *i2cRead ) ( uint8_t  slAddr, uint8_t*  rdData, int  rdSize );
  
  /** Pointer to I2C write implementation 
   * 
   * An implementation must
   *  - Send start bit
   *  - Send (`slAddr` << 1)
   *  - Send `wrSize` bytes (from `wrData`)
   *  - Send stop bit
   */
  int  ( *i2cWrite ) ( uint8_t  slAddr, uint8_t*  wrData, int  wrSize );
  
  /** Pointer to I2C write & read implementation 
   * 
   * An implementation must
   *  - Send start bit
   *  - Send (`slAddr` << 1)
   *  - Send `wrSize` bytes (from `wrData`)
   *  - Send start bit (repeated start)
   *  - Send (`slAddr` << 1) | 1
   *  - Read `rdSize` bytes into `rdData`
   *  - Send stop bit
   */
  int  ( *i2cWriteRead ) ( uint8_t  slAddr, uint8_t*  wrData, int  wrSize, uint8_t*  rdData, int  rdSize );   

  /** Pointer to delay function
   * 
   * An implementation must delay execution by the specified number of `ms`
   */
  void ( *msSleep ) ( uint32_t  ms );

  /** Pointer to reset function
   * 
   * Implementation must pulse the reset pin
   */
  int  ( *reset ) ( );
} HAL_t;


/**
 * @brief Initialize hardware and populate ::HAL_t object
 * 
 * Any implementation must initialize those members of the ::HAL_t object that
 *  are required by the sensor being operated with pointers to functions that
 *  implement the behavior as specified in the ::HAL_t member documentation.
 * 
 * @param hal   pointer to ::HAL_t object to be initialized
 * @return      error code
 * @retval  0   on success
 * @retval !=0  in case of error
 */
int  HAL_Init        ( HAL_t*  hal );

/**
 * @brief Cleanup before program exit
 * 
 * This function shall free up resources that have been allocated through
 * HAL_Init().
 * 
 * @param hal   pointer to ::HAL_t object to be deinitialized
 * @return      error code
 * @retval  0   on success
 * @retval !=0  in case of error
 */
int  HAL_Deinit      ( HAL_t*  hal );

/**
 * @brief Example error handler
 * 
 * The implementation of this function defines the behavior of the
 * example code when an error occurs during execution.
 * 
 * @param errorCode code of the error to be handled
 * @param context   additional context information
 */
void HAL_HandleError ( int  errorCode, void const*  context );


#endif // PLATFORM_H
