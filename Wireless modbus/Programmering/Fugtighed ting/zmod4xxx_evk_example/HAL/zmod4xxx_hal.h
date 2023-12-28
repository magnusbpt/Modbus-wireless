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
 * @file    zmod4xxx_hal.h
 * @brief   ZMOD4xxx specific hardware abstraction layer definitions
 * @version 2.6.0
 * @author Renesas Electronics Corporation
 */

#ifndef _ZMOD4XXX_HAL_H_
#define _ZMOD4XXX_HAL_H_

#include "zmod4xxx_types.h"

/** 
 * Init hardware and assign hardware specific functions to ZMOD4xxx object
 *
 * If example code is ported to the customer platform, this function must be
 *  re-implemented. The function must assign the zmod4xxx_dev_t#read, 
 *  zmod4xxx_dev_t#write and zmod4xxx_dev_t#delay_ms members of \a dev.
 *
 * \param    [in] dev   pointer to the sensor object
 * \return   error code
 * \retval   0 on success
 * \retval   !=0 hardware specific error code
 */
int  init_hardware ( zmod4xxx_dev_t*  dev );

/** 
 * Free up resources allocated by init_hardware
 *
 *  \return error code
 *  \retval 0 on success
 *  \retval !=0 hardware specific error code
 */
int  deinit_hardware ( );


#endif // _ZMOD4XXX_HAL_H_
