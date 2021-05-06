/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
/**@file
 *
 * @brief   Buzzer control for the User Interface module. The module uses PWM to
 *	    control the buzzer output frequency.
 */

#ifndef UI_MPU6050_H__
#define UI_MPU6050_H__

#include <zephyr.h>
#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Initialize buzzer in the user interface module. */
void mpu6050_init_ui(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_MPU6050_H__ */
