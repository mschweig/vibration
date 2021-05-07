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

#ifndef ALGORITHM_H__
#define ALGORITHM_H__

#include <zephyr.h>
#include "ui.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Initialize buzzer in the user interface module. */
void algorithm_init(struct k_work_q *work_q);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_H__ */
