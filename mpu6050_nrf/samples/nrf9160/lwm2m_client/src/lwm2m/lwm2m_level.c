/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <net/lwm2m.h>
#include <math.h>
#include "ui.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(app_lwm2m_level, CONFIG_APP_LOG_LEVEL);

#define INPUT_NAME "Fill level"

/* Callback function*/
void handle_level_events(float measurement)
{
	struct float32_value level;
	
    memset(&level,0,sizeof(level));

    level.val1 = measurement;
    
    lwm2m_engine_set_float32("3202/0/5600 ",&level);
    
}

int lwm2m_init_level(void)
{	
	/* create resource for status indication - update at every request */
	LOG_DBG("Create Fill Level Measurement LWM2M Resources.");

	/*Applicaton Type*/
	lwm2m_engine_set_res_data("3202/0/5750 ",
				  INPUT_NAME, sizeof(INPUT_NAME),
				  LWM2M_RES_DATA_FLAG_RO);

	return 0;
}