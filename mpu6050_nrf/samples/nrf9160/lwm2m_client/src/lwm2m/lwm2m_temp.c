/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <drivers/sensor.h>
#include <net/lwm2m.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(app_lwm2m_temp, CONFIG_APP_LOG_LEVEL);

/* use 25.5C if no sensor available */
static struct float32_value temp_float = { 25, 0 };
static int32_t timestamp;

void handle_temp_events(float measurement)
{
    temp_float.val1 = measurement;
	temp_float.val2 = 0;

	LOG_DBG("Setting Temperature to LwM2M engine");

	/* get current time from device */
	int32_t ts;
	lwm2m_engine_get_s32("3/0/13", &ts);
	/* set timestamp */
	lwm2m_engine_set_s32("3303/0/5518", ts);
    
    lwm2m_engine_set_float32("3303/0/5700", &temp_float);
}

int lwm2m_init_temp(void)
{

	lwm2m_engine_create_obj_inst("3303/0");
	//lwm2m_engine_register_read_callback("3303/0/5700", temp_read_cb);
	lwm2m_engine_set_res_data("3303/0/5518",
				  &timestamp, sizeof(timestamp), 0);
	return 0;
}
