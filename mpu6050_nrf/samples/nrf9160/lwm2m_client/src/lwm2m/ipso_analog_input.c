/*
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Source material for IPSO Accelerometer object (3353):
 * http://www.openmobilealliance.org/tech/profiles/lwm2m/3353.xml
 */

#define LOG_MODULE_NAME net_ipso_analog_input
#define LOG_LEVEL CONFIG_LWM2M_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <stdint.h>
#include <init.h>

#include "lwm2m_object.h"
#include "lwm2m_engine.h"

/* Server resource IDs */
#define ANALOG_INPUT							5600
#define ANALOG_INPUT_APPLICATION_TYPE_ID		5750
#define ANALOG_INPUT_SENSOR_TYPE_ID 5751

#define IPSO_OBJECT_ANALOG_INPUT_ID				3202

#define ANALOG_INPUT_MAX_ID           			2

/*
 * Calculate resource instances as follows:
 * start with ANALOG_INPUT_MAX_ID
 */
#define RESOURCE_INSTANCE_COUNT	(ANALOG_INPUT_MAX_ID)

/* resource state */
struct ipso_digital_output_data {
	bool state;
	bool poarity;
};

static struct float32_value float_analog_input_data = {0};

static struct lwm2m_engine_obj analog_input_ID;
static struct lwm2m_engine_obj_field fields[] = {
	OBJ_FIELD_DATA(ANALOG_INPUT, R, FLOAT32),
	OBJ_FIELD_DATA(ANALOG_INPUT_APPLICATION_TYPE_ID, RW_OPT, STRING),
        OBJ_FIELD_DATA(ANALOG_INPUT_SENSOR_TYPE_ID, R_OPT, STRING),
};

static struct lwm2m_engine_obj_inst inst;
static struct lwm2m_engine_res res[ANALOG_INPUT_MAX_ID];
static struct lwm2m_engine_res_inst res_inst[RESOURCE_INSTANCE_COUNT];

static struct lwm2m_engine_obj_inst *analog_input_create(uint16_t obj_inst_id)
{
	int i = 0, j = 0;

	/* Check that there is no other instance with this ID */
	if(inst.resource_count) {
		LOG_ERR("Can not create instance - "
				"already existing: %u", obj_inst_id);
		return NULL;
	}

	/* Set default values */
	(void)memset(&float_analog_input_data, 0, sizeof(float_analog_input_data));
	(void)memset(&res, 0, sizeof(res));
	
	init_res_instance(res_inst, ARRAY_SIZE(res_inst));

	/* initialize instance resource data */
	INIT_OBJ_RES_DATA(ANALOG_INPUT, res, i, res_inst, j,
			  &(float_analog_input_data),
			  sizeof(float_analog_input_data));
	INIT_OBJ_RES_OPTDATA(ANALOG_INPUT_APPLICATION_TYPE_ID, res, i,
			     res_inst, j);

	inst.resources = res;
	inst.resource_count = i;

	LOG_DBG("Create IPSO analog input instance new: %d", obj_inst_id);
	
	return &inst;
}

static int ipso_analog_input_init(struct device *dev)
{
	int ret;
	struct lwm2m_engine_obj_inst *obj_inst = NULL;
	
	analog_input_ID.obj_id = IPSO_OBJECT_ANALOG_INPUT_ID;
	analog_input_ID.fields = fields;
	analog_input_ID.field_count = ARRAY_SIZE(fields);
	analog_input_ID.max_instance_count = 1U;
	analog_input_ID.create_cb = analog_input_create;
	lwm2m_register_obj(&analog_input_ID);

	/* auto create the only instance */
	ret = lwm2m_create_obj_inst(IPSO_OBJECT_ANALOG_INPUT_ID, 0, &obj_inst);
	if (ret < 0) {
		LOG_ERR("Create LWM2M Analog IPSO instance 0 error: %d", ret);
	}
	LOG_DBG("Create LWM2M Analog IPSO successfull");
	return 0;
}

SYS_INIT(ipso_analog_input_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);