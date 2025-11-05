print("DEBUG: ds2482_custom __init__.py loaded!")

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_ADDRESS, CONF_ID

AUTO_LOAD = ["sensor"]

ds2482_custom_ns = cg.esphome_ns.namespace("ds2482_custom")
DS2482Component = ds2482_custom_ns.class_("DS2482Component", cg.PollingComponent, i2c.I2CDevice)

CONF_SENSORS = "sensors"
CONF_CHANNEL = "channel"

SENSOR_ITEM_SCHEMA = sensor.sensor_schema().extend({
    cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=7),
})

CONFIG_SCHEMA = (
    cv.Schema({
        cv.GenerateID(CONF_ID): cv.declare_id(DS2482Component),
        cv.Optional(CONF_ADDRESS, default=0x18): cv.i2c_address,
        cv.Optional(CONF_SENSORS, default=[]): cv.ensure_list(SENSOR_ITEM_SCHEMA),
    })
    .extend(cv.polling_component_schema("30s"))
    .extend(i2c.i2c_device_schema(0x18))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_address(config[CONF_ADDRESS]))
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for item in config[CONF_SENSORS]:
        sens = await sensor.new_sensor(item)
        cg.add(var.add_sensor(item[CONF_CHANNEL], sens))
