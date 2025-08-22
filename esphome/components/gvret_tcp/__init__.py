import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.const import CONF_ID, CONF_PORT
from esphome.components import canbus as canbus_comp

DEPENDENCIES = ["canbus"]

CONF_BUS_INDEX = "bus_index"
CONF_ON_TRANSMIT = "on_transmit"

gvret_ns = cg.esphome_ns.namespace("gvret_tcp")
GvretTcpServer = gvret_ns.class_("GvretTcpServer", cg.Component)
CanFrame = canbus_comp.canbus_ns.class_("CanFrame")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GvretTcpServer),
        cv.Optional(CONF_PORT, default=23): cv.port,
        cv.Optional(CONF_BUS_INDEX, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_ON_TRANSMIT): automation.validate_automation(single=False),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_bus_index(config[CONF_BUS_INDEX]))

    if CONF_ON_TRANSMIT in config:
        for conf in config[CONF_ON_TRANSMIT]:
            trigger = var.get_on_transmit_trigger()
            await automation.build_automation(trigger, [(CanFrame, "x")], conf)
