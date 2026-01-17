from typing import OrderedDict
from esphome import pins
from esphome.const import CONF_ID, CONF_RX_PIN, CONF_TX_PIN, CONF_DISCOVERY
from esphome.core import CORE

import esphome.codegen as cg
import esphome.config_validation as cv

AUTO_LOAD = ["light", "output"]

CONF_DALI_BUS = 'dali_bus'
CONF_INITIALIZE_ADDRESSES = 'initialize_addresses'
CONF_DEBUG_RXTX = 'debug_rxtx'
CONF_RX_PULL = 'rx_pull'

dali_ns = cg.esphome_ns.namespace('dali')
dali_lib_ns = cg.global_ns
DaliBusComponent = dali_ns.class_('DaliBusComponent', cg.Component)

# create enum in the C++ namespace so codegen can reference it
RxPullMode = dali_ns.enum('RxPullMode')
# Note: The C++ enum names are RX_PULL_NONE, RX_PULL_UP, RX_PULL_DOWN

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DaliBusComponent),
    cv.Required(CONF_RX_PIN): pins.gpio_input_pin_schema,
    cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_DISCOVERY): cv.All(cv.requires_component("light"), cv.boolean),
    cv.Optional(CONF_INITIALIZE_ADDRESSES): cv.boolean,
    cv.Optional(CONF_DEBUG_RXTX, default=False): cv.boolean,
    cv.Optional(CONF_RX_PULL, default="pulldown"): cv.one_of("none", "pullup", "pulldown"),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config: OrderedDict):
    var = cg.new_Pvariable(config[CONF_ID])
    bus = await cg.register_component(var, config)

    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))
    
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))

    if config.get(CONF_DISCOVERY, False):
        cg.add(var.do_device_discovery())

        CORE.register_platform_component("light", bus)

    if config.get(CONF_INITIALIZE_ADDRESSES, False):
        cg.add(var.do_initialize_addresses())

    # debug rxtx runtime flag
    if CONF_DEBUG_RXTX in config:
        cg.add(var.set_debug_rxtx(config[CONF_DEBUG_RXTX]))

    # rx_pull mapping -> call the C++ enum values
    rx_pull = config.get(CONF_RX_PULL, "pulldown")
    if rx_pull == "none":
        cg.add(var.set_rx_pull(dali_ns.RxPullMode.RX_PULL_NONE))
    elif rx_pull == "pullup":
        cg.add(var.set_rx_pull(dali_ns.RxPullMode.RX_PULL_UP))
    else:
        cg.add(var.set_rx_pull(dali_ns.RxPullMode.RX_PULL_DOWN))
