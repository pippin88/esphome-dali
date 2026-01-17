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
RxPullMode = dali_ns.enum('RxPullMode')

RX_PULL_MODES = {
    'NONE': RxPullMode.NONE,
    'PULLUP': RxPullMode.PULLUP,
    'PULLDOWN': RxPullMode.PULLDOWN,
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DaliBusComponent),
    cv.Required(CONF_RX_PIN): pins.gpio_input_pin_schema,
    cv.Required(CONF_TX_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_DISCOVERY): cv.All(cv.requires_component("light"), cv.boolean),
    cv.Optional(CONF_INITIALIZE_ADDRESSES): cv.boolean,
    cv.Optional(CONF_DEBUG_RXTX, default=False): cv.boolean,
    cv.Optional(CONF_RX_PULL, default='PULLDOWN'): cv.enum(RX_PULL_MODES, upper=True),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config: OrderedDict):
    var = cg.new_Pvariable(config[CONF_ID])
    bus = await cg.register_component(var, config)

    rx_pin = await cg.gpio_pin_expression(config[CONF_RX_PIN])
    cg.add(var.set_rx_pin(rx_pin))
    
    tx_pin = await cg.gpio_pin_expression(config[CONF_TX_PIN])
    cg.add(var.set_tx_pin(tx_pin))

    # Set debug_rxtx option
    cg.add(var.set_debug_rxtx(config[CONF_DEBUG_RXTX]))
    
    # Set rx_pull option
    cg.add(var.set_rx_pull(config[CONF_RX_PULL]))

    if config.get(CONF_DISCOVERY, False):
        cg.add(var.do_device_discovery())

    if config.get(CONF_INITIALIZE_ADDRESSES, False):
        cg.add(var.do_initialize_addresses())
