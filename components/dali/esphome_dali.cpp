
#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

// Force enable DALI RX/TX debug logging for this debug run
#undef DALI_DEBUG_RXTX
#define DALI_DEBUG_RXTX 1
static const bool DEBUG_LOG_RXTX = true;


using namespace esphome;
using namespace dali;

#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

void DaliBusComponent::setup() {
    // TX as output
    if (m_txPin) {
        m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
        // Ensure default output state (inverted logic used here).
        // For the optocoupler style DALI interface we must RELEASE the bus when idle.
        // Releasing = HIGH for these adapters.
        m_txPin->digital_write(HIGH);
    }

    // Configure RX with a pull-down so the input is not left floating.
    // If your DALI adapter already provides a pull/bias, set to FLAG_INPUT instead.
    if (m_rxPin) {
        m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT | gpio::Flags::FLAG_PULLDOWN);
    }

    DALI_LOGI("DALI bus ready");

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
        }

        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGW("No control gear detected on bus!");
        }

        DALI_LOGI("Begin device discovery via short-address scan...");

        bool duplicate_detected = false;

        for (short_addr_t short_addr = 0; short_addr <= ADDR_SHORT_MAX; ++short_addr) {
            // yield and reset WDT
            delay(1);
            esp_task_wdt_reset();

            // Skip addresses statically registered in YAML/config
            if (m_addresses[short_addr] == 0xFFFFFF) {
                DALI_LOGD("Address %.2x is statically registered; skipping probe", short_addr);
                continue;
            }

            // First-level probe: isDevicePresent (requires a specific affirmative reply)
            if (!dali.isDevicePresent(short_addr)) {
                DALI_LOGD("No device at short address %.2x (no affirmative present reply)", short_addr);
                continue;
            }

            // Second-level confirmation: query a capability (min level).
            // A noisy RX that returns all 1s will often return 0xFF for queries;
            // treat values outside the valid 0..254 range as non-present.
            int16_t min_level = dali.lamp.getMinLevel(short_addr);
            if (min_level < 0 || min_level > 254) {
                DALI_LOGW("Probe at short address %.2x returned invalid min_level=%d - ignoring (likely noise)", short_addr, (int)min_level);
                continue;
            }

            // We have a confirmed device
            DALI_LOGI("Found device at short address %.2x (min=%d)", short_addr, (int)min_level);

            // If already known, warn of duplicate
            if (m_addresses[short_addr]) {
                DALI_LOGW("Duplicate or already-registered short address detected: %.2x", short_addr);
                duplicate_detected = true;
            } else {
                // Try to obtain the device random/long address (C4/C3/C2)
                uint32_t long_addr = dali.getRandomAddress(short_addr);

                // Mark discovered; use long_addr when available
                m_addresses[short_addr] = (long_addr != 0 ? long_addr : 0xFFFFFF);

                int16_t max_level = dali.lamp.getMaxLevel(short_addr);
                DALI_LOGI("  short=%.2x long=0x%.6x min=%d max=%d", short_addr, long_addr, (int)min_level, (int)max_level);

                // Create dynamic light component using long_addr if available, else short_addr fallback.
                create_light_component(short_addr, long_addr);
            }

            // Small settle delay
            delay(10);
        }

        if (duplicate_detected) {
            DALI_LOGW("Duplicate short addresses detected on the bus!");
            DALI_LOGW("  Devices may report inconsistent capabilities.");
            DALI_LOGW("  You should fix your address assignments.");
        }

        DALI_LOGI("Device discovery finished.");
    }
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
#ifdef USE_LIGHT
    DaliLight* dali_light = new DaliLight { this };
    dali_light->set_address(short_addr);

    const int MAX_STR_LEN = 32;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];

    // Name and id include the long address when available for uniqueness
    if (long_addr != 0) {
        snprintf(name, MAX_STR_LEN, "DALI Light %.2x", short_addr);
        snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);
    } else {
        snprintf(name, MAX_STR_LEN, "DALI Light %.2x", short_addr);
        snprintf(id, MAX_STR_LEN, "dali_light_sa%.2x", short_addr);
    }
    // NOTE: Not freeing these strings, they will be owned by LightState.

    auto* light_state = new light::LightState { dali_light };
    App.register_light(light_state);
    App.register_component(light_state);
    light_state->set_name(name);
    light_state->set_object_id(id);
    light_state->set_disabled_by_default(false);
    light_state->set_restore_mode(light::LIGHT_RESTORE_DEFAULT_ON);
    light_state->add_effects({});

    DALI_LOGI("Created light component '%s' (%s)", name, id);
#else
    DALI_LOGE("Cannot add light component - not enabled");
#endif
}

void DaliBusComponent::loop() { }

void DaliBusComponent::dump_config() { }

void DaliBusComponent::writeBit(bool bit) {
    // NOTE: output is inverted - HIGH will pull the bus to 0V (logic low)
    bit = !bit;
    if (m_txPin)
        m_txPin->digital_write(bit ? LOW : HIGH);
    delayMicroseconds(HALF_BIT_PERIOD-6);
    if (m_txPin)
        m_txPin->digital_write(bit ? HIGH : LOW);
    delayMicroseconds(HALF_BIT_PERIOD-6);
}

void DaliBusComponent::writeByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        writeBit(b & 0x80);
        b <<= 1;
    }
}

uint8_t DaliBusComponent::readByte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= (m_rxPin ? m_rxPin->digital_read() : 0);
        delayMicroseconds(BIT_PERIOD); // 1/1200 seconds
    }
    return byte;
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    if (m_txPin)
        m_txPin->digital_write(HIGH);
    delay(1000);
    if (m_txPin)
        m_txPin->digital_write(LOW);
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    if (DEBUG_LOG_RXTX) DALI_LOGD("TX: addr=0x%02x data=0x%02x", address, data);

    // START bit
    writeBit(1);

    // address + data
    writeByte(address);
    writeByte(data);

    // Drive line low for a short time, then wait required spacing
    if (m_txPin)
        m_txPin->digital_write(LOW);

    delayMicroseconds(HALF_BIT_PERIOD * 2);
    delayMicroseconds(BIT_PERIOD * 4);
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    unsigned long startTime = millis();

    // Wait for START bit (line goes high for start)
    while (m_rxPin && m_rxPin->digital_read() == LOW) {
        if (millis() - startTime >= timeout_ms) {
            return 0;
        }
        delay(0);
    }

    // Wait to sample first data bit
    delayMicroseconds(BIT_PERIOD);
    delayMicroseconds(QUARTER_BIT_PERIOD);

    uint8_t data = readByte();
    if (DEBUG_LOG_RXTX) DALI_LOGD("RX: 0x%02x", data);

    // Wait for remaining stop bits and minimum spacing before next frame
    delayMicroseconds(BIT_PERIOD * 2);
    delayMicroseconds(BIT_PERIOD * 8);

    return data;
}
