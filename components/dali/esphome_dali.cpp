#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

//static const char *const TAG = "dali";
static const bool DEBUG_LOG_RXTX = false; // NOTE: Will probably trigger WDT

using namespace esphome;
using namespace dali;

void DaliBusComponent::setup() {
    // TX as output
    if (m_txPin) {
        m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
        // Ensure default output state (inverted logic)
        m_txPin->digital_write(LOW);
    }

    // Configure RX as plain input (no internal pull). Many DALI adapters
    // already provide a bias; internal pull-ups on some S3 boards can
    // interfere with backward-frame detection.
    if (m_rxPin) {
        m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);
    }

    DALI_LOGI("DALI bus ready");

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        // Can help if devices are not responding to anything
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

            // Probe the short address (QUERY_CONTROL_GEAR_PRESENT). The library
            // now treats presence as the response being exactly 0xFF.
            if (!dali.isDevicePresent(short_addr)) {
                DALI_LOGD("No device at short address %.2x", short_addr);
                continue;
            }

            // Device responded positively
            DALI_LOGI("Found device at short address %.2x", short_addr);

            // If already known, warn of duplicate
            if (m_addresses[short_addr]) {
                DALI_LOGW("Duplicate or already-registered short address detected: %.2x", short_addr);
                duplicate_detected = true;
            } else {
                // Mark discovered (we don't have long address here). Use 0xFFFFFF
                // only as an indicator the address is handled (mirror register_static_addr).
                m_addresses[short_addr] = 0xFFFFFF;

                // Query some info (like min level) for logging and capability decisions.
                int16_t min_level = dali.lamp.getMinLevel(short_addr);
                int16_t max_level = dali.lamp.getMaxLevel(short_addr);
                DALI_LOGI("  short=%.2x min=%d max=%d", short_addr, (int)min_level, (int)max_level);

                // Create dynamic light component. Pass short_addr into the long_addr parameter
                // so the generated object_id is unique even if we don't have a real long address.
                create_light_component(short_addr, (uint32_t)short_addr);
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

// create_light_component: ensure unique object ids when long_addr is zero
void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
#ifdef USE_LIGHT
    DaliLight* dali_light = new DaliLight { this };
    dali_light->set_address(short_addr);

    const int MAX_STR_LEN = 24;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];

    // Name and id include the short address so each dynamic component is unique
    snprintf(name, MAX_STR_LEN, "DALI Light %d", short_addr);
    if (long_addr != 0) {
        snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);
    } else {
        // Fallback: use short address in id
        snprintf(id, MAX_STR_LEN, "dali_light_%.2x", short_addr);
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
