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

        if (this->m_initialize_addresses != DaliInitMode::DiscoverOnly) {
            if (this->m_initialize_addresses == DaliInitMode::InitializeAll) {
                DALI_LOGI("Randomizing addresses for *all* DALI devices");
                dali.bus_manager.initialize(ASSIGN_ALL);
            }
            else if (this->m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
                // Only randomize devices without an assigned short address
                DALI_LOGI("Randomizing addresses for unassigned DALI devices");
                dali.bus_manager.initialize(ASSIGN_UNINITIALIZED);
            }

            dali.bus_manager.randomize();
            dali.bus_manager.terminate();

            // Seem to need a delay to allow time for devices to randomize...
            delay(50);
        }

        DALI_LOGI("Begin device discovery via short-address scan...");

        // New discovery approach:
        // Some adapters and setups respond better to probing each short address
        // rather than broadcast SEARCHADDR. Iterate all short addresses and query
        // each one directly using QUERY_CONTROL_GEAR_PRESENT (isDevicePresent).
        //
        // This mirrors the behavior of many DALI tools that poll each short address.
        bool duplicate_detected = false;

        for (short_addr_t short_addr = 0; short_addr <= ADDR_SHORT_MAX; ++short_addr) {
            // Lightly yield to the OS / WDT
            delay(1);
            esp_task_wdt_reset();

            // Skip addresses we've statically reserved in config
            if (m_addresses[short_addr] == 0xFFFFFF) {
                DALI_LOGD("Address %.2x is statically registered; skipping probe", short_addr);
                continue;
            }

            // Query the device presence
            bool present = false;
            // Wrap in a try-like guard through timing/short delays to avoid blocking too long
            // (sendQueryCommand has its own timeout)
            present = dali.isDevicePresent(short_addr);

            if (!present) {
                DALI_LOGD("No device at short address %.2x", short_addr);
                continue;
            }

            DALI_LOGI("Found device at short address %.2x", short_addr);

            // If we already know about a device at this short address, warn about duplicates
            if (m_addresses[short_addr]) {
                DALI_LOGW("Duplicate or already-registered short address detected: %.2x", short_addr);
                duplicate_detected = true;
                // We continue to query capabilities even if it's already registered.
            } else {
                // Attempt to query a long address or other identifying information via bus_manager
                uint32_t long_addr = 0;
                // bus_manager.tryGetLongAddress(...) is not available generically, so leave as 0 if unknown.
                // Register the discovered short address and create a dynamic light component.
                m_addresses[short_addr] = 0xFFFFFF; // mark as detected
                create_light_component(short_addr, long_addr);
            }

            // Small delay between probes to ensure devices can respond & bus recovers
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

    const int MAX_STR_LEN = 20;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];
    snprintf(name, MAX_STR_LEN, "DALI Light %d", short_addr);
    snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);
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
    // Make sure you set discovery: true, or specify a light component somewhere in your YAML!
    DALI_LOGE("Cannot add light component - not enabled");
#endif
}

void DaliBusComponent::loop() {

}

void DaliBusComponent::dump_config() {

}

#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

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

// Implement the virtual methods declared in dali.h / esphome_dali.h
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
