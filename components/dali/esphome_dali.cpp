#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

// Use the qqqlab/Waveshare DALI driver library (already present in repo)
#include "DALI_Lib.h"
extern Dali dali; // global object defined by DALI_Lib

using namespace esphome;
using namespace dali;

// Default debug macro (can be overridden at compile time)
#ifndef DALI_DEBUG_RXTX
#define DALI_DEBUG_RXTX 0
#endif
static const bool DEFAULT_DEBUG_LOG_RXTX = (DALI_DEBUG_RXTX != 0);

// Use confirmed raw pins for ISR-level gpio access to avoid relying on GPIOPin internals.
// These match the Waveshare example: TX=17, RX=14
static constexpr gpio_num_t RAW_TX_GPIO = (gpio_num_t)17;
static constexpr gpio_num_t RAW_RX_GPIO = (gpio_num_t)14;

// IRAM ISR: call the DALI library timer() at the required sampling frequency
static void IRAM_ATTR dali_timer_isr() {
    ::dali.timer();
}

// ISR-safe wrappers (file scope) — match the types expected by Dali::begin()
static bool IRAM_ATTR dali_bus_is_high_wrapper() {
    return gpio_get_level(RAW_RX_GPIO) ? true : false;
}
static void IRAM_ATTR dali_bus_set_low_wrapper() {
    gpio_set_level(RAW_TX_GPIO, 0);
}
static void IRAM_ATTR dali_bus_set_high_wrapper() {
    gpio_set_level(RAW_TX_GPIO, 1);
}

void DaliBusComponent::setup() {
    const bool DEBUG_LOG_RXTX = this->m_debug_rxtx ? true : DEFAULT_DEBUG_LOG_RXTX;

    // Configure TX pin via generated GPIOPin wrapper
    if (m_txPin) {
        m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
        // Ensure release-high idle (adapter expects HIGH to release)
        m_txPin->digital_write(HIGH);
    }

    // Configure RX pin with selected pull
    if (m_rxPin) {
        switch (m_rx_pull) {
            case RxPullMode::RX_PULL_UP:
                m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT | gpio::Flags::FLAG_PULLUP);
                break;
            case RxPullMode::RX_PULL_DOWN:
                m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT | gpio::Flags::FLAG_PULLDOWN);
                break;
            case RxPullMode::RX_PULL_NONE:
            default:
                m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);
                break;
        }
    }

    DALI_LOGI("DALI bus ready (rx_pull=%d debug=%d)", (int)m_rx_pull, (int)DEBUG_LOG_RXTX);

    // Start sampling timer at ~9600 Hz so the qqqlab driver can oversample Manchester
    // Use Arduino wrapper available in this PlatformIO environment: timerBegin(frequency)
    m_timer = timerBegin(9600);
    if (m_timer) {
        timerAttachInterrupt(m_timer, &dali_timer_isr);
        DALI_LOGD("DALI sampling timer configured (9600 Hz)");
    } else {
        DALI_LOGW("Failed to start DALI sampling timer");
    }

    // Initialize the library with the ISR-safe wrappers
    // Note: Dali::begin expects (bus_is_high, bus_set_high, bus_set_low)
    ::dali.begin(&dali_bus_is_high_wrapper, &dali_bus_set_high_wrapper, &dali_bus_set_low_wrapper);

    // Discovery (optional)
    if (m_discovery) {
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

            // First-level probe: dali.isDevicePresent()
            if (!dali.isDevicePresent(short_addr)) {
                DALI_LOGD("No device at short address %.2x (no affirmative present reply)", short_addr);
                continue;
            }

            // Confirm via min-level query
            int16_t min_level = dali.lamp.getMinLevel(short_addr);
            if (min_level < 0 || min_level > 254) {
                DALI_LOGW("Probe at short address %.2x returned invalid min_level=%d - ignoring (likely noise)", short_addr, (int)min_level);
                continue;
            }

            DALI_LOGI("Found device at short address %.2x (min=%d)", short_addr, (int)min_level);

            if (m_addresses[short_addr]) {
                DALI_LOGW("Duplicate or already-registered short address detected: %.2x", short_addr);
                duplicate_detected = true;
            } else {
                uint32_t long_addr = dali.getRandomAddress(short_addr);
                m_addresses[short_addr] = (long_addr != 0 ? long_addr : 0xFFFFFF);

                int16_t max_level = dali.lamp.getMaxLevel(short_addr);
                DALI_LOGI("  short=%.2x long=0x%.6x min=%d max=%d", short_addr, long_addr, (int)min_level, (int)max_level);

                create_light_component(short_addr, long_addr);
            }

            delay(10);
        }

        if (duplicate_detected) {
            DALI_LOGW("Duplicate short addresses detected on the bus!");
        }

        DALI_LOGI("Device discovery finished.");
    }
}
... (rest unchanged) ...
