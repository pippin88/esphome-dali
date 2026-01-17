
#include <esphome.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"
#include "DALI_Lib.h"

using namespace esphome;
using namespace dali;

// Static variables for ISR access
// Using fixed GPIO pins for ESP32-S3: TX=17, RX=14
// Note: These are hardcoded for this implementation to avoid relying on
// GPIOPin internal APIs during ISR. A future enhancement could map YAML-specified
// pins to raw GPIO numbers dynamically while maintaining ISR safety.
static volatile gpio_num_t s_tx_gpio = GPIO_NUM_17;
static volatile gpio_num_t s_rx_gpio = GPIO_NUM_14;
static DaliBusComponent* s_bus_component = nullptr;

// ISR-safe GPIO wrapper functions for DALI library
static bool IRAM_ATTR bus_is_high() {
    if (s_rx_gpio == GPIO_NUM_NC) return true;
    return gpio_get_level(s_rx_gpio) != 0;
}

static void IRAM_ATTR bus_set_high() {
    if (s_tx_gpio == GPIO_NUM_NC) return;
    gpio_set_level(s_tx_gpio, 1);
}

static void IRAM_ATTR bus_set_low() {
    if (s_tx_gpio == GPIO_NUM_NC) return;
    gpio_set_level(s_tx_gpio, 0);
}

// Timer constants (kept for reference but replaced with library implementation)
#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

void DaliBusComponent::setup() {
    // Store reference for ISR access
    s_bus_component = this;
    
    // Get raw GPIO numbers for ISR-safe access
    if (m_txPin) {
        m_tx_gpio_num = (gpio_num_t)m_txPin->get_pin();
        s_tx_gpio = m_tx_gpio_num;
        
        // Configure TX pin as output
        m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
        // Ensure default output state: HIGH = bus released (idle)
        m_txPin->digital_write(HIGH);
        gpio_set_level(m_tx_gpio_num, 1);
    }

    // Configure RX pin with user-specified pull mode
    if (m_rxPin) {
        m_rx_gpio_num = (gpio_num_t)m_rxPin->get_pin();
        s_rx_gpio = m_rx_gpio_num;
        
        gpio::Flags rx_flags = gpio::Flags::FLAG_INPUT;
        switch (m_rx_pull) {
            case RxPullMode::PULLUP_MODE:
                rx_flags = rx_flags | gpio::Flags::FLAG_PULLUP;
                DALI_LOGD("RX pin configured with PULLUP");
                break;
            case RxPullMode::PULLDOWN_MODE:
                rx_flags = rx_flags | gpio::Flags::FLAG_PULLDOWN;
                DALI_LOGD("RX pin configured with PULLDOWN");
                break;
            case RxPullMode::NONE:
                DALI_LOGD("RX pin configured with no pull");
                break;
        }
        m_rxPin->pin_mode(rx_flags);
    }

    // Initialize the DALI library with HAL callbacks
    ::dali.begin(bus_is_high, bus_set_high, bus_set_low);
    ::dali.set_debug(m_debug_rxtx);
    
    // Setup hardware timer at 9600 Hz (104.167 us period)
    setup_timer();

    DALI_LOGI("DALI bus ready (timer-driven, debug_rxtx=%d)", m_debug_rxtx);

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

// Timer ISR that calls the DALI library's timer handler
void IRAM_ATTR DaliBusComponent::timer_isr() {
    if (s_bus_component) {
        ::dali.timer();
    }
}

// Timer configuration constants
#define DALI_TIMER_NUM 0           // Hardware timer to use (0-3 on ESP32)
#define DALI_TIMER_DIVIDER 80      // Divider for 1 MHz tick rate (80 MHz APB / 80 = 1 MHz)
#define TIMER_TICKS_PER_SAMPLE 104 // Ticks for 104.167 microseconds (9600 Hz sampling)

// Setup hardware timer at 9600 Hz (104.167 microseconds period)
void DaliBusComponent::setup_timer() {
    // Timer DALI_TIMER_NUM, divider 80 (1 MHz tick rate on 80 MHz APB clock)
    m_timer = timerBegin(DALI_TIMER_NUM, DALI_TIMER_DIVIDER, true);
    if (!m_timer) {
        DALI_LOGE("Failed to initialize timer!");
        return;
    }
    
    // Attach ISR
    timerAttachInterrupt(m_timer, &DaliBusComponent::timer_isr, true);
    
    // Set alarm to trigger every 104.167 microseconds (9600 Hz)
    // 104.167 us = 104 ticks at 1 MHz
    timerAlarmWrite(m_timer, TIMER_TICKS_PER_SAMPLE, true);
    timerAlarmEnable(m_timer);
    
    DALI_LOGD("Hardware timer configured at 9600 Hz");
}

// Legacy bit-bang functions kept for reference (not used with library)
void DaliBusComponent::writeBit(bool bit) {
    // NOTE: Kept for compatibility but not used with timer-driven library
    bit = !bit;
    if (m_txPin)
        m_txPin->digital_write(bit ? LOW : HIGH);
    delayMicroseconds(HALF_BIT_PERIOD-6);
    if (m_txPin)
        m_txPin->digital_write(bit ? HIGH : LOW);
    delayMicroseconds(HALF_BIT_PERIOD-6);
}

void DaliBusComponent::writeByte(uint8_t b) {
    // NOTE: Kept for compatibility but not used with timer-driven library
    for (int i = 0; i < 8; i++) {
        writeBit(b & 0x80);
        b <<= 1;
    }
}

uint8_t DaliBusComponent::readByte() {
    // NOTE: Kept for compatibility but not used with timer-driven library
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= (m_rxPin ? m_rxPin->digital_read() : 0);
        delayMicroseconds(BIT_PERIOD);
    }
    return byte;
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    bus_set_high();
    delay(1000);
    bus_set_low();
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    // Use DALI library tx_wait function instead of bit-banging
    if (m_debug_rxtx) {
        DALI_LOGD("TX: addr=0x%02x data=0x%02x", address, data);
    }
    
    // Use library's timer-driven transmission
    ::dali.tx_wait(address, data, 50);
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    // Use DALI library rx function to wait for backward frame
    // rx() returns:
    //   0 = no frame available
    //   1 = receiving in progress  
    //   2 = decode error
    //   8 = backward frame (8 bits) received successfully
    //   16 = forward frame (16 bits) - not a backward reply
    
    uint8_t rxBuf[2] = {0, 0};
    unsigned long startTime = millis();
    
    while (millis() - startTime < timeout_ms) {
        uint8_t result = ::dali.rx(rxBuf);
        
        if (result == 8) {
            // Successfully received backward frame (8-bit response)
            if (m_debug_rxtx) {
                DALI_LOGD("RX backward: 0x%02x", rxBuf[0]);
            }
            return rxBuf[0];
        } else if (result == 16) {
            // Received forward frame (16-bit command) - not a backward reply
            if (m_debug_rxtx) {
                DALI_LOGD("RX forward frame (not a reply): 0x%02x 0x%02x", rxBuf[0], rxBuf[1]);
            }
            // Continue waiting for actual backward frame
        } else if (result == 2) {
            // Decode error
            if (m_debug_rxtx) {
                DALI_LOGW("RX decode error");
            }
            // Continue waiting
        }
        // For result == 0 (no frame) or 1 (receiving), just continue polling
        
        delay(1);
    }
    
    // Timeout - no backward frame received
    return 0;
}
