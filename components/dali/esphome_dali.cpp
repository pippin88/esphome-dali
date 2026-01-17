
#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"

// Use the qqqlab/Waveshare DALI driver library (already present in repo)
#include "DALI_Lib.h"
extern Dali dali; // global object defined by DALI_Lib

using namespace esphome;

static const bool DEFAULT_DEBUG_LOG_RXTX = false;

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

namespace esphome {
namespace dali {

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
      if (!::dali.isDevicePresent(short_addr)) {
        DALI_LOGD("No device at short address %.2x (no affirmative present reply)", short_addr);
        continue;
      }

      // Confirm via min-level query using library cmd
      int16_t min_level = ::dali.cmd(DALI_QUERY_MIN_LEVEL, short_addr);
      if (min_level < 0 || min_level > 254) {
        DALI_LOGW("Probe at short address %.2x returned invalid min_level=%d - ignoring (likely noise)", short_addr, (int)min_level);
        continue;
      }

      DALI_LOGI("Found device at short address %.2x (min=%d)", short_addr, (int)min_level);

      if (m_addresses[short_addr]) {
        DALI_LOGW("Duplicate or already-registered short address detected: %.2x", short_addr);
        duplicate_detected = true;
      } else {
        // Try to read random/long address bytes (C4/C3/C2)
        int16_t rH = ::dali.cmd(DALI_QUERY_RANDOM_ADDRESS_H, short_addr);
        int16_t rM = ::dali.cmd(DALI_QUERY_RANDOM_ADDRESS_M, short_addr);
        int16_t rL = ::dali.cmd(DALI_QUERY_RANDOM_ADDRESS_L, short_addr);
        uint32_t long_addr = 0;
        if (rH >= 0 && rM >= 0 && rL >= 0) {
          long_addr = ((uint32_t)rH << 16) | ((uint32_t)rM << 8) | (uint32_t)rL;
        }

        m_addresses[short_addr] = (long_addr != 0 ? long_addr : 0xFFFFFF);

        int16_t max_level = ::dali.cmd(DALI_QUERY_MAX_LEVEL, short_addr);
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

void DaliBusComponent::resetBus() {
  DALI_LOGD("Resetting bus");
  // release
  if (m_txPin) m_txPin->digital_write(HIGH);
  delay(1000);
  if (m_txPin) m_txPin->digital_write(LOW);
}

// send a forward frame (address + data) using the qqqlab library API (addr+data)
void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
  if (m_debug_rxtx) {
    DALI_LOGD("TX: addr=0x%02x data=0x%02x (via library tx_wait)", address, data);
  }

  // Use the library's blocking transmit that takes address+data
  // The qqqlab library in this repo provides a tx_wait(addr, data, timeout_ms) variant.
  bool ok = ::dali.tx_wait(address, data, 500);
  if (!ok) {
    DALI_LOGW("dali.tx_wait returned failure for addr=0x%02x data=0x%02x", address, data);
  }
}

// wait for backward frame (reply) using the library's rx() function
uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
  unsigned long start = millis();
  uint8_t rxbuf[4];

  while (millis() - start < timeout_ms) {
    int rv = ::dali.rx(rxbuf);
    switch (rv) {
      case 0:
        // nothing yet
        break;
      case 1:
        // receiving in progress; allow more time
        break;
      case 2:
        // decode error
        DALI_LOGW("dali.rx returned decode error");
        return 0;
      case 8:
        // backward frame (8 bits) decoded
        if (m_debug_rxtx) DALI_LOGD("RX (back): 0x%02x", rxbuf[0]);
        return rxbuf[0];
      case 16:
        // forward frame was observed - ignore for backward read
        if (m_debug_rxtx) DALI_LOGD("RX: forward frame observed while waiting for backward reply");
        break;
      default:
        // other unexpected values - continue
        break;
    }
    delayMicroseconds(100);
  }

  if (m_debug_rxtx) DALI_LOGD("receiveBackwardFrame timeout");
  return 0;
}

void DaliBusComponent::loop() { }

void DaliBusComponent::dump_config() {
  DALI_LOGI("DALI bus config: rx_pull=%d debug=%d", (int)m_rx_pull, (int)m_debug_rxtx);
}

/* Keep the existing helpers in case other code paths rely on them (bit-bang fallback).
   They are not used for backward-frame decoding anymore. */

void DaliBusComponent::writeBit(bool bit) {
  if (m_txPin)
    m_txPin->digital_write(bit ? LOW : HIGH);
  delayMicroseconds(416);
  if (m_txPin)
    m_txPin->digital_write(bit ? HIGH : LOW);
  delayMicroseconds(416);
}

void DaliBusComponent::writeByte(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    writeBit(b & 0x80);
    b <<= 1;
  }
}

uint8_t DaliBusComponent::readByte() {
  // Deprecated for backward frames - use library's rx()
  uint8_t byte = 0;
  for (int i = 0; i < 8; i++) {
    byte <<= 1;
    byte |= (m_rxPin ? m_rxPin->digital_read() : 0);
    delayMicroseconds(833);
  }
  return byte;
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
#ifdef USE_LIGHT
  DaliLight* dali_light = new DaliLight { this };
  dali_light->set_address(short_addr);

  const int MAX_STR_LEN = 32;
  char* name = new char[MAX_STR_LEN];
  char* id = new char[MAX_STR_LEN];

  if (long_addr != 0) {
    snprintf(name, MAX_STR_LEN, "DALI Light %.2x", short_addr);
    snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);
  } else {
    snprintf(name, MAX_STR_LEN, "DALI Light %.2x", short_addr);
    snprintf(id, MAX_STR_LEN, "dali_light_sa%.2x", short_addr);
  }

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

}  // namespace dali
}  // namespace esphome
