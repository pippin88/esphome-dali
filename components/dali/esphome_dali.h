#pragma once

#include <esphome.h>
#include "dali.h"

namespace esphome {
namespace dali {

enum class DaliInitMode {
    DiscoverOnly,
    InitializeUnassigned,
    InitializeAll
};

// RX pull mode: names chosen to avoid collision with SDK macros
enum class RxPullMode {
    NONE,
    PULLUP_MODE,
    PULLDOWN_MODE
};

class DaliBusComponent : public Component, public DaliPort {
public:
    DaliBusComponent()
        : Component { }
        , dali { *this }
    { }

    void setup() override;
    void loop() override;
    void dump_config() override;

    // Generated GPIOPin type setters (wired by __init__.py)
    void set_tx_pin(GPIOPin* tx_pin) { m_txPin = tx_pin; }
    void set_rx_pin(GPIOPin* rx_pin) { m_rxPin = rx_pin; }

    /// Enable device discovery on setup
    void do_device_discovery() { m_discovery = true; }

    /// Initialize addresses mode
    void do_initialize_addresses(DaliInitMode mode = DaliInitMode::InitializeUnassigned) { m_initialize_addresses = mode; }

    // Runtime setters called by codegen (from YAML)
    void set_rx_pull(RxPullMode pull) { m_rx_pull = pull; }
    void set_debug_rxtx(bool v) { m_debug_rxtx = v; }

    float get_setup_priority() const override { return setup_priority::HARDWARE; }

    void register_static_addr(short_addr_t short_addr) {
        if (short_addr <= ADDR_SHORT_MAX) {
            m_addresses[short_addr] = 0xFFFFFF;
        }
    }

    DaliMaster dali;

public: // DaliPort
    void resetBus() override;
    void sendForwardFrame(uint8_t address, uint8_t data) override;
    uint8_t receiveBackwardFrame(unsigned long timeout_ms = 100) override;

private:
    void writeBit(bool bit);
    void writeByte(uint8_t b);
    uint8_t readByte();

    void create_light_component(short_addr_t short_addr, uint32_t long_addr);

    // Generated pin objects
    GPIOPin* m_rxPin{nullptr};
    GPIOPin* m_txPin{nullptr};

    // Flags and storage
    bool m_discovery = false;
    DaliInitMode m_initialize_addresses = DaliInitMode::DiscoverOnly;
    uint32_t m_addresses[ADDR_SHORT_MAX+1] = {0};

    // runtime options
    RxPullMode m_rx_pull = RxPullMode::PULLDOWN_MODE;
    bool m_debug_rxtx = false;

    // hardware timer handle (Arduino wrapper in this environment)
    hw_timer_t* m_timer{nullptr};
};

} // namespace dali
} // namespace esphome
