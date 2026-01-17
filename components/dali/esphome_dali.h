#pragma once

#include <esphome.h>
#include "dali.h"
#include "DALI_Lib.h"

namespace esphome {
namespace dali {

enum class DaliInitMode {
    DiscoverOnly,
    InitializeUnassigned,
    InitializeAll
};

enum class RxPullMode {
    NONE,
    PULLUP,
    PULLDOWN
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

    // Use the generated GPIOPin type (unqualified)
    void set_tx_pin(GPIOPin* tx_pin) { m_txPin = tx_pin; }
    void set_rx_pin(GPIOPin* rx_pin) { m_rxPin = rx_pin; }

    // Configuration setters for YAML options
    void set_debug_rxtx(bool debug) { m_debug_rxtx = debug; }
    void set_rx_pull(RxPullMode pull) { m_rx_pull = pull; }

    /// @brief Perform automatic device discovery on setup.
    /// Light components will automatically be created and appear in HomeAssistant
    void do_device_discovery() { m_discovery = true; }

    /// @brief Initialize long and short addresses for devices on the bus.
    /// @param mode 
    //          InitializeUnassigned - only devices that do not yet have an assigned short address
    ///         InitializeAll - all devices on the bus
    /// @note
    void do_initialize_addresses(DaliInitMode mode = DaliInitMode::InitializeUnassigned) { m_initialize_addresses = mode; }

    // NOTE: Must have a higher priority number than the components that depend on this.
    // ie, this must be initialized first.
    float get_setup_priority() const override { return setup_priority::HARDWARE; }

    void register_static_addr(short_addr_t short_addr) {
        if (short_addr < ADDR_SHORT_MAX) {
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
    
    // Timer ISR setup
    void setup_timer();
    static void IRAM_ATTR timer_isr();

    // Generated pin type
    GPIOPin* m_rxPin{nullptr};
    GPIOPin* m_txPin{nullptr};

    // Raw GPIO numbers for ISR-safe access
    gpio_num_t m_tx_gpio_num;
    gpio_num_t m_rx_gpio_num;

    bool m_discovery = false;
    DaliInitMode m_initialize_addresses = DaliInitMode::DiscoverOnly;
    uint32_t m_addresses[ADDR_SHORT_MAX+1] = {0};
    
    // Configuration options
    bool m_debug_rxtx = false;
    RxPullMode m_rx_pull = RxPullMode::PULLDOWN;
    
    // Hardware timer handle
    hw_timer_t* m_timer = nullptr;
};

} // namespace dali
} // namespace esphome
