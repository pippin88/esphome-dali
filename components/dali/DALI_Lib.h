#pragma once

#include <stdint.h>

// DALI Library - Timer-driven implementation
// Based on qqqlab/Waveshare DALI library design

#define DALI_RX_BUFFER_SIZE 8
#define DALI_TX_BUFFER_SIZE 16

// DALI timing constants for 9600 Hz sampling (104.167 us per sample)
#define DALI_SAMPLES_PER_BIT 8  // 1200 baud = 833us per bit / 104us = ~8 samples
#define DALI_HALF_BIT_SAMPLES 4

class Dali {
public:
    // Callback function types for HAL
    typedef bool (*BusIsHighFunc)();
    typedef void (*BusSetHighFunc)();
    typedef void (*BusSetLowFunc)();

    Dali();
    
    // Initialize the library with hardware abstraction callbacks
    void begin(BusIsHighFunc bus_is_high, BusSetHighFunc bus_set_high, BusSetLowFunc bus_set_low);
    
    // Call this from a 9600 Hz timer ISR
    void timer();
    
    // Transmit a forward frame and wait for completion
    // Returns true if transmission successful
    bool tx_wait(uint8_t addr, uint8_t data, uint16_t timeout_ms = 50);
    
    // Transmit a forward frame and wait for backward frame response
    // Returns the response byte, or 0 if timeout/error
    uint8_t tx_wait_rx(uint8_t addr, uint8_t data, uint16_t timeout_ms = 100);
    
    // Check if a backward frame has been received (passive monitoring)
    // Returns number of bits received (8 if complete frame), fills *data with the byte
    uint8_t rx(uint8_t* data);
    
    // Enable/disable debug logging
    void set_debug(bool enable) { m_debug = enable; }
    
private:
    // HAL callbacks
    BusIsHighFunc m_bus_is_high;
    BusSetHighFunc m_bus_set_high;
    BusSetLowFunc m_bus_set_low;
    
    // TX state machine
    enum TxState {
        TX_IDLE,
        TX_START_BIT,
        TX_DATA_BIT_HIGH,
        TX_DATA_BIT_LOW,
        TX_STOP_BITS,
        TX_WAIT_RESPONSE,
        TX_COMPLETE
    };
    
    volatile TxState m_tx_state;
    volatile uint8_t m_tx_addr;
    volatile uint8_t m_tx_data;
    volatile uint8_t m_tx_bit_index;
    volatile uint8_t m_tx_sample_count;
    volatile bool m_tx_active;
    volatile bool m_tx_success;
    
    // RX state machine
    enum RxState {
        RX_IDLE,
        RX_START_BIT,
        RX_DATA_BIT,
        RX_STOP_BIT,
        RX_COMPLETE
    };
    
    volatile RxState m_rx_state;
    volatile uint8_t m_rx_data;
    volatile uint8_t m_rx_bit_index;
    volatile uint8_t m_rx_sample_count;
    volatile uint8_t m_rx_bits_received;
    volatile bool m_rx_frame_ready;
    
    // Response storage
    volatile uint8_t m_response_byte;
    volatile bool m_response_ready;
    
    // Debug flag
    bool m_debug;
    
    // Internal helpers
    void tx_timer_handler();
    void rx_timer_handler();
    void start_tx(uint8_t addr, uint8_t data);
    void reset_rx();
};

// Global instance (to be used by ISR)
extern Dali dali;
