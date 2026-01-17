#include "DALI_Lib.h"
#include "dali.h"  // For logging macros
#include <Arduino.h>  // For millis() and delay()

// Global instance
Dali dali;

Dali::Dali()
    : m_bus_is_high(nullptr)
    , m_bus_set_high(nullptr)
    , m_bus_set_low(nullptr)
    , m_tx_state(TX_IDLE)
    , m_tx_active(false)
    , m_tx_success(false)
    , m_rx_state(RX_IDLE)
    , m_rx_bits_received(0)
    , m_rx_frame_ready(false)
    , m_response_ready(false)
    , m_debug(false)
{
}

void Dali::begin(BusIsHighFunc bus_is_high, BusSetHighFunc bus_set_high, BusSetLowFunc bus_set_low) {
    m_bus_is_high = bus_is_high;
    m_bus_set_high = bus_set_high;
    m_bus_set_low = bus_set_low;
    
    // Initialize bus to idle state (high)
    if (m_bus_set_high) {
        m_bus_set_high();
    }
    
    m_tx_state = TX_IDLE;
    m_rx_state = RX_IDLE;
    m_tx_active = false;
    m_response_ready = false;
    m_rx_frame_ready = false;
}

void Dali::timer() {
    // This is called from ISR at 9600 Hz (every 104.167 us)
    
    // Handle TX state machine
    if (m_tx_active) {
        tx_timer_handler();
    }
    
    // Handle RX state machine (passive monitoring or response reception)
    rx_timer_handler();
}

void Dali::tx_timer_handler() {
    m_tx_sample_count++;
    
    switch (m_tx_state) {
        case TX_IDLE:
            // Nothing to do
            break;
            
        case TX_START_BIT:
            // Start bit: HIGH for half bit, then LOW for half bit
            if (m_tx_sample_count <= DALI_HALF_BIT_SAMPLES) {
                m_bus_set_high();
            } else {
                m_bus_set_low();
            }
            
            if (m_tx_sample_count >= DALI_SAMPLES_PER_BIT) {
                m_tx_state = TX_DATA_BIT_HIGH;
                m_tx_sample_count = 0;
                m_tx_bit_index = 0;
            }
            break;
            
        case TX_DATA_BIT_HIGH:
        case TX_DATA_BIT_LOW: {
            // Transmit 16 bits (8 addr + 8 data) using Manchester encoding
            // Manchester: logical 1 = HIGH then LOW, logical 0 = LOW then HIGH
            uint16_t frame = ((uint16_t)m_tx_addr << 8) | m_tx_data;
            bool bit = (frame >> (15 - m_tx_bit_index)) & 1;
            
            // Manchester encoding: 1 = HIGH then LOW, 0 = LOW then HIGH
            if (bit) {
                if (m_tx_sample_count <= DALI_HALF_BIT_SAMPLES) {
                    m_bus_set_high();
                } else {
                    m_bus_set_low();
                }
            } else {
                if (m_tx_sample_count <= DALI_HALF_BIT_SAMPLES) {
                    m_bus_set_low();
                } else {
                    m_bus_set_high();
                }
            }
            
            if (m_tx_sample_count >= DALI_SAMPLES_PER_BIT) {
                m_tx_bit_index++;
                m_tx_sample_count = 0;
                
                if (m_tx_bit_index >= 16) {
                    // All bits sent, send stop bits
                    m_tx_state = TX_STOP_BITS;
                    m_tx_sample_count = 0;
                }
            }
            break;
        }
            
        case TX_STOP_BITS:
            // Stop bits: 4 samples of HIGH
            m_bus_set_high();
            
            if (m_tx_sample_count >= 4) {
                // TX complete
                m_tx_state = TX_IDLE;
                m_tx_active = false;
                m_tx_success = true;
            }
            break;
            
        default:
            m_tx_state = TX_IDLE;
            m_tx_active = false;
            break;
    }
}

void Dali::rx_timer_handler() {
    // Sample the bus
    bool bus_high = m_bus_is_high ? m_bus_is_high() : true;
    
    switch (m_rx_state) {
        case RX_IDLE:
            // Look for start bit (HIGH-to-LOW transition for Manchester start)
            if (!bus_high) {
                m_rx_state = RX_START_BIT;
                m_rx_sample_count = 0;
                m_rx_bit_index = 0;
                m_rx_data = 0;
            }
            break;
            
        case RX_START_BIT:
            m_rx_sample_count++;
            // Wait for full start bit (8 samples)
            if (m_rx_sample_count >= DALI_SAMPLES_PER_BIT) {
                m_rx_state = RX_DATA_BIT_FIRST_HALF;
                m_rx_sample_count = 0;
            }
            break;
            
        case RX_DATA_BIT_FIRST_HALF:
            m_rx_sample_count++;
            
            // Sample at mid-point of first half
            // First half: samples 1-4, mid-point at sample 2-3
            // After increment above, m_rx_sample_count==3 means we're at the 3rd sample (0-indexed: sample 2)
            if (m_rx_sample_count == 3) {
                m_rx_first_half_sample = bus_high;
            }
            
            // After first half, move to second half
            if (m_rx_sample_count >= DALI_HALF_BIT_SAMPLES) {
                m_rx_state = RX_DATA_BIT_SECOND_HALF;
            }
            break;
            
        case RX_DATA_BIT_SECOND_HALF:
            m_rx_sample_count++;
            
            // Sample at mid-point of second half
            // Second half: samples 5-8, mid-point at sample 6-7
            // After increment above, m_rx_sample_count==7 means we're at the 7th sample (0-indexed: sample 6)
            if (m_rx_sample_count == 7) {
                bool second_half_sample = bus_high;
                
                // Manchester decoding: compare first and second half
                // Logical 1: HIGH->LOW (first half HIGH, second half LOW)
                // Logical 0: LOW->HIGH (first half LOW, second half HIGH)
                m_rx_data = (m_rx_data << 1);
                if (m_rx_first_half_sample && !second_half_sample) {
                    // HIGH->LOW = 1
                    m_rx_data |= 1;
                }
                // else LOW->HIGH or other = 0 (already shifted in 0)
            }
            
            if (m_rx_sample_count >= DALI_SAMPLES_PER_BIT) {
                m_rx_sample_count = 0;
                m_rx_bit_index++;
                
                if (m_rx_bit_index >= 8) {
                    // Received all 8 bits
                    m_rx_bits_received = 8;
                    m_rx_frame_ready = true;
                    m_response_byte = m_rx_data;
                    m_response_ready = true;
                    m_rx_state = RX_STOP_BIT;
                } else {
                    // Continue with next bit
                    m_rx_state = RX_DATA_BIT_FIRST_HALF;
                }
            }
            break;
            
        case RX_STOP_BIT:
            m_rx_sample_count++;
            if (m_rx_sample_count >= 4 || bus_high) {
                // Stop bit complete, return to idle
                m_rx_state = RX_IDLE;
                m_rx_sample_count = 0;
            }
            break;
            
        default:
            m_rx_state = RX_IDLE;
            break;
    }
}

void Dali::start_tx(uint8_t addr, uint8_t data) {
    m_tx_addr = addr;
    m_tx_data = data;
    m_tx_bit_index = 0;
    m_tx_sample_count = 0;
    m_tx_state = TX_START_BIT;
    m_tx_active = true;
    m_tx_success = false;
    m_response_ready = false;
    
    if (m_debug) {
        DALI_LOGD("DALI TX: addr=0x%02x data=0x%02x", addr, data);
    }
}

void Dali::reset_rx() {
    m_rx_state = RX_IDLE;
    m_rx_frame_ready = false;
    m_rx_bits_received = 0;
    m_response_ready = false;
}

bool Dali::tx_wait(uint8_t addr, uint8_t data, uint16_t timeout_ms) {
    reset_rx();
    start_tx(addr, data);
    
    // Wait for TX to complete
    uint32_t start = millis();
    while (m_tx_active && (millis() - start) < timeout_ms) {
        delay(1);
    }
    
    if (!m_tx_success) {
        if (m_debug) {
            DALI_LOGW("DALI TX timeout");
        }
        return false;
    }
    
    return true;
}

uint8_t Dali::tx_wait_rx(uint8_t addr, uint8_t data, uint16_t timeout_ms) {
    reset_rx();
    start_tx(addr, data);
    
    // Wait for TX to complete
    uint32_t start = millis();
    while (m_tx_active && (millis() - start) < timeout_ms) {
        delay(1);
    }
    
    if (!m_tx_success) {
        if (m_debug) {
            DALI_LOGW("DALI TX timeout");
        }
        return 0;
    }
    
    // Wait for response (backward frame)
    // DALI spec: response should arrive within 22 bit periods (~18ms)
    start = millis();
    while (!m_response_ready && (millis() - start) < timeout_ms) {
        delay(1);
    }
    
    if (m_response_ready) {
        uint8_t response = m_response_byte;
        if (m_debug) {
            DALI_LOGD("DALI RX: response=0x%02x", response);
        }
        m_response_ready = false;
        return response;
    }
    
    // No response (common for commands that don't reply)
    return 0;
}

uint8_t Dali::rx(uint8_t* data) {
    if (m_rx_frame_ready && data) {
        *data = m_rx_data;
        m_rx_frame_ready = false;
        return m_rx_bits_received;
    }
    return 0;
}
