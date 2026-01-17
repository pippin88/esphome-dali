# DALI Library Integration - Implementation Notes

## Overview

This implementation integrates a timer-driven DALI library (based on the qqqlab/Waveshare design) into the ESPHome DALI component. The new implementation replaces the fragile bit-banging approach with a hardware timer-driven ISR for reliable DALI communication.

## Key Features

### 1. Hardware Timer-Driven Communication
- **Sampling Rate**: 9600 Hz (104.167 µs per sample)
- **Encoding**: Manchester encoding for forward frames (TX)
- **Decoding**: NRZ for backward frames (RX)
- **Timer**: Configurable hardware timer (defaults to Timer 0)

### 2. ISR-Safe GPIO Operations
- Direct ESP-IDF GPIO calls from ISR context
- Cached GPIO pin numbers for fast access
- IRAM_ATTR functions for zero-latency interrupt handling

### 3. Runtime YAML Configuration

Two new configuration options are available in YAML:

```yaml
dali:
  debug_rxtx: false  # Enable/disable TX/RX debug logging
  rx_pull: PULLDOWN  # Configure RX pin pull resistor
```

#### debug_rxtx Option
- **Type**: Boolean
- **Default**: false
- **Description**: When enabled, logs all DALI TX and RX frames to serial output
- **Use Case**: Debugging DALI bus communication, verifying commands are sent/received

#### rx_pull Option
- **Type**: Enum (NONE, PULLUP, PULLDOWN)
- **Default**: PULLDOWN
- **Description**: Configures the internal pull resistor for the RX pin
- **PULLDOWN**: Recommended for most Waveshare-style DALI adapters
- **NONE**: Use if your DALI adapter already has pull resistors
- **PULLUP**: Rarely needed, for inverted RX signals

### 4. Backward Compatibility
- Existing YAML configurations work without changes
- Sensible defaults ensure no breaking changes
- Discovery and address initialization preserved

## Architecture

### File Structure

```
components/dali/
├── DALI_Lib.h          # Timer-driven DALI library interface
├── DALI_Lib.cpp        # Timer-driven DALI library implementation
├── esphome_dali.h      # ESPHome bus component header (updated)
├── esphome_dali.cpp    # ESPHome bus component (updated)
├── __init__.py         # YAML config schema (updated)
├── light.py            # Light platform config (fixed API)
└── dali.h              # DALI protocol definitions (unchanged)
```

### Call Flow

1. **Setup**:
   ```
   DaliBusComponent::setup()
   ├── Configure GPIO pins
   ├── Cache raw GPIO numbers
   ├── dali.begin(callbacks)  # Initialize library
   └── setup_timer()           # Start 9600 Hz timer
   ```

2. **Transmission**:
   ```
   sendForwardFrame(addr, data)
   └── dali.tx_wait(addr, data)
       └── ISR: dali.timer()
           └── tx_timer_handler()  # Manchester encoding
   ```

3. **Reception**:
   ```
   receiveBackwardFrame()
   └── dali.rx(&data)
       └── ISR: dali.timer()
           └── rx_timer_handler()  # NRZ decoding
   ```

## Implementation Details

### Timer Configuration

The hardware timer is configured as follows:
- **Timer Number**: 0 (configurable via DALI_TIMER_NUM constant)
- **Divider**: 80 (80 MHz APB clock → 1 MHz tick rate)
- **Period**: 104 ticks (104 µs = 9600 Hz)
- **Auto-reload**: Enabled

### Manchester Encoding (TX)

Forward frames use Manchester encoding:
- **Logical 1**: HIGH for half bit, then LOW for half bit
- **Logical 0**: LOW for half bit, then HIGH for half bit
- **Start bit**: Logical 1
- **Data**: 16 bits (8-bit address + 8-bit data)
- **Stop bits**: 4 samples of HIGH

### NRZ Decoding (RX)

Backward frames use simple NRZ encoding:
- Sample in the middle of each bit period
- **HIGH**: Logical 1
- **LOW**: Logical 0
- **Frame**: 8 data bits

### ISR Safety

The ISR implementation ensures:
- All ISR functions marked with IRAM_ATTR
- Direct ESP-IDF GPIO calls (no C++ wrapper overhead)
- Null pointer checks before dereferencing
- Minimal processing in ISR (state machine updates only)

## Testing

### Validation Steps

1. **Configuration Validation**:
   ```bash
   esphome config your_config.yaml
   ```
   Should report "Configuration is valid!"

2. **Code Generation**:
   Generated C++ should include:
   - `dali_bus->set_debug_rxtx(false);`
   - `dali_bus->set_rx_pull(dali::PULLDOWN);`

3. **Runtime Testing**:
   - Enable `debug_rxtx: true` in YAML
   - Monitor serial output for "TX:" and "RX:" messages
   - Verify DALI devices respond to commands
   - Test light control from Home Assistant

### Troubleshooting

#### No RX Responses
- Try different `rx_pull` settings (NONE, PULLUP, PULLDOWN)
- Verify RX pin is correctly connected to DALI RX circuit
- Enable `debug_rxtx: true` to see if TX frames are sent

#### Discovery Fails
- Check TX pin connection
- Verify DALI power supply (16V)
- Enable debug logging to see bus activity

#### Timer Conflicts
- If Timer 0 conflicts with other components, change `DALI_TIMER_NUM` in esphome_dali.cpp
- ESP32 has 4 hardware timers (0-3)

## Future Improvements

Potential enhancements for future versions:

1. **Configurable Timer Number**: Add YAML option to select hardware timer
2. **Adaptive Sampling**: Auto-adjust sampling rate based on bus conditions
3. **Error Recovery**: Automatic retry on transmission failures
4. **Bus Monitor Mode**: Passive monitoring without active transmission
5. **Statistics**: Track TX/RX success rates, error counts

## References

- DALI Protocol: IEC 62386
- qqqlab DALI Library: Inspiration for timer-driven approach
- Waveshare DALI Hardware: Reference hardware platform
- ESPHome Documentation: Component development guidelines
