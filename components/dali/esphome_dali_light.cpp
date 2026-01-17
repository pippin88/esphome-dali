
#include <esphome.h>
#include "esphome_dali_light.h"
#include "esphome/core/log.h"

using namespace esphome;

static const char *const TAG = "dali.light";

#define DALI_MAX_BRIGHTNESS_F (254.0f)

namespace esphome {
namespace dali {

light::LightTraits DaliLight::get_traits() {
  light::LightTraits traits;
  // Only advertise brightness control here. More capabilities (CT) are
  // detected at runtime in setup_state() and surfaced separately.
  traits.set_supported_color_modes({ light::ColorMode::BRIGHTNESS });
  return traits;
}

// Called when the LightState is created; query device capabilities here.
void DaliLight::setup_state(light::LightState *state) {
  // Initialization code for DaliLight
  // Exclude broadcast and group addresses from querying
  if ((this->address_ != ADDR_BROADCAST) && ((this->address_ & ADDR_GROUP_MASK) == 0)) {
    ESP_LOGD(TAG, "Querying DALI device capabilities (SA: %.2x)...", this->address_);

    // Use the high-level DaliMaster on the bus to check presence and query min/max
    if (bus->dali.isDevicePresent(this->address_)) {
      ESP_LOGD(TAG, "DALI[%.2x] is present", this->address_);

      this->dali_level_min_ = bus->dali.lamp.getMinLevel(this->address_);
      this->dali_level_max_ = bus->dali.lamp.getMaxLevel(this->address_);
      if (this->dali_level_max_ < this->dali_level_min_) {
        // Sanity: swap if reported backwards
        uint8_t t = this->dali_level_max_;
        this->dali_level_max_ = this->dali_level_min_;
        this->dali_level_min_ = t;
      }
      this->dali_level_range_ = (float)(this->dali_level_max_ - this->dali_level_min_ + 1);
      ESP_LOGD(TAG, "Reported min:%d max:%d", this->dali_level_min_, this->dali_level_max_);

      // Check color temperature capability (store flag; don't change traits here)
      this->tc_supported_ = bus->dali.color.isTcCapable(this->address_);
      if (this->tc_supported_) {
        ESP_LOGD(TAG, "DALI[%.2x] supports color temperature", this->address_);
        // Query a couple of parameters (best-effort)
        uint16_t coolest = bus->dali.color.queryParameter(this->address_, DaliColorParam::ColourTemperatureTcCoolest);
        uint16_t warmest = bus->dali.color.queryParameter(this->address_, DaliColorParam::ColourTemperatureTcWarmest);
        ESP_LOGD(TAG, "Tc(cool)=%d, Tc(warm)=%d", coolest, warmest);
        // Keep reported values if they look reasonable
        if (coolest >= 100 && warmest <= 1000 && coolest < warmest) {
          this->dali_tc_coolest_ = (float)coolest;
          this->dali_tc_warmest_ = (float)warmest;
        }
      } else {
        ESP_LOGD(TAG, "DALI[%.2x] does not report color temperature support", this->address_);
      }
    } else {
      ESP_LOGW(TAG, "DALI[%.2x] not present during setup_state", this->address_);
    }
  } else {
    ESP_LOGD(TAG, "Skipping capability query for group/broadcast address %.2x", this->address_);
  }
}

void DaliLight::write_state(light::LightState *state) {
  // Called when the light state should be written to the hardware
  bool on = false;
  float brightness = 0.0f;
  float color_temperature = 0.0f;

  state->current_values_as_binary(&on);

  if (!on) {
    // Turn off the light
    ESP_LOGI(TAG, "UI -> TURN OFF (short addr=%.2x)", this->address_);
    bus->dali.lamp.setBrightness(this->address_, 0);
    return;
  }

  // Determine which values are present
  if (tc_supported_) {
    // If color temperature supported, prefer temperature + brightness if provided
    state->current_values_as_ct(&color_temperature, &brightness);
    // Mapping of CT is left as-is; brightness mapping applies below.
  } else {
    state->current_values_as_brightness(&brightness);
  }

  // brightness is in range 0..1.0 — map to DALI level considering reported min/max
  uint8_t dali_value = 0;
  if (brightness <= 0.0f) {
    dali_value = 0;
  } else {
    // Map fractional brightness into device's min..max range
    uint32_t mapped = (uint32_t)(brightness * (this->dali_level_range_ - 1.0f)) + this->dali_level_min_;
    if (mapped < 1) mapped = 1;
    if (mapped > 254) mapped = 254;
    dali_value = (uint8_t)mapped;
  }

  ESP_LOGI(TAG, "UI -> SET BRIGHTNESS addr=%.2x brightness=%.3f -> dali=%d", this->address_, brightness, dali_value);

  // Use DaliMaster high-level API to set brightness
  bus->dali.lamp.setBrightness(this->address_, dali_value);
}

}  // namespace dali
}  // namespace esphome
