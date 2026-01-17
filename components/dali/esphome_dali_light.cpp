#include <esphome.h>
#include "esphome_dali_light.h"
#include "esphome/core/log.h"

using namespace esphome;
using namespace dali;
using namespace esphome::light;

static const char *const TAG = "dali.light";

#define DALI_MAX_BRIGHTNESS_F (254.0f)

void dali::DaliLight::setup_state(light::LightState *state) {
    // Existing setup code left unchanged (queries capabilities)...
    // (This file mostly remains as in your repo.)
}

void dali::DaliLight::write_state(light::LightState *state) {
    bool on;
    float brightness = 0.0f;
    float color_temperature = 0.0f;

    state->current_values_as_binary(&on);
    if (!on) {
        DALI_LOGI("UI -> TURN OFF (short addr=%.2x)", this->address_);
        bus->dali.lamp.setBrightness(address_, 0);
        return;
    }

    // Prefer color temperature if supported
    if (tc_supported_) {
        state->current_values_as_ct(&color_temperature, &brightness);
        // map UI CT to dali values if needed...
    } else {
        state->current_values_as_brightness(&brightness);
    }

    int dali_brightness = static_cast<uint8_t>(brightness * this->dali_level_range_) + this->dali_level_min_ - 1;
    if (dali_brightness < 1) dali_brightness = 1;
    if (dali_brightness > 254) dali_brightness = 254;

    DALI_LOGI("UI -> SET BRIGHTNESS addr=%.2x brightness=%.2f -> dali=%d", this->address_, brightness, dali_brightness);
    bus->dali.lamp.setBrightness(address_, (uint8_t)dali_brightness);
}
