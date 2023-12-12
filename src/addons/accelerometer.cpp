#include "addons/accelerometer.h"
#include "storagemanager.h"
#include "helper.h"
#include "config.pb.h"

#include "enums.pb.h"

#include "hardware/adc.h"

#include <math.h>

#define ADC_MAX ((1 << 12) - 1)
#define ANALOG_CENTER 0.5f // 0.5f is center
#define ANALOG_MAX 1.0f    // 1.0f is max

AccelerometerInput::AccelerometerInput() {
    AccelerometerOptions& options = Storage::getInstance().getAddonOptions().accelerometerOptions;
    options.enabled = true;
    options.has_enabled = true;
}

bool AccelerometerInput::available() {
    return Storage::getInstance().getAddonOptions().accelerometerOptions.enabled;
}

void AccelerometerInput::setup() {
    AccelerometerOptions& options = Storage::getInstance().getAddonOptions().accelerometerOptions;
    options.enabled = true;
    options.has_enabled = true;
    // Make sure GPIO is high-impedance, no pullups etc
    if (isValidPin(options.accelerometer1PinX) ) {
        adc_gpio_init(options.accelerometer1PinX);
    }
    if ( isValidPin(options.accelerometer1PinY) ) {
        adc_gpio_init(options.accelerometer1PinY);
    }
    
	
	// Read pins deviation from center for calibration
	if (options.auto_calibrate) {
		if ( isValidPin(options.accelerometer1PinX) ) {
			adc_select_input(options.accelerometer1PinX-26); // ANALOG1-X
			adc_1_x_center = adc_read();
		}
		if ( isValidPin(options.analogAdc1PinY) ) {
			adc_select_input(options.analogAdc1PinY-26); // ANALOG1-Y
			adc_1_y_center = adc_read();
		}
	}
}

void AccelerometerInput::process()
{
    const AccelerometerOptions& options = Storage::getInstance().getAddonOptions().accelerometerOptions;
    Gamepad * gamepad = Storage::getInstance().GetGamepad();
    float adc_1_x = ACCELEROMETER_CENTER;
    float adc_1_y = ACCELEROMETER_CENTER;
    float adc_deadzone = options.accelerometer_deadzone / 200.0f;
    float x_magnitude_1 = 0.0f;
    float y_magnitude_1 = 0.0f;
    float magnitude = 0.0f;

    if ( isValidPin(options.analogAdc1PinX) ) {
        adc_1_x = readPin(options.analogAdc1PinX, adc_1_x_center, options.auto_calibrate);
    }
    if ( isValidPin(options.analogAdc1PinY) ) {
        adc_1_y = readPin(options.analogAdc1PinY, adc_1_y_center, options.auto_calibrate);
    }

    // Calculations for radialDeadzone() and adjustCircularity()
    // Apply scaled radial deadzones
    if (adc_1_x != ACCELEROMETER_CENTER && adc_1_y != ACCELEROMETER_CENTER) {
        x_magnitude_1 = adc_1_x - ACCELEROMETER_CENTER;
        y_magnitude_1 = adc_1_y - ACCELEROMETER_CENTER;
        magnitude = sqrt((x_magnitude_1 * x_magnitude_1) + (y_magnitude_1 * y_magnitude_1));
        if (adc_deadzone) {
            radialDeadzone(adc_1_x, adc_1_y, adc_deadzone, x_magnitude_1, y_magnitude_1, magnitude);
        }
    }

    // Alter coordinates to force perfect circularity
    if (options.forced_circularity) {
        if (adc_1_x != ACCELEROMETER_CENTER && adc_1_y != ACCELEROMETER_CENTER)
            adjustCircularity(adc_1_x, adc_1_y, adc_deadzone, x_magnitude_1, y_magnitude_1, magnitude);
        
    }

    // Convert to 16-bit value
    if (options.analogAdc1Mode == DpadMode::DPAD_MODE_LEFT_ANALOG) {
        gamepad->state.lx = (uint16_t)(65535.0f*adc_1_x);
        gamepad->state.ly = (uint16_t)(65535.0f*adc_1_y);
    } else if (options.analogAdc1Mode == DpadMode::DPAD_MODE_RIGHT_ANALOG) {
        gamepad->state.rx = (uint16_t)(65535.0f*adc_1_x);
        gamepad->state.ry = (uint16_t)(65535.0f*adc_1_y);
    }
}

float AccelerometerInput::readPin(int pin, uint16_t center, bool autoCalibrate) {
	adc_select_input(pin - 26);
	uint16_t adc_hold = adc_read();

	// Calibrate axis based on off-center
	uint16_t adc_calibrated;

	if (autoCalibrate) {        
		if (adc_hold > center) {
			adc_calibrated = (((adc_hold * 3.35) / 1024) - 1.65) / 0.330;
		}
		else if (adc_hold == center) {
			adc_calibrated = ADC_MAX / 2;
		}
		else {
			adc_calibrated = (((adc_hold * 3.35) / 1024) - 1.65) / 0.330;
		}
	}
	else {
		adc_calibrated = adc_hold;
	}

	float adc_value = ((float)adc_calibrated) / ADC_MAX;

	return adc_value;
}

uint16_t AccelerometerInput::map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void AccelerometerInput::radialDeadzone(float& x, float& y, float deadzone, float x_magnitude, float y_magnitude, float magnitude) {
    if (magnitude < deadzone) {
        x = ACCELEROMETER_CENTER;
        y = ACCELEROMETER_CENTER;
    }
    else {
        float scaling_factor = (magnitude - deadzone) / (1.0f - (deadzone + (deadzone * 0.6f)));
        x = ((x_magnitude / magnitude) * scaling_factor) + ACCELEROMETER_CENTER;
        y = ((y_magnitude / magnitude) * scaling_factor) + ACCELEROMETER_CENTER;

        x = std::fmin(x, 1.0f);
        y = std::fmin(y, 1.0f);
    }
}

void AccelerometerInput::adjustCircularity(float& x, float& y, float deadzone, float x_magnitude, float y_magnitude, float magnitude) {
    if (magnitude > ACCELEROMETER_CENTER) {
        x = ((x_magnitude / magnitude) * ACCELEROMETER_CENTER + ACCELEROMETER_CENTER);
        y = ((y_magnitude / magnitude) * ACCELEROMETER_CENTER + ACCELEROMETER_CENTER);
    }
}