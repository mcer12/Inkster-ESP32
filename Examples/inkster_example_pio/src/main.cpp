
/* Inkster arduino example -  */

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <Arduino.h>
#include <NeoPixelBus.h>

extern "C"
{
#include "epd_driver.h"
#include "epd_highlevel.h"
}
#include "dragon.h"
#include "roboto12.h"

// Has to be NeoEsp32I2s0800KbpsMethod method or NeoEsp32BitBang800KbpsMethod in order not to conflict with epdiy!
// Note that bitbang method can be prone to glitching.
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0Ws2812xMethod> strip(1, 3);
NeoGamma<NeoGammaTableMethod> colorGamma;

EpdiyHighlevelState hl;
uint8_t *fb; // EPD 2bpp buffer

int deepSleepSeconds = 86400;
int temperature = 25;

uint8_t led_power_toggle_pin = 14; // This pin is used to toggle power to the WS2812.
uint8_t button_1_pin = 36;
uint8_t button_2_pin = 39;
uint8_t button_3_pin = 13;
uint64_t wakeUpPins = GPIO_SEL_13 | GPIO_SEL_36 | GPIO_SEL_39; // Wakeup pins, same as button pins
int adc_multiplier = 1.0;                                      // multiplier to compensate ADC innacuracy
int bat_pin = 34;                                              // Battery ADC pin
int therm_pin = 35;                                            // Thermistor ADC pin

// Thermistor settings, change only if you're using different thermistor than specified
int therm_resistance_nom = 100000; // resistance at 25 degrees C
int therm_temp_nom = 25;           // temp. for nominal resistance (almost always 25 C)
int therm_b = 4250;                // The beta coefficient of the thermistor (usually 3000-4000)
int therm_resistor = 100000;       // the value of the 'other' resistor (measured)

esp_adc_cal_characteristics_t *adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

uint8_t readButtons()
{
    uint8_t result = 0;
    if (digitalRead(button_1_pin) == HIGH)
    {
        result = result * 10 + 1;
    }
    if (digitalRead(button_2_pin) == HIGH)
    {
        result = result * 10 + 2;
    }
    if (digitalRead(button_3_pin) == HIGH)
    {
        result = result * 10 + 3;
    }
    return result;
}

void print_uint64_t(uint64_t num)
{

    char rev[128];
    char *p = rev + 1;

    while (num > 0)
    {
        *p++ = '0' + (num % 10);
        num /= 10;
    }
    p--;
    /*Print the number which is now in reverse*/
    while (p > rev)
    {
        Serial.print(*p--);
    }
}

void checkWakeupReason()
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
    {
        uint64_t rst_pin = esp_sleep_get_ext1_wakeup_status();

        if (rst_pin == GPIO_SEL_36)
        {
            Serial.println("[SYSTEM] Wakeup by button 1");
        }
        if (rst_pin == GPIO_SEL_39)
        {
            Serial.println("[SYSTEM] Wakeup by button 2");
        }
        if (rst_pin == GPIO_SEL_13)
        {
            Serial.println("[SYSTEM] Wakeup by button 3");
        }
    }
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
    {
        Serial.println("[SYSTEM] Wakeup by timer");
    }
}

int readADC(int pin, uint8_t sample_count)
{
    uint8_t i;
    float average;
    int samples[sample_count];

    // take N samples in a row, with a slight delay
    for (i = 0; i < sample_count; i++)
    {
        samples[i] = analogRead(pin);
    }

    // average all the samples out
    average = 0;
    for (i = 0; i < sample_count; i++)
    {
        average += samples[i];
    }
    average /= sample_count;

    average *= adc_multiplier; // tilt the curve bit

    return average;
}

int readADC(int pin)
{
    return readADC(pin, 10); // 10 samples by default
}

float measureBattery()
{

    int average = readADC(bat_pin);

    // convert the value to resistance
    // 4095 for 12-bits, 2047 for 11-bits, 1023 for 10-bits, 511 for 9 bits.
    float value = (float)esp_adc_cal_raw_to_voltage(average, adc_chars) * 2 / 1000;

    return value;
}

float measureThermistor()
{
    float resistance;

    int average = readADC(therm_pin);
    float vout = (float)esp_adc_cal_raw_to_voltage(average, adc_chars);

    // convert the value to resistance
    // 4095 for 12-bits, 2047 for 11-bits, 1023 for 10-bits, 511 for 9 bits.
    resistance = (3300 / vout) - 1;
    resistance = therm_resistor / resistance;

    float steinhart;
    steinhart = resistance / therm_resistance_nom; // (R/Ro)
    steinhart = log(steinhart);                    // ln(R/Ro)
    steinhart /= therm_b;                          // 1/B * ln(R/Ro)
    steinhart += 1.0 / (therm_temp_nom + 273.15);  // + (1/To)
    steinhart = 1.0 / steinhart;                   // Invert
    steinhart -= 273.15;                           // convert to C

    return steinhart;
}

void ledColorSet(RgbColor color)
{
    strip.ClearTo(color);
    strip.Show();
}

void ledInit()
{
    pinMode(led_power_toggle_pin, OUTPUT);
    digitalWrite(led_power_toggle_pin, HIGH);
    delay(5); // small delay to let the led power up
    strip.Begin();
    strip.Show();
    Serial.println("[SYSTEM] LED initiated");
}

void deepSleep()
{
    epd_poweroff();
    epd_deinit(); // needs to be called before sleep for low power consumption

    ledColorSet(RgbColor(0, 0, 0));

    pinMode(led_power_toggle_pin, OUTPUT);
    digitalWrite(led_power_toggle_pin, LOW); // cut power to WS2812

    Serial.print("[SYSTEM] Going to sleep for ");
    Serial.println(deepSleepSeconds);
    Serial.println(" seconds.");

    esp_sleep_enable_ext1_wakeup(wakeUpPins, ESP_EXT1_WAKEUP_ANY_HIGH);

    esp_sleep_enable_timer_wakeup(1000000LL * deepSleepSeconds);

    esp_deep_sleep_start();
}

void loop()
{
    /* wow so much empty */
}

void setup()
{

    Serial.begin(115200);
    Serial.println();

    // analogSetAttenuation(ADC_11db); // 11db=0-3.3v(default), 6db=0-2.2v, 2.5db=0-1.5v, 0db=0-1v
    // analogReadResolution(12);       // 12=0->4095(default), 11=0->2047, 10=0->1024, 9=0->511

    checkWakeupReason();

    epd_init(EPD_LUT_64K);
    hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
    fb = epd_hl_get_framebuffer(&hl);

    epd_poweron();

    ledInit();

    ledColorSet(RgbColor(100, 0, 100));
    delay(500);
    ledColorSet(RgbColor(0, 100, 100));
    delay(500);
    ledColorSet(RgbColor(100, 100, 0));
    delay(500);
    ledColorSet(RgbColor(0, 0, 0));

    // Characterize ADC at particular atten

    // Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        Serial.println("eFuse Vref");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        Serial.println("Two Point");
    }
    else
    {
        Serial.println("Default");
    }

    uint32_t reading = readADC(bat_pin);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars) * 2;
    Serial.println(voltage);

    // ESP32 ADC is highly inaccurate and will have to be callibrated
    // Easiest way to calibrate ADC is using fully charged lipo with known voltage or better, using ESP32 vref fuse value
    float bat = measureBattery();
    float temp = measureThermistor(); // Thermistor is powered with the display to save power -> you need to run epd_poweron() before measurement!
    Serial.print("[SYSTEM] Battery voltage: ");
    Serial.print(bat);
    Serial.println("V");
    Serial.print("[SYSTEM] Thermistor temperature: ");
    Serial.print(temp);
    Serial.println("°C");

    int cursor_x = epd_rotated_display_width() / 2;
    int cursor_y = epd_rotated_display_height() / 2;

    EpdFontProperties font_props = epd_font_properties_default();
    font_props.flags = EPD_DRAW_ALIGN_CENTER;

    String infoText = "Battery voltage: " + String(bat) + "\nTemperature: " + String(temp);

    epd_fullclear(&hl, temperature);
    // epd_set_rotation(EPD_ROT_INVERTED_LANDSCAPE);
    epd_write_string(&roboto, infoText.c_str(), &cursor_x, &cursor_y, fb, &font_props);
    epd_hl_update_screen(&hl, MODE_GC16, temperature);

    delay(10000);

    EpdRect dragon_area = {
        .x = 0,
        .y = 0,
        .width = dragon_width,
        .height = dragon_height};

    unsigned long updateMillis = millis();

    // If you want to make multiple screen updates, run epd_fullclear before epd_copy_to_framebuffer
    epd_fullclear(&hl, temperature);
    epd_copy_to_framebuffer(dragon_area, dragon_data, epd_hl_get_framebuffer(&hl));

    enum EpdDrawError _err = epd_hl_update_screen(&hl, MODE_GC16, temperature);

    Serial.print("[SYSTEM] Display update took ");
    Serial.print((millis() - updateMillis));
    Serial.println("ms");

    deepSleep();
}