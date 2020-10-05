#ifndef Secrets_h
#define Secrets_h
#define SSID_RR "Router Name"
#define PASS_RR "Password for Router"
int BrokerAddrDefault = 11;                 // sub ip of your MQTT broker (NOT 0 or 255);
#define ThrottleNameDefault "DagThrottle"   // change this to Identify different throttles to Rocrail

static const uint8_t OLED_SCL = SCL;
static const uint8_t OLED_SDA = SDA;

int On_Board_LED = 16;  //

const int buttonLastLoco = 15;
const int buttonNextLoco = 32;
const int buttonBackward = 18;
const int buttonForward = 19;
const int buttonStop = 23;
const int buttonF0 = 13;
const int buttonF1 = 5;
const int buttonF2 = 12;
const int buttonF3 = 14;
const int buttonF4 = 33;
const int buttonF5 = 25;
const int buttonF6 = 26;
const int buttonF7 = 27;

const int FaderUp = 2;
const int FaderDown = 4;

const int ADC_IN = 35;                    //  has divide by 2 resistor network to V battery                                                    // was 35
//NOTE: ADC1 (8 channels, attached to GPIOs 32 - 39),ADC2 is not usable with WiFi https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html
const int ADC_SPEED = 34;                                                                                                          // für Fader
const int Cal_factor = 121;               // set at 121  (for 220K/68k voltage divider.

#endif
