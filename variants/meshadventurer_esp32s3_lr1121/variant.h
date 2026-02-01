// For OLED LCD
#define I2C_SDA 17
#define I2C_SCL 18

// For GPS, 'undef's not needed
#define GPS_TX_PIN 22
#define GPS_RX_PIN 21
#define PIN_GPS_EN 8
#define GPS_POWER_TOGGLE // Moved definition from platformio.ini to here

#define BUTTON_PIN 0  // The middle button GPIO on the T-Beam
//#define BATTERY_PIN 1 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
//#define ADC_CHANNEL ADC1_GPIO1_CHANNEL
//#define ADC_MULTIPLIER 2
//#define EXT_PWR_DETECT 4    // Pin to detect connected external power source for LILYGO® TTGO T-Energy T18 and other DIY boards
//#define EXT_NOTIFY_OUT 12   // Overridden default pin to use for Ext Notify Module (#975).
#define LED_PIN 2           // add status LED (compatible with core-pcb and DIY targets)   


#define LORA_DIO_1 47
#define LORA_NSS 4
#define LORA_RESET 1
#define LORA_BUSY 40
#define LORA_SCLK 5
#define LORA_MISO 6
#define LORA_MOSI 10

#define LR11X0_DIO_AS_RF_SWITCH    true
#define LR11X0_DIO3_TCXO_VOLTAGE   3.0

#define PIN_NEOPIXEL            48
#define NEOPIXEL_NUM            1