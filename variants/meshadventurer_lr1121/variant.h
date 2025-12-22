// For OLED LCD
#define I2C_SDA 21
#define I2C_SCL 22

// For GPS, 'undef's not needed
#define GPS_TX_PIN 15
#define GPS_RX_PIN 12
#define PIN_GPS_EN 4
#define GPS_POWER_TOGGLE // Moved definition from platformio.ini to here

#define BUTTON_PIN 39  // The middle button GPIO on the T-Beam
#define BATTERY_PIN 35 // A battery voltage measurement pin, voltage divider connected here to measure battery voltage
#define ADC_CHANNEL ADC1_GPIO35_CHANNEL
#define ADC_MULTIPLIER 2
#define EXT_PWR_DETECT 4    // Pin to detect connected external power source for LILYGO® TTGO T-Energy T18 and other DIY boards
#define EXT_NOTIFY_OUT 12   // Overridden default pin to use for Ext Notify Module (#975).
#define LED_PIN 2           // add status LED (compatible with core-pcb and DIY targets)


#define LORA_DIO_1 33
#define LORA_NSS 18
#define LORA_RESET 23
#define LORA_BUSY 32
#define LORA_SCLK 5
#define LORA_MISO 19
#define LORA_MOSI 27

#define LR11X0_DIO_AS_RF_SWITCH    true
#define LR11X0_DIO3_TCXO_VOLTAGE   3.0