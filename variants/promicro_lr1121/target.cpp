#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>

PromicroBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
#if ENV_INCLUDE_GPS
#include <helpers/sensors/MicroNMEALocationProvider.h>
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
EnvironmentSensorManager sensors;
#endif

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true, true);
#endif

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#define RF_SWITCH_TABLE

#ifdef RF_SWITCH_TABLE

#ifdef NICERF_2G4
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO7,
  RADIOLIB_LR11X0_DIO8,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6  DIO7  DIO8
  { LR11x0::MODE_STBY,  {LOW,  LOW,  LOW,  LOW  }},
  { LR11x0::MODE_RX,    {LOW,  LOW,  LOW,  LOW  }},
  { LR11x0::MODE_TX,    {LOW,  LOW,  LOW,  HIGH }},
  { LR11x0::MODE_TX_HP, {LOW,  LOW,  LOW,  HIGH }},
  { LR11x0::MODE_TX_HF, {LOW,  HIGH, HIGH, LOW  }},
  { LR11x0::MODE_GNSS,  {LOW,  LOW,  LOW,  LOW  }},
  { LR11x0::MODE_WIFI,  {HIGH, LOW,  LOW,  LOW  }},
  END_OF_MODE_TABLE,
};
#endif

#ifdef NICERF_1G9
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO8,
  RADIOLIB_NC,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
    // mode                 DIO5  DIO6  DIO8
    {LR11x0::MODE_STBY,  {LOW,  LOW,  LOW  }},
    {LR11x0::MODE_RX,    {LOW,  LOW,  LOW  }},
    {LR11x0::MODE_TX,    {LOW,  LOW,  HIGH }},
    {LR11x0::MODE_TX_HP, {LOW,  LOW,  HIGH }},
    {LR11x0::MODE_TX_HF, {LOW,  HIGH, LOW  }},
    {LR11x0::MODE_GNSS,  {LOW,  LOW,  LOW  }},
    {LR11x0::MODE_WIFI,  {HIGH, LOW,  LOW  }},
    END_OF_MODE_TABLE,
};
#endif

#endif

bool radio_init() {
Serial.println("Starting radio...");
rtc_clock.begin(Wire);
#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;  // fail
  }
  
  radio.setCRC(2);
  radio.explicitHeader();

#ifdef RF_SWITCH_TABLE
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
#endif
#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

  return true;  // success
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(uint8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

