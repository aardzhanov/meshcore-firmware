#include <Arduino.h>
#include "target.h"
#include <helpers/radiolib/CustomLR1110Wrapper.h>
#include <helpers/sensors/MicroNMEALocationProvider.h>

ESP32Board board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);
WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
MASensorManager sensors = MASensorManager(nmea);

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
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

#ifndef LORA_CR
  #define LORA_CR      5
#endif


bool radio_init() {
  //rtc_clock.begin(Wire);

#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  SPI.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
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

void MASensorManager::start_gps() {
  if(!gps_active) {
    MESH_DEBUG_PRINTLN("starting GPS");
    gps_active = true;
  }
}

void MASensorManager::stop_gps() {
  if(gps_active) {
    MESH_DEBUG_PRINTLN("stopping GPS");
    gps_active = false;
  }
}

bool MASensorManager::begin() {
  Serial1.setPins(PIN_GPS_RX, PIN_GPS_TX);
  Serial1.begin(9600);
  delay(500);
  return true;
}

bool MASensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if(requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  return true;
}

void MASensorManager::loop() {
  static long next_gps_update = 0;
  _location->loop();
  if(millis() > next_gps_update && gps_active) {
    if(_location->isValid()) {
      node_lat = ((double)_location->getLatitude()) / 1000000.;
      node_lon = ((double)_location->getLongitude()) / 1000000.;
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
    }
    next_gps_update = millis() + 1000;
  }
}

int MASensorManager::getNumSettings() const { return 1; }  // just one supported: "gps" (power switch)

const char* MASensorManager::getSettingName(int i) const {
  return i == 0 ? "gps" : NULL;
}
const char* MASensorManager::getSettingValue(int i) const {
  if(i == 0) {
    return gps_active ? "1" : "0";
  }
  return NULL;
}
bool MASensorManager::setSettingValue(const char* name, const char* value) {
  if(strcmp(name, "gps") == 0) {
    if(strcmp(value, "0") == 0) {
      stop_gps();
    } else {
      start_gps();
    }
    return true;
  }
  return false;  // not supported
}
