//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Spirometrie Project
// https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/
// BLE by Andreas Spiess https://github.com/SensorsIot/Bluetooth-BLE-on-Arduino-IDE
// Modifications by Ulrich Rissel
const String Version = "V1.1 2022/04/24"; //20mm Venturi
// TTGO T-Display: GND-G, SDA-Pin21, SCL-Pin22 VCC-3V for all sensors
//
// Use 20mm Venturi nozzle for trained athlets! Flow limit = 8.1 L/sec
// Use 16mm Nozzle for higher resolution at normal air flow, limit = 4.6 L/sec
// Modifikations for 16/20mm in lines 104 - 112
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "esp_adc_cal.h" // ADC calibration data
#define ADC_EN 14  //ADC_EN is the ADC detection enable port
#define ADC_PIN 34
int vref = 1100;

#include "Sensirion_GadgetBle_Lib.h"  //This is library to connect to Sensirion App
#include "DFRobot_OxygenSensor.h"    //Library for Oxygen sensor
#include "STC31.h"
#include <Wire.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>

// declarations for bluetooth serial --------------
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

// declarations for BLE ---------------------
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h> // used for notifications 0x2902: Client Characteristic Configuration

byte flags = 0b00111110;
byte bpm;

byte heart[8] = { 0b00001110, 60, 0, 0, 0 , 0, 0, 0}; // defines the BT heartrate characteristic
// Byte[0]: flags: 0b00001110:
// Byte[0]: not used/n.u./n.u./RR value available/Energy val.av./Sensor contact status/Sens.cont.supported/HR Format: (0: uint_8)
// Byte[1]: HR (uint_8)
// Byte[2]: Energy in J MSB
// Byte[3]: Energy in J LSB
// Byte[4]: RR
// Byte[5]: RR
// Byte[6]: ?
// Byte[7]: ?

byte hrmPos[1] = {2};

bool _BLEClientConnected = false;

#define heartRateService BLEUUID((uint16_t)0x180D)
BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A38), BLECharacteristic::PROPERTY_READ);
BLEDescriptor heartRateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901)); //0x2901: Characteristic User Description

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

// ------------------------------------------

//#include <Adafruit_BMP085.h> //Library for barometric sensor
//Adafruit_BMP085 bmp;

//Starts Screen for TTGO device
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

//Labels the pressure sensor: mySensor
//Omron_D6FPH mySensor;

//Label of oxygen sensor
DFRobot_OxygenSensor Oxygen;
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
#define Oxygen_IICAddress ADDRESS_3  //I2C  label for o2 address

//Defines button state for adding wt
const int  buttonPin1 = 0;
const int  buttonPin2 = 35;
int wtTotal = 0;
int buttonPushCounter1 = 0;   // counter for the duration of button1 pressed
int buttonState1 = 1;         // current state of the button
int buttonPushCounter2 = 0;   // counter for the duration of button2 pressed
int buttonState2 = 1;         // current state of the button
int screenChanged = 0;
int screenNr = 1;
int HeaderStreamed = 0;
int HeaderStreamedBT = 0;
int DEMO = 0; // 1 = DEMO-mode

float area_1 = 0.000531;   // = 26mm diameter

// ################################################################################
// Parameters for 16mm Venturi nozzle ---------------------------------------
float area_2 = 0.000201; // uncomment for 16mm Venturi
float correctionSensor = 0.92; // for 16mm venturi measuresed with 3L calibration syringe
// ################################################################################
// Parameters for 20mm Venturi nozzle ---------------------------------------
// float area_2 = 0.000284; // uncomment for 20mm Venturi
// float correctionSensor = 1.00; // for 20mm venturi measuresed with 3L calibration syringe
// ################################################################################

float rho = 1.225; // ATP conditions: density based on ambient conditions, dry air
float rhoSTPD = 1.292; // STPD conditions: density at 0°C, MSL, 1013.25 hPa, dry air
float rhoBTPS = 1.123; // BTPS conditions: density at ambient  pressure, 35°C, 95% humidity
float massFlow = 0;
float volFlow = 0;
float volumeTotal = 0; //variable for holding total volume of breath
float pressure = 0.0; //differential pressure of the venturi nozzle
// ---- Venturi/TE-sign & autozero ----
float pressureOffset = 0.0f;  // autozero-offset i Pa (dras av råvärdet)
int   pressureSign   = +1;    // +1 om TE-PLUS-porten ska ge positivt vid utandning.
                              // Sätt till -1 om tecknet blir "fel" vid ditt montage.
float volumeVE = 0.0;
float volumeVEmean = 0.0;
float volumeExp = 0.0;

//############### correction for sensor volumeTotal ###################



//#####################################################################

float weightkg = 70.0; // Standard-body-weight

float TimerVolCalc = 0.0;
float Timer5s = 0.0;
float Timer1min = 0.0;
float TimerVO2calc = 0.0;
float TimerVO2diff = 0.0; // used for integral of calories
float TimerStart = 0.0;
float TotalTime = 0.0;
String TotalTimeMin = String("00:00");
int readVE = 0;
float TimerVE = 0.0;
float DurationVE = 0.0;

float lastO2 = 0;
float initialO2 = 0;
float co2 = 0;
float calTotal = 0;
float vo2Cal = 0;
float vo2CalDay = 0.0; // calories per day
float vo2CalDayMax = 0.0; //highest value of calories per day
float vo2Max = 0; //value of vo2Max/min/kg, calculated every 30 seconds
float vo2Total = 0.0; //value of total vo2Max/min
float vo2MaxMax = 0; //Best value of vo2 max for whole time machine is on

float freqVE = 0.0; //ventilation frequency
float freqVEmean = 0.0;

float expiratVol = 0.0; // last expiratory volume in L
float volumeTotalOld = 0.0;
float volumeTotal2 = 0.0;
float TempC = 15.0; //Air temperature in Celsius barometric sensor BMP180
float PresPa = 101325; //uncorrected (absolute) barometric pressure

float Battery_Voltage = 0.0;


// ---- TE SM923x/SM933x minimal driver (I2C) ----
#define TE_PRESSURE_I2C_ADDR 0x6C     // 7-bit I2C address
#define TE_REG_DSP_T     0x2E          // temperature (2 bytes)
#define TE_REG_DSP_S     0x30          // pressure (2 bytes)
#define TE_REG_STATUS    0x36          // status (2 bytes)

GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_CO2);

// Sätt rätt spann för din modell:
// SM9336 = differential ±250 Pa => pmin=-250, pmax=+250
// SM9233 = gage 0..250 Pa => pmin=0, pmax=+250
static const float  TE_PMIN   = -250.0f;   // Pa
static const float  TE_PMAX   =  250.0f;   // Pa
static const int16_t TE_OUTMIN = -26215;
static const int16_t TE_OUTMAX =  26214;

// ---- CO2: STC31 (Mikroe CO2 Click) ----
STC31 co2Sensor;                  // CO2-objekt
bool  stc_ok     = false;   // init-status
float co2Pct     = NAN;     // CO2 vol%
float stcTempC   = NAN;     // intern temp från STC31 (°C)
float vco2Total  = 0.0f;    // ml/min (beräknat)
float RER        = NAN;     // VCO2/VO2 (dimensionlöst)
const float FI_CO2 = 0.04f; // inandad CO2 ≈ 0.04 vol% (400 ppm)


void i2cScan() {
  Serial.println("Scanning I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found I2C device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Scan done.");
}



bool te_begin() {
  Wire.beginTransmission(TE_PRESSURE_I2C_ADDR);
  Wire.write(0x2E);                               // blocklässtart: T,S,STATUS
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TE_PRESSURE_I2C_ADDR, (uint8_t)6) != 6) return false;
  while (Wire.available()) (void)Wire.read();     // töm buffert
  return true;
}
/*
float testPa = 0.0f;
if (te_readPressurePa(testPa)) {
  Serial.print("TE pressure OK, Pa = ");
  Serial.println(testPa);
} else {
  Serial.println("TE read failed (setup check).");
}*/

bool te_readInt16(uint8_t reg, int16_t &val) {
  Wire.beginTransmission(TE_PRESSURE_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  if (Wire.requestFrom(TE_PRESSURE_I2C_ADDR, (uint8_t)2) != 2) return false;
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  val = (int16_t)((hi << 8) | lo);
  return true;
}

// Läs tryck i Pascal (Pa) via transferfunktionen i databladet
bool te_readPressurePa(float &p_pa) {
  int16_t counts = 0;
  if (!te_readInt16(TE_REG_DSP_S, counts)) return false;
  p_pa = TE_PMIN + ( ((float)counts - (float)TE_OUTMIN) *
                     (TE_PMAX - TE_PMIN) /
                     ((float)TE_OUTMAX - (float)TE_OUTMIN) );
  return true;
}

bool te_autozero(uint16_t duration_ms = 800) {
  // Mäter i stillhet (ingen andning) och sätter pressureOffset till medelvärdet.
  uint32_t t0 = millis();
  uint16_t n = 0;
  double acc = 0.0;
  while (millis() - t0 < duration_ms) {
    float p;
    if (te_readPressurePa(p)) {
      acc += p;
      n++;
    }
    delay(5);
  }
  if (n < 5) return false;   // för få prover
  pressureOffset = (float)(acc / n);
  return true;
}


//----------------------------------------------------------------------------------------------------------
//                  SETUP
//----------------------------------------------------------------------------------------------------------
void setup()
{
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  //defines ADC characteristics for battery voltage
  /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
  */

  gadgetBle.begin();  //uncomment to activate Sensirion App
  Serial.println(gadgetBle.getDeviceIdString()); //uncomment for Sensirion App
  //gadgetBle.writeCO2(1);
  //gadgetBle.writeTemperature(1);
  //gadgetBle.writeHumidity(1);
  //gadgetBle.commit(); 

  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    //Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    //Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    //Serial.println("Default Vref: 1100mV");
  }

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  readVoltage();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("SpiroVO2max", 0, 10, 4);
  tft.drawString(Version, 0, 40, 4);
  tft.drawString("Initialising...", 0, 70, 4);

  if (!digitalRead(buttonPin2)) { // DEMO Mode if button2 is pressed during power on
    DEMO = 1;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("DEMO-MODE!", 0, 100, 4);
  }
  delay (3000);

  tft.fillScreen(TFT_BLACK);

  Wire.begin(21,22);
  Wire.setClock(100000); // 100 kHz är säkrast till att börja med
    delay(50);
  Serial.begin(115200);

  if (!Serial) {
    tft.drawString("Serial ERROR!", 0, 0, 4);
  }
  else {
    tft.drawString("Serial ok", 0, 0, 4);
  }

    i2cScan();

  if (!SerialBT.begin("SpiroVO2max")) { // Start Bluetooth with device name
    tft.drawString("BT NOT ready!", 0, 18, 4);
  }
  else {
    tft.drawString("BT ready", 0, 18, 4);
  }

  if (!Oxygen.begin(Oxygen_IICAddress)) {
    tft.drawString("O2-Sensor ERROR!", 0, 36, 4);
  }
  else {
    tft.drawString("O2-Sensor ok", 0, 36, 4);
  }
  // Init CO2 (STC31)
    stc_ok = co2Sensor.begin(Wire);
    Serial.printf("CO2 sensor: %s (addr 0x%02X)\n", stc_ok ? "OK" : "FAILED", co2Sensor.address());

    if (!te_begin()) {
     tft.drawString("Flow-Sensor ERROR!", 0, 54, 4);
    } else {
    tft.drawString("Flow-Sensor ok", 0, 54, 4);
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Zeroing flow...", 0, 72, 4);
      if (te_autozero(1000)) {
        tft.drawString("Zero OK         ", 0, 72, 4);
      } else {
        tft.drawString("Zero FAILED     ", 0, 72, 4);
      }

// --- Visa idle-dP (bara vid uppstart) i ~3 sekunder ---
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.fillRect(0, 90, 240, 22, TFT_BLACK);    // rensa ytan högre upp
    tft.drawString("Idle dP (Pa):", 0, 96, 4);  // flytta etikett

    uint32_t tEnd = millis() + 3000;
      while (millis() < tEnd) {
    float p_raw;
      if (te_readPressurePa(p_raw)) {
        float dp = pressureSign * (p_raw - pressureOffset);
        tft.fillRect(150, 90, 90, 22, TFT_BLACK);   // rensa sifferrutan
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.drawFloat(dp, 2, 150, 96, 4);           // flytta värdet
      }
  delay(120);
      }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

  //Serial.println("Flow-Sensor I2c connect success!");
  delay (2000);

  InitBLE(); // init BLE for transmitting VO2 as heartrate
  bpm = 30; // initial test value

  initialO2 = Oxygen.ReadOxygenData(COLLECT_NUMBER); //read and check initial VO2%
  if (initialO2 < 20.00) {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(5, 5, 4);
    tft.println("INITIAL VO2% LOW!");
    tft.setCursor(5, 30, 4);
    tft.println("Wait to continue!");
    while (digitalRead(buttonPin1)) {
      initialO2 = Oxygen.ReadOxygenData(COLLECT_NUMBER);
      tft.setCursor(5, 67, 4);
      tft.println("VO2%: ");
      tft.setCursor(120, 67, 4);
      tft.println(initialO2);
      tft.setCursor(5, 105, 4);
      tft.println("Continue              >>>");
      delay (500);
    }
    if (initialO2 < 20.00) initialO2 = 20.90;
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.println("Initial VO2% set to:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(100, 55, 4);
    tft.println(initialO2);
    delay (5000);
  }

  GetWeightkg();// Enter weight
  AirDensity();
  tftParameters();// show initial sensor parameters
  while (digitalRead(buttonPin2)) { //wait until button2 is pressed
    tft.setCursor(220, 5, 4);
    tft.print(">");
    delay (500);
    tft.setCursor(220, 5, 4);
    tft.print("    ");
    delay (500);
  }

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, 5, 4);
  tft.println("Use 3L calib.pump");
  tft.setCursor(0, 30, 4);
  tft.println("for sensor check.");
  tft.setCursor(0, 105, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Press to start      >>>");

  while (digitalRead(buttonPin1)); // Start measurement ---------

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  TimerVolCalc = millis(); //timer for the volume (VE) integral function
  Timer5s = millis();
  Timer1min = millis();
  TimerVO2calc = millis(); //timer between VO2max calculations
  TimerStart = millis(); // holds the millis at start
  TotalTime = 0;
  //BatteryBT(); // TEST for battery discharge log ++++++++++++++++++++++++++++++++++++++++++++
}

  void te_debug_once() {
  Serial.println("\n[TE debug] read 6 bytes @0x2E:");
  Wire.beginTransmission(TE_PRESSURE_I2C_ADDR);
  Wire.write(0x2E);
  uint8_t e = Wire.endTransmission(false);
  Serial.printf("endTransmission(false) -> %u\n", e);
  int n = Wire.requestFrom(TE_PRESSURE_I2C_ADDR, (uint8_t)6);
  Serial.printf("requestFrom -> %d bytes\n", n);
  while (Wire.available()) Serial.printf(" %02X", Wire.read());
  Serial.println();
}

//----------------------------------------------------------------------------------------------------------
//                  MAIN PROGRAM
//----------------------------------------------------------------------------------------------------------

void loop()
{
  TotalTime = millis() - TimerStart; // calculates actual total time
  VolumeCalc(); //Starts integral function

  // VO2max calculation, tft display and excel csv every 5s --------------
  if ((millis() - TimerVO2calc) > 5000 && fabs(pressure) < 0.2) { // calls vo2maxCalc() for calculation Vo2Max every 5 seconds.
    TimerVO2diff = millis() - TimerVO2calc;
    TimerVO2calc = millis();  //resets the timer
    vo2maxCalc();  //vo2 max function call
    //goFigure();  //vo2 max function call
    if (TotalTime >= 10000) {
      showScreen();
      volumeTotal2 = 0;  //resets volume2 to 0 (used for initial 10s sensor test)
      readVoltage();
    }
    gadgetBle.handleEvents();
    ExcelStream(); //send csv data via wired com port
    ExcelStreamBT(); //send csv data via Bluetooth com port ++++++++++++++++++++++++++++++++++

    // send BLE data ----------------

    bpm = int(vo2Max + 0.5);
    heart[1] = (byte)bpm;

    int energyUsed = calTotal * 4.184; // conversion kcal into kJ
    heart[3] = energyUsed / 256;
    heart[2] = energyUsed - (heart[3] * 256);

    // Serial.println(bpm);
    // Serial.println(energyUsed);
    delay (100);

    heartRateMeasurementCharacteristics.setValue(heart, 8); //set the new value for heartrate
    heartRateMeasurementCharacteristics.notify(); // send a notification that value has changed

    sensorPositionCharacteristic.setValue(hrmPos, 1);
    // bpm++; // TEST only

    // ------------------------------

      gadgetBle.handleEvents();


  }


  if (TotalTime >= 10000) {// after 10 sec. activate the buttons for switching the screens
    ReadButtons();
    if (buttonPushCounter1 > 20 && buttonPushCounter2 > 20) ESP.restart();
    if (buttonPushCounter1 == 2) {
      screenNr--;
      screenChanged = 1;
    }
    if (buttonPushCounter2 == 2) {
      screenNr++;
      screenChanged = 1;
    }
    if (screenNr < 1) screenNr = 5;
    if (screenNr > 5) screenNr = 1;
    if (screenChanged == 1) {
      showScreen();
      screenChanged = 0;
    }
  }

  if (millis() - Timer1min > 30000) {
    Timer1min = millis(); // reset timer
    //BatteryBT(); //TEST für battery discharge log ++++++++++++++++++++++++++++++++++++++++++
  }

  TimerVolCalc = millis(); //part of the integral function to keep calculation volume over time
  // Resets amount of time between calcs
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 1000) {    // skicka till appen var 1 sekund
      gadgetBle.writeCO2(volumeVEmean);    // visning: andningsvolym / ventilation
      gadgetBle.writeTemperature(vo2Max);  // visning: VO₂-värde
      gadgetBle.writeHumidity(lastO2);     // visning: O₂-halt från sensorn
      gadgetBle.commit();                  // skicka paketet till appen
      lastUpdate = millis();
    }

        // --- Skicka till MyAmbience var ~1 s ---
      static uint32_t lastBle = 0;
      if (millis() - lastBle >= 1000) {

        // --- Uppdatera mätdata innan skick ---
        ReadO2();
        vo2maxCalc();

        // Skydda mot NaN och negativa värden (ingen övre begränsning)
        float tempLike   = isnanf(vo2Max)   ? 0.0f : vo2Max;      // visas som "Temperature"
        float humLike    = isnanf(lastO2)   ? 0.0f : lastO2;      // O2% -> "Humidity"
        int   co2LikePpm = (int)lroundf( isnanf(vo2Total) ? 0.0f : (vo2Total < 0.0f ? 0.0f : vo2Total) );

        // --- Skicka till appen ---
        gadgetBle.writeTemperature(tempLike);
        gadgetBle.writeHumidity(humLike);
        gadgetBle.writeCO2(co2LikePpm);
        gadgetBle.commit();

        lastBle = millis();
      }

// Kör BLE-händelser varje varv
    gadgetBle.handleEvents();               // låt BLE-stacken göra jobbet
}

//----------------------------------------------------------------------------------------------------------
//                  FUNCTIONS
//----------------------------------------------------------------------------------------------------------

void ConvertTime (float ms) {
  long inms = long(ms);
  int h, m, s;
  String strh, strm, strs;
  s = (inms / 1000) % 60;
  m  = (inms / 60000) % 60;
  h  = (inms / 3600000) % 24;
  strs = String(s);
  if (s < 10) strs = String("0") + strs;
  strm = String(m);
  if (m < 10) strm = String("0") + strm;
  strh = String(h);
  if (h < 10) strh = String("0") + strh;
  TotalTimeMin = String(strh) + String(":") + String(strm) + String(":") + String(strs);
}

//--------------------------------------------------

void showScreen() { // select active screen
  ConvertTime (TotalTime);
  tft.setRotation(1);
  switch (screenNr) {
    case 1:
      tftScreen1();
      break;
    case 2:
      tftScreen2();
      break;
    case 3:
      tft.setRotation(2);
      tftScreen3();
      break;
    case 4:
      tft.setRotation(0);
      tftScreen3();
      break;
    case 5:
      tftParameters();
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

//--------------------------------------------------

void VolumeCalc() {

// Read pressure from TE SM9xxx
  float pressureraw = NAN;
    if (te_readPressurePa(pressureraw)) {
  // Ta bort offset och sätt riktning (inandning/utandning)
  float dp = pressureSign * (pressureraw - pressureOffset);

  // Low-pass filtrering (IIR). Justera alpha 0.85–0.95 vid behov.
  const float alpha = 0.90f;
    if (isnan(pressure)) pressure = dp;  // init första gången
    else pressure = alpha * pressure + (1.0f - alpha) * dp;
    } else {
      pressure = NAN;  // triggar felhantering längre ned
    }

  if (DEMO == 1) {
    pressure = 16; // TEST+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ((millis() - TimerVO2calc) > 2500) pressure = 0; // TEST++++++++++++++++++++++++++++
  }

  if (TotalTime < 10000) { // initial check of volume sensor
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(0, 5, 4);
    tft.println("Total Volume (ml):");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(0, 55, 7);
    tft.println(volumeTotal2, 0);
    tft.setCursor(0, 105, 4);
    tft.print(expiratVol, 3);
    tft.setCursor(100, 105, 4);
    tft.print(TotalTime / 1000, 1);
    //tft.setCursor(170, 105, 4);
    //tft.println(pressure, 1);
  }

  if (isnan(pressure)) { //isnan = is not a number,  unvalid sensor data
    tft.fillScreen(TFT_RED);    
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("VENTURI ERROR!", 120, 55, 4);
  }
  if (fabs(pressure) > 260) { // upper limit of flow sensor warning
    //tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("SENSOR LIMIT!", 120, 55, 4);
  }
  //if (pressure < 0) pressure = 0;

  if (fabs(pressure) < 0.2 && readVE == 1) { // read volumeVE
    readVE = 0;
    DurationVE = millis() - TimerVE;
    TimerVE = millis(); // start timerVE
    volumeExp = volumeTotal;
    volumeTotal = 0; // resets volume for next breath
    volumeVE = volumeExp / DurationVE * 60;
    volumeExp = volumeExp / 1000;
    volumeVEmean = (volumeVEmean * 3 / 4) + (volumeVE / 4); // running mean of one minute volume (VE)
    if (volumeVEmean < 1) volumeVEmean = 0;
    freqVE = 60000 / DurationVE;
    if (volumeVE < 0.1) freqVE = 0;
    freqVEmean = (freqVEmean * 3 / 4) + (freqVE / 4);
    if (freqVEmean < 1) freqVEmean = 0;

    /*
        Serial.print("volumeExp: ");
        Serial.print(volumeExp);
        Serial.print("   VE: ");
        Serial.print(volumeVE);
        Serial.print("   VEmean: ");
        Serial.print(volumeVEmean);
        Serial.print("   freqVE: ");
        Serial.print(freqVE, 1);
        Serial.print("   freqVEmean: ");
        Serial.println(freqVEmean, 1);
    */
  }
  if (millis() - TimerVE > 5000) readVE = 1; // readVE at least every 5s

  if (pressure >= 0.2) { // ongoing integral of volumeTotal
    if (volumeTotal > 50) readVE = 1;
    massFlow = 1000 * sqrt((abs(pressure) * 2 * rho) / ((1 / (pow(area_2, 2))) - (1 / (pow(area_1, 2))))); //Bernoulli equation
    volFlow = massFlow / rho; //volumetric flow of air
    volFlow = volFlow * correctionSensor; // correction of sensor calculations
    volumeTotal = volFlow * (millis() - TimerVolCalc) + volumeTotal;
    volumeTotal2 = volFlow * (millis() - TimerVolCalc) + volumeTotal2;
  }
  else if ((volumeTotal2 - volumeTotalOld) > 200) { //calculate actual expiratory volume
    expiratVol = (volumeTotal2 - volumeTotalOld) / 1000;
    volumeTotalOld = volumeTotal2;
  }

}


//--------------------------------------------------
void ExcelStream() {
  //HeaderStreamed = 1;// TEST: Deactivation of header
  if (HeaderStreamed == 0) {
    Serial.print("Time");
    Serial.print(",");
    Serial.print("VO2");
    Serial.print(",");
    Serial.print("VO2MAX");
    Serial.print(",");
    Serial.print("VO2total");
    Serial.print(",");
    Serial.print("kcal");
    Serial.print(",");
    Serial.print("Bvol");
    Serial.print(",");
    Serial.print("VEmin");
    Serial.print(",");
    Serial.print("Brate");
    Serial.print(",");
    Serial.print("outO2%");
    Serial.print(",");
    Serial.println("inO2%");
    HeaderStreamed = 1;
  }
  Serial.print(float(TotalTime / 1000), 0);
  Serial.print(",");
  Serial.print(vo2Max);
  Serial.print(",");
  Serial.print(vo2MaxMax);
  Serial.print(",");
  Serial.print(vo2Total);
  Serial.print(",");
  Serial.print(calTotal);
  Serial.print(",");
  Serial.print(volumeExp);
  Serial.print(",");
  Serial.print(volumeVEmean);
  Serial.print(",");
  Serial.print(freqVEmean);
  Serial.print(",");
  Serial.print(lastO2);
  Serial.print(",");
  Serial.println(initialO2);
}
//--------------------------------------------------
void ExcelStreamBT() {
  //HeaderStreamedBT = 1;// TEST: Deactivation of header
  if (HeaderStreamedBT == 0) {
    SerialBT.print("Time");
    SerialBT.print(",");
    SerialBT.print("VO2");
    SerialBT.print(",");
    SerialBT.print("VO2MAX");
    SerialBT.print(",");
    SerialBT.print("VO2total");
    SerialBT.print(",");
    SerialBT.print("kcal");
    SerialBT.print(",");
    SerialBT.print("Bvol");
    SerialBT.print(",");
    SerialBT.print("VEmin");
    SerialBT.print(",");
    SerialBT.print("Brate");
    SerialBT.print(",");
    SerialBT.print("outO2%");
    SerialBT.print(",");
    SerialBT.println("inO2%");
    HeaderStreamedBT = 1;
  }
  SerialBT.print(float(TotalTime / 1000), 0);
  SerialBT.print(",");
  SerialBT.print(vo2Max);
  SerialBT.print(",");
  SerialBT.print(vo2MaxMax);
  SerialBT.print(",");
  SerialBT.print(vo2Total);
  SerialBT.print(",");
  SerialBT.print(calTotal);
  SerialBT.print(",");
  SerialBT.print(volumeExp);
  SerialBT.print(",");
  SerialBT.print(volumeVEmean);
  SerialBT.print(",");
  SerialBT.print(freqVEmean);
  SerialBT.print(",");
  SerialBT.print(lastO2);
  SerialBT.print(",");
  SerialBT.println(initialO2);
}

//--------------------------------------------------

void BatteryBT() {
  //HeaderStreamedBT = 1;// TEST: Deactivation of header
  if (HeaderStreamedBT == 0) {
    SerialBT.print("Time");
    SerialBT.print(",");
    SerialBT.println("Voltage");
    HeaderStreamedBT = 1;
  }
  SerialBT.print(float(TotalTime / 1000), 0);
  SerialBT.print(",");
  SerialBT.println(Battery_Voltage);
}

//--------------------------------------------------

void ReadO2() {
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER);
  lastO2 = oxygenData;
  if (lastO2 > initialO2) initialO2 = lastO2; // correction for drift of O2 sensor

  if (DEMO == 1) lastO2 = initialO2 - 4; //TEST+++++++++++++++++++++++++++++++++++++++++++++
  co2 = initialO2 - lastO2;
}

//--------------------------------------------------

void AirDensity () { // calculation for dry air
  // Använd befintliga variabler som fallback:
  // TempC är initierad till 15.0 (rad ~162) och
  // PresPa är initierad till 101325 Pa (rad ~163).
  // Justera TempC i koden om du har en annan tempkälla.
  float tAmb = TempC;   // t.ex. 15.0 °C initialt
 float pAmb = PresPa;  // t.ex. 101325 Pa initialt

  rho     = pAmb / (tAmb + 273.15f) / 287.058f;   // torr luft
  rhoBTPS = pAmb / (35.0f + 273.15f) / 292.9f;    // BTPS: 35°C, 95% RH
}

//--------------------------------------------------

void vo2maxCalc() { //V02max calculation every 5s
  ReadO2();
  AirDensity();// calculates air density

  co2 = initialO2 - lastO2;  //calculated level of CO2 based on Oxygen level loss

  // --- Läs CO2 från STC31 och räkna VCO2 & RER ---
    float c, t;
    if (stc_ok && co2Sensor.readCO2(c, t)) {
      co2Pct   = c;   // vol%
      stcTempC = t;   // °C
    }

    // VE i STPD (samma korr som du använder för VO2)
    float VE_stpd = volumeVEmean * rhoBTPS / rhoSTPD; // L/min (STPD)

    // VCO2 (ml/min) ≈ VE_stpd * ((FeCO2 - FiCO2)/100) * 1000
    if (!isnan(co2Pct)) vco2Total = VE_stpd * ((co2Pct - FI_CO2) / 100.0f) * 1000.0f;
    else                vco2Total = NAN;

    // Din VO2-beräkning finns redan nedan:
    // vo2Total = volumeVEmean * rhoBTPS / rhoSTPD * co2 * 10;  // (ml/min)

    // RER = VCO2 / VO2
    if (vo2Total > 1e-3 && !isnan(vco2Total)) RER = vco2Total / vo2Total;
    else                                       RER = NAN;

  if (co2 < 0) co2 = 0; // correction for sensor drift

  vo2Total = volumeVEmean * rhoBTPS / rhoSTPD * co2 * 10; // = vo2 in ml/min (* co2% * 10 for L in ml)
  vo2Max = vo2Total / weightkg; //correction for wt
  if (vo2Max > vo2MaxMax) vo2MaxMax = vo2Max;

  vo2Cal = vo2Total / 1000 * 4.86; //vo2Max liters/min * 4.86 Kcal/liter = kcal/min
  calTotal = calTotal + vo2Cal * TimerVO2diff / 60000; // integral function of calories
  vo2CalDay = vo2Cal * 1440.0; // actual calories/min. * 1440 min. = cal./day
  if (vo2CalDay > vo2CalDayMax) vo2CalDayMax = vo2CalDay;
}

//--------------------------------------------------------
void tftScreen1() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(5, 5, 4);
  tft.print("Time  ");
  tft.setCursor(120, 5, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println(TotalTimeMin);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.setCursor(5, 30, 4);
  tft.print("VO2 ");
  tft.setCursor(120, 30, 4);
  tft.println(vo2Max);

  tft.setCursor(5, 55, 4);
  tft.print("VO2MAX ");
  tft.setCursor(120, 55, 4);
  tft.println(vo2MaxMax);

  tft.setCursor(5, 80, 4);
  tft.print("VO2total ");
  tft.setCursor(120, 80, 4);
  tft.println(vo2Total, 0);

  tft.setCursor(5, 105, 4);
  tft.print("kcal ");
  tft.setCursor(120, 105, 4);
  tft.println(calTotal, 0);

}
//--------------------------------------------------------
//--------------------------------------------------------
void tftScreen2() {

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(5, 5, 4);
  tft.print("Time  ");
  tft.setCursor(120, 5, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println(TotalTimeMin);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.setCursor(5, 30, 4);
  tft.print("Bvol ");
  tft.setCursor(120, 30, 4);
  tft.println(volumeExp);

  tft.setCursor(5, 55, 4);
  tft.print("VEmin ");
  tft.setCursor(120, 55, 4);
  tft.println(volumeVEmean, 1);

  tft.setCursor(5, 80, 4);
  tft.print("Brate ");
  tft.setCursor(120, 80, 4);
  tft.println(freqVEmean, 1);

  tft.setCursor(5, 105, 4);
  tft.print("outO2% ");
  tft.setCursor(120, 105, 4);
  tft.println(lastO2);
}


//--------------------------------------------------------
void tftScreen3() {

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(5, 5, 4);
  tft.print("Time  ");
  tft.setCursor(5, 30, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println(TotalTimeMin);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.setCursor(5, 55, 4);
  tft.print("VO2 ");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 80, 7);
  tft.println(vo2Max, 1);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(5, 130, 4);   // placera lite längre ner på skärmen
  tft.print("dP (Pa): ");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print(pressure, 2);     // visar t.ex. 1.23 Pa

}

//--------------------------------------------------------



//--------------------------------------------------------
void tftParameters() {

  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);

  tft.setCursor(5, 5, 4);
  tft.print("*C");
  tft.setCursor(120, 5, 4);
  tft.println(TempC, 1);

  tft.setCursor(5, 30, 4);
  tft.print("hPA");
  tft.setCursor(120, 30, 4);
  tft.println((PresPa / 100));

  tft.setCursor(5, 55, 4);
  tft.print("kg/m3");
  tft.setCursor(120, 55, 4);
  tft.println(rho, 4);

  tft.setCursor(5, 80, 4);
  tft.print("KG");
  tft.setCursor(120, 80, 4);
  tft.println(weightkg, 1);

  tft.setCursor(5, 105, 4);
  tft.print("O2%");
  tft.setCursor(120, 105, 4);
  tft.println(initialO2);

}

//--------------------------------------------------------
void ReadButtons() {
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  if (buttonState1 == LOW) {
    buttonPushCounter1++;
  }
  else {
    buttonPushCounter1 = 0;
  }
  if (buttonState2 == LOW) {
    buttonPushCounter2++;
  }
  else {
    buttonPushCounter2 = 0;
  }
}
//---------------------------------------------------------

void GetWeightkg() {

  Timer5s = millis();
  int weightChanged = 0;
  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.drawString("Enter weight in kg", 20, 10, 4);
  tft.drawString(String (weightkg), 48, 48, 7);

  while ((millis() - Timer5s) < 5000) {
    ReadButtons();

    if (buttonPushCounter1 > 0) {
      weightkg = weightkg - 0.5;
      if (buttonPushCounter1 > 8) weightkg = weightkg - 1.5;
      weightChanged = 1;
    }

    if (buttonPushCounter2 > 0) {
      weightkg = weightkg + 0.5;
      if (buttonPushCounter2 > 8) weightkg = weightkg + 1.5;
      weightChanged = 1;
    }

    if (weightkg < 20) weightkg = 20;
    if (weightkg > 200) weightkg = 200;
    if (weightChanged > 0) {
      tft.fillScreen(TFT_BLUE);
      tft.drawString("New weight in kg is:", 10, 10, 4);
      tft.drawString(String (weightkg), 48, 48, 7);
      weightChanged = 0;
      Timer5s = millis();
    }
    delay(200);
  }
}

//---------------------------------------------------------

void readVoltage() {
  uint16_t v = analogRead(ADC_PIN);
  Battery_Voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  if (Battery_Voltage >= 4.3) tft.setTextColor(TFT_BLACK, TFT_WHITE); // USB powered, charging
  if (Battery_Voltage < 4.3) tft.setTextColor(TFT_BLACK, TFT_GREEN); // battery full
  if (Battery_Voltage < 3.9) tft.setTextColor(TFT_BLACK, TFT_YELLOW); // battery half
  if (Battery_Voltage < 3.7) tft.setTextColor(TFT_WHITE, TFT_RED); // battery critical
  tft.setCursor(0, 0, 4);
  tft.print(String(Battery_Voltage) + "V");
}

//---------------------------------------------------------

void InitBLE() {
  BLEDevice::init("SpiroVO2-HR"); //creates the device name

  // (1) Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer(); //creates the BLE server
  pServer->setCallbacks(new MyServerCallbacks()); //creates the server callback function

  // (2) Create the BLE Service "heartRateService"
  BLEService *pHeart = pServer->createService(heartRateService); //creates heatrate service with 0x180D

  // (3) Create the characteristics, descriptor, notification
  pHeart->addCharacteristic(&heartRateMeasurementCharacteristics); //creates heartrate characteristics 0x2837
  heartRateDescriptor.setValue("Rate from 0 to 200"); //describtion of the characteristic
  heartRateMeasurementCharacteristics.addDescriptor(&heartRateDescriptor);
  heartRateMeasurementCharacteristics.addDescriptor(new BLE2902()); //necessary for notifications
  //client switches server notifications on/off via BLE2902 protocol

  // (4) Create additional characteristics
  pHeart->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  // (5) define advertising the heartRateService
  pServer->getAdvertising()->addServiceUUID(heartRateService);

  // (6) start the server and the advertising
  pHeart->start();
  pServer->getAdvertising()->start();  // Start advertising
}

//---------------------------------------------------------

void goFigure(){
  TimerVO2calc = TotalTime + 0.5;
  float percentN2exp;
  float co2 = 20.9 - lastO2;  //this is the calculated level of Co2 based on Oxygen level loss
  float volumeMinute = volumeTotal * 2.0;  //doubles the air volume to represent 1 minute of breathing'
  
  volumeMinute = volumeMinute/1000.0; //gives liters of air VE
  Serial.print("liters/min uncorrected");
  Serial.print(volumeMinute);
  //co2 = lastO2/10000.0;
  //lastO2 = 17.5;
  Serial.print("CO2  ");
  Serial.print(co2);
  Serial.print("02  ");
  Serial.println(lastO2);
  percentN2exp = (100.0 - (co2 + lastO2));  //calculation expresses vol of expired nitrogen
  volumeMinute = volumeMinute * 0.852;  //Calculation for dry air at room temp
  vo2Max = volumeMinute * (((percentN2exp/100.0) * 0.265) - (lastO2/100.0)); //Vo2 max calculation
  Serial.print("VO2Max!  ");
  vo2Cal = vo2Max * 4.86; //this is vo2Max in liters/min multiplied by factor of Kcal/liter giving kcal/min
  calTotal = calTotal + vo2Cal/2.0;
  vo2MaxMax = vo2Cal * 1440.0;
  Serial.print(vo2Max);
  vo2Max = (vo2Max * 1000.0)/(float(wtTotal)/2.2); //correcion for wt and changed to CC Final vo2 max 
  //This will broadcast the data to Sensirion App:  Uncomment all to activate!
 
  
  if(vo2Max > vo2MaxMax) vo2MaxMax = vo2Max;
  
  Serial.print("vo2Cal  ");
  Serial.print(vo2Cal); 
  Serial.print("vo2Cal in 24 hours=   ");
  Serial.println(calTotal);
  //Uncomment all to broadcast datat to Sensirion App
  if(TimerVO2calc == 0.5) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Orange
    tft.setCursor(60, 40, 7);
  int puff = volumeTotal; 
    tft.println(puff); 
  }else {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK); 
  tft.setCursor(5, 5, 4);
  tft.print("Time  ");
  tft.setCursor(120, 5, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK); 
  tft.println(TimerVO2calc);
  int stopper = TimerVO2calc * 10;
  if(stopper % 2){
  tft.setCursor(5, 35, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK); 
  tft.print("VO2MAX ");
  tft.setCursor(120, 35, 4);
  tft.println(vo2MaxMax);
  tft.setCursor(5, 65, 4);
  tft.print("KCalMax ");
  tft.setCursor(120, 65, 4);
  tft.println(vo2CalDayMax);
  tft.print("Kcal/Day");
  tft.setCursor(120, 95, 4);
  tft.println(vo2MaxMax);
  }else{
  tft.setCursor(5, 35, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK); 
  tft.print("O2  ");
  tft.setCursor(120, 35, 4);
  tft.println(lastO2);
  tft.setCursor(5, 65, 4);
  tft.print("VO2  ");
  tft.setCursor(120, 65, 4);
  tft.println(vo2Max);
  tft.setCursor(5, 95, 4);  
  tft.print("Cals  ");
  tft.setCursor(120, 95, 4);
  tft.println(calTotal);
  }
  }
  volumeMinute = volumeMinute * 1000;
   if(volumeMinute < 10) {
    lastO2 = 1;
    vo2Max = 1;
    volumeMinute = 1;
  }
  
  gadgetBle.writeCO2(volumeMinute);
  gadgetBle.writeTemperature(vo2Max);
  gadgetBle.writeHumidity(lastO2);

  Serial.print("BLE update -> CO2: ");
  Serial.print(volumeMinute);
  Serial.print(" | Temp: ");
  Serial.print(vo2Max);
  Serial.print(" | Humidity: ");
  Serial.println(lastO2);

  gadgetBle.commit();
 
   delay(3);
   
  
}
