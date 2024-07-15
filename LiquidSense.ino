#include <Arduino.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Adafruit_ADS1X15.h>
#include "EEPROM.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

#define BUTTON_PIN 35

// Insert your network credentials
#define WIFI_SSID "Redmi 10"
#define WIFI_PASSWORD "123456789d"

// Insert Firebase project API Key
#define API_KEY "AIzaSyAKBYejSVLzlMmzpVaup-cS4uACOQ52Z30"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://liquid-sense-f601f-default-rtdb.asia-southeast1.firebasedatabase.app/"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int allcount = 0, count = 0;
bool signupOK = false;

// float phValue2, turbValue2, tdsValue2, tempValue2;
// int day2, month2, year2;
// double lat2, lon2;

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define GPS_BAUDRATE 9600  

TinyGPSPlus gps;  // the TinyGPS++ object

DFRobot_ESP_PH_WITH_ADC ph;
Adafruit_ADS1115 ads;

LiquidCrystal_I2C lcd(0x27, 16, 4);

// #define TdsSensorPin 27
#define VREF 6.144  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30    // sum of sample point

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
int lineNumber = 1;
//Wifi retry counter
int x;

//Turbidity Sensor
float volt;
float ntu;
float adc2 = 0;

int16_t adc0, adc1;

float voltage, phValue, temp;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;  // current temperature for compensation

//bool isPHrunning = false;

char formattedString[100];
char dataString[1000];

float SDpHvalue, SDTURBValue, SDTDSValue, SDTempValue;
int SDday, SDmonth, SDyear;
double SDlat, SDlong;

int valueday = 0, valuemonth = 0, valueyear = 0;
double valuelat = 0.0, valuelong = 0.0;

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  int bytesRead = file.readBytes(dataString, sizeof(dataString) - 1);

  if (bytesRead > 0) {
    dataString[bytesRead] = '\0';  
  } else {
    Serial.println("Failed to read data from file.");
  }

  file.close();

  Serial.print("Read from file: ");
  Serial.println(dataString);  

}

void writeFile(fs::FS &fs, const char *path, const char *message) {

  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

float readTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void setup() {
  Serial.begin(115200);
  // pinMode(TdsSensorPin,INPUT);
  Serial2.begin(9600, SERIAL_8N1, 25, 26);  // GPS serial
  EEPROM.begin(32);  //needed EEPROM.begin to store calibration k in eeprom
  ph.begin();
  sensors.begin();

  lcd.begin();
  lcd.backlight();


  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }


  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  ads.setGain(GAIN_TWOTHIRDS);

  pinMode(BUTTON_PIN, INPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
    x++;
    if (x > 10) {
      break;
    }
  }
  Serial.println("Wifi did not connect");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    firebaseConnect();
  }
}

void loop() {
  // if (isPHrunning == false) {

  //   digitalWrite(27, HIGH);  // turn on relay and connect TDS probe wires
  //   adc0 = ads.readADC_SingleEnded(0);

  // static unsigned long analogSampleTimepoint = millis();
  // if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
  //   analogSampleTimepoint = millis();
  //   analogBuffer[analogBufferIndex] = adc0;  //read the analog value and store into the buffer
  //   analogBufferIndex++;
  //   if (analogBufferIndex == SCOUNT) {
  //     analogBufferIndex = 0;
  //   }
  // }

  // static unsigned long printTimepoint = millis();
  // if (millis() - printTimepoint > 800U) {
  //   printTimepoint = millis();
  //   for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
  //     analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

  //     // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  //     averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 65536.0;

  //     //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  //     float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  //     //temperature compensation
  //     float compensationVoltage = averageVoltage / compensationCoefficient;

  //     //convert voltage value to tds value
  //     tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
  //   }
  //   Serial.print("TDS Value:");
  //   Serial.print(tdsValue, 0);
  //   Serial.println("ppm");
  //   Serial.println(ads.computeVolts(adc0));
  // }

  // } else {

  //   digitalWrite(27, LOW);  // turn off relay and disconnect TDS probe wires
  // }
  // TDSsense();

  adc0 = ads.readADC_SingleEnded(0);

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = adc0;  //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 65536.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
    }
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
    // Serial.println(ads.computeVolts(adc0));
  }

  PHsense();

  ph.calibration(voltage, temperature);

  TurbSense();
  DisplayLCD();

  GPScall();

  byte buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW) {
    Serial.println("Button is pressed");
    Serial.println("Data Saved");
    count++;
    SaveDataToSD();
  } else {
    Serial.println("Button is not pressed");
  }
  // delay(100);

  // SaveDataToSD();

  if (WiFi.status() == WL_CONNECTED) {
    ReadDataToVariables();
  }

  
}

void PHsense() {

  static unsigned long timepoint = millis();
  if (millis() - timepoint > 2000U) 
  {
    //isPHrunning = true;
    timepoint = millis();
    voltage = ads.readADC_SingleEnded(1) / 10;  // read the voltage

    temp = readTemperature();  // read your temperature sensor to execute temperature compensation
    Serial.print("temperature:");
    Serial.print(temp, 1);
    Serial.println("^C");

    phValue = ph.readPH(voltage, temp);  // convert voltage to pH with temperature compensation
    Serial.print("pH:");
    Serial.println(phValue, 4);
    isPHrunning = false;
  }
}

void TurbSense() {

  volt = 0;
  for (int i = 0; i < 200; i++) {
    adc2 = ads.readADC_SingleEnded(2);

    volt += (adc2 * 0.1875 * 1.22) / 1000;
  }

  volt = volt / 200;
  volt = round_to_dp(volt, 2);

  if (volt < 2.5) {
    ntu = 3000;
  } else {
    ntu = -1120.4 * sq(volt) + 5742.3 * volt - 4353.8;
    ntu = ntu - 355.28;
  }
  if (ntu<0){
    ntu = 0;
  }
  Serial.println(volt);
  Serial.println(ntu);
}

float round_to_dp(float in_value, int decimal_place) {
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}

void DisplayLCD() {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TURB: ");
  lcd.print(ntu);
  lcd.print("NTU");
  lcd.setCursor(0, 1);
  lcd.print("pH: ");
  lcd.print(phValue);
  lcd.print(" pH");
  lcd.setCursor(-4, 2);
  lcd.print("TDS: ");
  lcd.print(tdsValue);
  lcd.print(" ppm");
  lcd.setCursor(-4, 3);
  lcd.print("TEMP: ");
  lcd.print(temp);
  lcd.print(" ^C");
  //delay(10);
}

void GPScall() {

  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated()) {
      valuelat = gps.location.lat();
      Serial.print("Latitude= ");
      Serial.print(valuelat, 6);
      valuelong = gps.location.lng();
      Serial.print(" Longitude= ");
      Serial.println(valuelong, 6);
      valueday = gps.date.day();
      valuemonth = gps.date.month();
      valueyear = gps.date.year();
      Serial.print("Date= ");
      Serial.print(valueday);
      Serial.print("/");
      Serial.print(valuemonth);
      Serial.print("/");
      Serial.println(valueyear);
    }
  }
}


void SaveDataToSD() {
  SDpHvalue = phValue;
  SDTURBValue = ntu;
  SDTDSValue = tdsValue;
  SDTempValue = temp;
  SDday = valueday;
  SDmonth = valuemonth;
  SDyear = valueyear;
  SDlat = valuelat;
  SDlong = valuelong;

  // Allocate a buffer to hold the formatted string
  sprintf(formattedString, "%.2f, %.2f, %.2f, %.2f, %d, %d, %d, %lf, %lf\n",
          SDpHvalue, SDTURBValue, SDTDSValue, SDTempValue,
          SDday, SDmonth, SDyear, SDlat, SDlong);
  // writeFile(SD, "/sensor_data.txt", formattedString);
  appendFile(SD, "/sensor_data.txt", formattedString);
}

void ReadDataToVariables() {
  readFile(SD, "/sensor_data.txt");

  float phValue2, turbValue2, tdsValue2, tempValue2;
  int day2, month2, year2;
  double lat2, lon2;

  char *line;
  lineNumber = 1;
  line = strtok(dataString, "\n");
  while (line != NULL) {
    sscanf(line, "%f, %f, %f, %f, %d, %d, %d, %lf, %lf",
           &phValue2, &turbValue2, &tdsValue2, &tempValue2,
           &day2, &month2, &year2, &lat2, &lon2);

    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)) {
      sendDataPrevMillis = millis();
      if (Firebase.RTDB.setInt(&fbdo, "/sensor_data/data" + String(allcount) + "/Day", day2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      if (Firebase.RTDB.setInt(&fbdo, "/sensor_data/data" + String(allcount) + "/Year", year2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      if (Firebase.RTDB.setInt(&fbdo, "/sensor_data/data" + String(allcount) + "/Month", month2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }

      if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/data" + String(allcount) + "/pH", phValue2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/data" + String(allcount) + "/Turbidity", turbValue2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/data" + String(allcount) + "/TDS", tdsValue2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      if (Firebase.RTDB.setFloat(&fbdo, "/sensor_data/data" + String(allcount) + "/Temperature", tempValue2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }

      if (Firebase.RTDB.setDouble(&fbdo, "/sensor_data/data" + String(allcount) + "/Latitude", lat2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      if (Firebase.RTDB.setDouble(&fbdo, "/sensor_data/data" + String(allcount) + "/Longitude", lon2)) {
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }

      allcount++;

      line = strtok(NULL, "\n");
      lineNumber++;
    }
    // if (lineNumber > count) {
    //   deleteFile(SD, "/sensor_data.txt");
    //   count = 0;
    // }
  }
  deleteFile(SD, "/sensor_data.txt");
}

void firebaseConnect() {
  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}