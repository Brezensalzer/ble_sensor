//-------------------------------------------------------------------------------
// ble_sensor.ino
//-------------------------------------------------------------------------------
#include <bluefruit.h>

// save power without serial interface...
const boolean DEBUG = false;

// BME280 I2C Sensor
#include <Wire.h>
#include <BMx280I2C.h>
#define I2C_ADDRESS 0x76
//create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMx280I2C bmx280(I2C_ADDRESS);
// define the GPIO pin used as i2c vcc
#define VCC_I2C 10

// ADC Battery Monitoring
const int adcPin = A5;
// float mv_per_lsb = 3600.0F/1024.0F; // 10-bit ADC with 3.6V input range

// BLE Environmental Sensing Service
BLEService        envSvc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);  // 0x181A
BLECharacteristic tempChar = BLECharacteristic(UUID16_CHR_TEMPERATURE);   // 0x2A6E
BLECharacteristic humChar = BLECharacteristic(UUID16_CHR_HUMIDITY);       // 0x2A6F
BLECharacteristic airChar = BLECharacteristic(UUID16_CHR_PRESSURE);       // 0x2A6D

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

//------------------------------------------------------------------------------
uint8_t batteryLevel()
//------------------------------------------------------------------------------
{
  int adcValue = analogRead(adcPin);
  // 1024 = 3.6V, 768 = 2.7V cut off voltage for LiFePO4 batteries
  uint8_t value = map(adcValue, 768, 1024, 0, 100);
  return value;  
}

//------------------------------------------------------------------------------
void setup()
//------------------------------------------------------------------------------
{
  if (DEBUG) //------------------------------------------
  {
    Serial.begin(115200);
    int i = 0;
    while ( !Serial ) 
    {
      i++;
      if (i > 20) break;
      delay(100);   // for nrf52840 with native usb
    }
  }
  else
  {
    Serial.end();
  }
  
  //--- setup BLE ---------------------------------------------------
  if (DEBUG) //------------------------------------------
  {
    Serial.println("ItsyBitsy nRF52840 Environmental Sensor");
    Serial.println("---------------------------------------\n");
  }

  // Initialise the Bluefruit module
  if (DEBUG) { Serial.println("Initialise the ItsyBitsy nRF52840 module"); }
  Bluefruit.begin();

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  if (DEBUG) { Serial.println("Configuring the Device Information Service"); }
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Adafruit ItsyBitsy nRF52840");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  if (DEBUG) { Serial.println("Configuring the Battery Service"); }
  blebas.begin();
  blebas.write(batteryLevel());

  // Setup the Temperature Monitor service using
  // BLEService and BLECharacteristic classes
  if (DEBUG) { Serial.println("Configuring the Temperature Monitor Service"); }
  setupTemp();

  // Setup the advertising packet(s)
  if (DEBUG) { Serial.println("Setting up the advertising payload(s)"); }
  startAdv();

  if (DEBUG) { Serial.println("\nAdvertising"); }
  //--- setup BLE ---------------------------------------------------

  //--- power on i2c vcc --------------------------------------------
  pinMode(VCC_I2C, OUTPUT);
}

//------------------------------------------------------------------------------
void startAdv(void)
//------------------------------------------------------------------------------
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Temperature Service UUID
  Bluefruit.Advertising.addService(envSvc);

  // Include Name
  Bluefruit.setName("Bluefruit");
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

//------------------------------------------------------------------------------
void setupTemp(void)
//------------------------------------------------------------------------------
{
  envSvc.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  tempChar.setProperties(CHR_PROPS_NOTIFY|CHR_PROPS_READ);
  tempChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tempChar.setFixedLen(2);
  tempChar.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  tempChar.begin();
  
  humChar.setProperties(CHR_PROPS_NOTIFY|CHR_PROPS_READ);
  humChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  humChar.setFixedLen(2);
  humChar.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  humChar.begin();
  
  airChar.setProperties(CHR_PROPS_NOTIFY|CHR_PROPS_READ);
  airChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  airChar.setFixedLen(2);
  airChar.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  airChar.begin();
}

//------------------------------------------------------------------------------
void connect_callback(uint16_t conn_handle)
//------------------------------------------------------------------------------
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  if (DEBUG) 
  { 
    Serial.print("Connected to ");
    Serial.println(central_name);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
//------------------------------------------------------------------------------
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
//------------------------------------------------------------------------------
{
  (void) conn_handle;
  (void) reason;

  if (DEBUG) 
  { 
    Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
    Serial.println("Advertising!");
  }
}

//------------------------------------------------------------------------------
void cccd_callback(short unsigned int conn_hdl, BLECharacteristic* chr, short unsigned int cccd_value)
//------------------------------------------------------------------------------
{
    if (DEBUG) 
    { 
      // Display the raw request packet
      Serial.print("CCCD Updated: ");
      //Serial.printBuffer(request->data, request->len);
      Serial.print(cccd_value);
      Serial.println("");
    }
    
    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == tempChar.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            if (DEBUG) { Serial.println("Temperature Measurement 'Notify' enabled"); }
        } else {
            if (DEBUG) { Serial.println("Temperature Measurement 'Notify' disabled"); }
        }
    }
    if (chr->uuid == humChar.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            if (DEBUG) { Serial.println("Humidity Measurement 'Notify' enabled"); }
        } else {
            if (DEBUG) { Serial.println("Humidity Measurement 'Notify' disabled"); }
        }
    }
    if (chr->uuid == airChar.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            if (DEBUG) { Serial.println("Air Pressure Measurement 'Notify' enabled"); }
        } else {
            if (DEBUG) { Serial.println("Air Pressure Measurement 'Notify' disabled"); }
        }
    }
}

//------------------------------------------------------------------------------
void loop()
//------------------------------------------------------------------------------
{
  //--- power on i2c vcc --------------------------------------------
  digitalWrite(VCC_I2C, HIGH);
  if (DEBUG) Serial.println("setting pin VCC_I2C to HIGH");
  delay(10);
  
  //--- setup BME280 sensor -----------------------------------------
  Wire.begin();

  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
  //and reads compensation parameters.
  if (!bmx280.begin())
  {
    if (DEBUG) { Serial.println("begin() failed. check your BMx280 Interface and I2C Address."); }
    while (1);
  }

  if (DEBUG) //------------------------------------------
  {
    if (bmx280.isBME280())
      Serial.println("sensor is a BME280");
    else
      Serial.println("sensor is a BMP280");
  }
  
  //reset sensor to default parameters.
  bmx280.resetToDefaults();

  //by default sensing is disabled and must be enabled by setting a non-zero
  //oversampling setting.
  //set an oversampling setting for pressure and temperature measurements. 
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
  bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

  //if sensor is a BME280, set an oversampling setting for humidity measurements.
  if (bmx280.isBME280())
    bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);
  //--- setup BME280 sensor -----------------------------------------

  if ( Bluefruit.connected() ) {
    digitalWrite(LED_RED, LOW);

    // BME280 sensor wakeup
    bmx280.writePowerMode(BMx280MI::BMx280_MODE_FORCED);

    //--- start a measurement ---------------------------------------------------
    if (!bmx280.measure())
    {
      if (DEBUG) { Serial.println("could not start measurement, is a measurement already running?"); }
      return;
    }
  
    //wait for the measurement to finish
    do
    {
      delay(100);
    } while (!bmx280.hasValue());
    
    float celsius = bmx280.getTemperature();
    int temp = int(celsius * 100.0);
    uint8_t tempdata[2] = { lowByte(temp), highByte(temp) };      // temperature value 
    
    float humidity = bmx280.getHumidity();
    int hum = int(humidity * 100.0);
    uint8_t humdata[2] = { lowByte(hum), highByte(hum) };         // humidity value 

    float pressure = bmx280.getPressure();
    int air = int(pressure/100.0);                                // air pressure value
    uint8_t airdata[2] = { lowByte(air), highByte(air) };

    uint8_t bat = batteryLevel();
    blebas.write(bat);
    blebas.notify(bat);
    if (DEBUG) { Serial.print("Battery Level: "); Serial.println(bat); }

    // Note: We use .notify instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if ( tempChar.notify(tempdata, sizeof(tempdata)) ){
      if (DEBUG) { Serial.print("Temperature Measurement updated to: "); Serial.println(celsius); }
    }else{
      if (DEBUG) { Serial.println("WARN: Notify not set in the CCCD or not connected!"); }
    }

    if ( humChar.notify(humdata, sizeof(humdata)) ){
      if (DEBUG) { Serial.print("Humidity Measurement updated to: "); Serial.println(humidity); }
    }else{
      if (DEBUG) { Serial.println("WARN: Notify not set in the CCCD or not connected!"); }
    }

    if ( airChar.notify(airdata, sizeof(airdata)) ){
      if (DEBUG) { Serial.print("Air Pressure Measurement updated to: "); Serial.println(pressure); }
    }else{
      if (DEBUG) { Serial.println("WARN: Notify not set in the CCCD or not connected!"); }
    }

  } // if ( Bluefruit.connected())

  // BME280 sensor sleep
  bmx280.writePowerMode(BMx280MI::BMx280_MODE_SLEEP);
  
  Wire.end();

  //--- power off i2c vcc --------------------------------------------
  digitalWrite(VCC_I2C, LOW);
  if (DEBUG) Serial.println("setting pin VCC_I2C to LOW");

  delay(30000);  // 30s
}
