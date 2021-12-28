//-------------------------------------------------------------------------------
// ble_sensor.ino
//-------------------------------------------------------------------------------
#include <bluefruit.h>

#include <Wire.h>
#include <BMx280I2C.h>

#define I2C_ADDRESS 0x76

//create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMx280I2C bmx280(I2C_ADDRESS);

// Environmental Sensing Service:  0x181A
// Temperature Celsius Char: 0x2A1F
BLEService        tempSvc = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
BLECharacteristic tempChar = BLECharacteristic(UUID16_CHR_TEMPERATURE_CELSIUS);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

int  temp = 0;

//------------------------------------------------------------------------------
void setup()
//------------------------------------------------------------------------------
{
  Serial.begin(115200);
  int i = 0;
  while ( !Serial ) 
  {
    i++;
    if (i > 20) break;
    delay(100);   // for nrf52840 with native usb
  }

  //--- setup sensor ------------------------------------------------
  Wire.begin();

  //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
  //and reads compensation parameters.
  if (!bmx280.begin())
  {
    Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
    while (1);
  }

  if (bmx280.isBME280())
    Serial.println("sensor is a BME280");
  else
    Serial.println("sensor is a BMP280");

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
  //--- setup sensor ------------------------------------------------


  //--- setup BLE ---------------------------------------------------
  Serial.println("ItsyBitsy nRF52840 Environmental Sensor");
  Serial.println("---------------------------------------\n");

  // Initialise the Bluefruit module
  Serial.println("Initialise the ItsyBitsy nRF52840 module");
  Bluefruit.begin();

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Adafruit ItsyBitsy nRF52840");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Temperature Monitor service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Temperature Monitor Service");
  setupTemp();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");
}

// ------------------------------------------------------------------------------
void startAdv(void)
// ------------------------------------------------------------------------------
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Temperature Service UUID
  Bluefruit.Advertising.addService(tempSvc);

  // Include Name
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

// ------------------------------------------------------------------------------
void setupTemp(void)
// ------------------------------------------------------------------------------
{
  tempSvc.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  tempChar.setProperties(CHR_PROPS_NOTIFY|CHR_PROPS_READ);
  tempChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tempChar.setFixedLen(2);
  tempChar.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  tempChar.begin();
}

// ------------------------------------------------------------------------------
void connect_callback(uint16_t conn_handle)
// ------------------------------------------------------------------------------
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
// ------------------------------------------------------------------------------
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
// ------------------------------------------------------------------------------
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
}

// ------------------------------------------------------------------------------
void cccd_callback(short unsigned int conn_hdl, BLECharacteristic* chr, short unsigned int cccd_value)
// ------------------------------------------------------------------------------
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == tempChar.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("Temperature Measurement 'Notify' enabled");
        } else {
            Serial.println("Temperature Measurement 'Notify' disabled");
        }
    }
}

// ------------------------------------------------------------------------------
void loop()
// ------------------------------------------------------------------------------
{  
  if ( Bluefruit.connected() ) {
    digitalWrite(LED_RED, LOW);

    //--- start a measurement ---------------------------------------------------
    if (!bmx280.measure())
    {
      Serial.println("could not start measurement, is a measurement already running?");
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
    
    blebas.write(98);                                             // simulated battery value

    // Note: We use .notify instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    if ( tempChar.notify(tempdata, sizeof(tempdata)) ){
      Serial.print("Temperature Measurement updated to: "); Serial.println(celsius); 
      Serial.print("Humidity Measurement updated to: "); Serial.println(humidity);
      Serial.print("Air Pressure Measurement updated to: "); Serial.println(pressure);
    }else{
      Serial.println("WARN: Notify not set in the CCCD or not connected!");
    }
  } else {
    digitalWrite(LED_RED, HIGH);
  }

  // Only send update once per 30 seconds
  delay(30000);
}
