# ble_sensor
Bluetooth Low Energy Sensor with Adafruit ItsyBitsy nRF52840 and BME280 module

## Experiments to optimize power consumption.
### Run 1: Indirect current measurement BLE module with sensor module

- Adafruit ItsyBitsy nRF52840 Express development board
- BME280 temperature, humidity and pressure sensor via I2C
- Measurement interval: 60s
- Power supply: LiFePO4 battery in AA-Format with 550 mAh, fully charged

**Start time: 05.01.2022, 21:39**
**measured battery level: 70%**

**Stop time: 14.01.2022, 01:14**
**measured battery level: 10 %**

**Runtime: 171 h, 35 min (ca. 7 days)**
**Estimated current consumption: 3 mA**

(500 mAh / 171,5h = 2,9 mA)

***
### Run 2: Indirect current measurement BLE module without sensor module

- Measurement interval: 60s
- Power supply: LiFePO4 battery in AA-Format with 550 mAh, fully charged

**Start time: 22.01.2022, 15:15**
**measured battery level: 70%**

**stopped after 170h (ca. 7 days)**
**measured battery level: 68 %**

The i2c bus and the sensor board seem to be the power hungry culprits
***
### Run 3: Indirect current measurement BLE module with sensor module
- I2C sensor is powered via GPIO pin and switched off during sleep/delay
- Measurement interval: 60s
- Power supply: LiFePO4 battery in AA-Format with 550 mAh, fully charged

**Start time: 06.02.2022, 10:30**
**measured battery level: 70%**

**still running**
