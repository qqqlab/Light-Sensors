#ifndef QQQAS7261_H
#define QQQAS7261_H

#include <stdint.h>

class AS7261 {
public:
  void begin();
  void autoGain(); //adjust Integration Time and Gain
  uint8_t getVersion(); //61 oder 65
  void selectDevice(uint8_t device);
  void setMeasurementMode(uint8_t mode);
  void setGain(uint8_t gain);
  uint8_t getGain();
  float getGainValue();
  void setIntegrationTime(uint8_t integrationValue);
  uint8_t getIntegrationTime();
  void enableInterrupt();
  void disableInterrupt();
  void takeMeasurements();
  void MeasurementFromAdress(int address);
  void takeMeasurementsWithBulb();
  
  //Get RAW AS7261 readings
  int getX_CIE();
  int getY_CIE();
  int getZ_CIE();
  int getNIR();
  int getDark();
  int getClear();
  
  int getChannel(uint8_t channelRegister);
  
  float getCalibratedX();
  float getCalibratedY();
  float getCalibratedZ();
  
  float getCalibratedValue(uint8_t calAddress);
  
  float convertBytesToFloat(uint32_t myLong);
  uint8_t dataAvailable();
  void clearDataAvailable();
  void enableIndicator();
  void disableIndicator();
  void setIndicatorCurrent(uint8_t current);
  void enableBulb();
  void disableBulb();
  void setBulbCurrent(uint8_t current);
  uint8_t getTemperature();
  void softReset();
  uint8_t virtualReadRegister(uint8_t virtualAddr);
  void virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite);

private:
  uint8_t _intTime = 100;
  uint8_t _gain = 1; //Gain 0: 1x, 1: 3.7x, 2: 16x, 3: 64x

  uint8_t i2c_read_byte(uint8_t adr, uint8_t reg);
  void i2c_write_byte(uint8_t adr, uint8_t reg, uint8_t d);
  uint8_t read8(uint8_t reg);
  void write8(uint8_t reg, uint8_t d);
};





//Sensor Type Indentifiers
#define SENSORTYPE_AS7261 61
#define SENSORTYPE_AS72651 65

#define AS72XX_SLAVE_TX_VALID 0x02
#define AS72XX_SLAVE_RX_VALID 0x01

#define AS72XX_SLAVE_STATUS_REG 0x00
#define AS72XX_SLAVE_WRITE_REG 0x01
#define AS72XX_SLAVE_READ_REG 0x02

//Register addresses
#define AS726x_DEVICE_TYPE 0x00
#define AS726x_HW_VERSION 0x01
#define AS726x_CONTROL_SETUP 0x04
#define AS726x_INT_T 0x05
#define AS726x_DEVICE_TEMP 0x06
#define AS726x_LED_CONTROL 0x07

//AS7261 Registers
//Raw channel registers
#define AS7261_X 0x08
#define AS7261_Y 0x0A
#define AS7261_Z 0x0C
#define AS7261_NIR 0x0E
#define AS7261_DARK 0x10
#define AS7261_CLEAR 0x12
//Calibrated channel registers
#define AS7261_X_CAL 0x14
#define AS7261_Y_CAL 0x18
#define AS7261_Z_CAL 0x1C
#define AS7261_x_CAL 0x20 //x
#define AS7261_y_CAL 0x24 //y
#define AS7261_up_CAL 0x28 //u'
#define AS7261_vp_CAL 0x2C //v'
#define AS7261_u_CAL 0x30 //u = v'
#define AS7261_v_CAL 0x34 //v = 2/3 u'
#define AS7261_LUX_CAL 0x38 //lux always 0
#define AS7261_CCT_CAL 0x40 //cct always 0



//AS7265X Device Selection
#define AS7265X_DEV_SELECT_CONTROL  0x4F
#define AS72652_ACTIVE  4 // Bit 4 of AS7265X_DEV_SELECT_CONTROL is set if AS72652 is detected
#define AS72653_ACTIVE  5 // Bit 5 of AS7265X_DEV_SELECT_CONTROL is set if AS72653 is detected
//AS7265X Device Selcetors
#define AS72651_id      0x00
#define AS72652_id      0x01
#define AS72653_id      0x02

#define POLLING_DELAY 5 //Amount of ms to wait between checking for virtual register changes

#endif