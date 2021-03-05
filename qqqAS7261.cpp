#include "qqqAS7261.h"
#include <Wire.h>


void AS7261::begin(){
    disableBulb();
    disableIndicator();
    setIntegrationTime(_intTime); //50 * 2.8ms = 140ms. 0 to 255 is valid.
    //If you use Mode 2 or 3 (all the colors) then integration time is double. 140*2 = 280ms between readings.
    setGain(_gain); 
    setMeasurementMode(2); //continuous
}

void AS7261::autoGain(){
  uint16_t cRaw = getClear();
  int32_t intTimeNew = _intTime * 32000 / ((int32_t)cRaw + 1);
  if(intTimeNew > 255 && _gain<3) {
    _gain++;
    intTimeNew /= 4;
  }else if(intTimeNew < 63 && _gain>0) {
    _gain--;
    intTimeNew *= 4;
  }
  if(intTimeNew>255) intTimeNew = 255;
  _intTime = intTimeNew;  
  setIntegrationTime(_intTime); 
  setGain(_gain);
}

// returns the Sensor Version
// AS7261 or AS7265X
uint8_t AS7261::getVersion() {
    return virtualReadRegister(AS726x_HW_VERSION);
}
 
//Sets the measurement mode
//Mode 0: Continuous reading of VBGY (7262) / STUV (7263)
//Mode 1: Continuous reading of GYOR (7262) / RTUX (7263)
//Mode 2: Continuous reading of all channels (power-on default)
//Mode 3: One-shot reading of all channels
void AS7261::setMeasurementMode(uint8_t mode) {
    if (mode > 0b11) mode = 0b11;

    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
    value &= 0b11110011; //Clear BANK bits
    value |= (mode << 2); //Set BANK bits with user's choice
    virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

//Sets the gain value
//Gain 0: 1x (power-on default)
//Gain 1: 3.7x
//Gain 2: 16x
//Gain 3: 64x
void AS7261::setGain(uint8_t gain) {
    if (gain > 0b11) gain = 0b11;
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
    value &= 0b11001111; //Clear GAIN bits
    value |= (gain << 4); //Set GAIN bits with user's choice
    virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write

    _gain = gain;
}

uint8_t AS7261::getGain() {
  return _gain;
}

float AS7261::getGainValue() {
  return (_gain==0 ? 1.0 : (_gain==1 ? 3.71 : (_gain==3 ? 16.0 : 64.0)));
}

//Sets the integration value
//Give this function a uint8_t from 0 to 255.
//Time will be 2.8ms * [integration value]
void AS7261::setIntegrationTime(uint8_t integrationValue) {
    virtualWriteRegister(AS726x_INT_T, integrationValue); //Write
    _intTime = integrationValue;
}

uint8_t AS7261::getIntegrationTime() {return _intTime;}

// not needed for bank mode 3!
void AS7261::enableInterrupt() {
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
    value |= 0b01000000; //Set INT bit
    virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

//Disables the interrupt pin witch is not connected so disable it!
void AS7261::disableInterrupt() {
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
    value &= 0b10111111; //Clear INT bit
    virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

//Tells IC to take measurements and polls for data ready flag
void AS7261::takeMeasurements() {
    clearDataAvailable(); //Clear DATA_RDY flag when using Mode 3

    //Goto mode 3 for one shot measurement of all channels
    setMeasurementMode(3);

    //Wait for data to be ready
    while (dataAvailable() == 0) delay(POLLING_DELAY); //Potential TODO: avoid this to get faster nearly parralel mesurments

    //Readings can now be accessed via getViolet(), getBlue(), etc
}


//Get RAW AS7261 readings
int AS7261::getX_CIE() { return(getChannel(AS7261_X));}
int AS7261::getY_CIE() { return(getChannel(AS7261_Y));}
int AS7261::getZ_CIE() { return(getChannel(AS7261_Z));}
int AS7261::getNIR() { return(getChannel(AS7261_NIR));}
int AS7261::getDark() { return(getChannel(AS7261_DARK));}
int AS7261::getClear() { return(getChannel(AS7261_CLEAR));}


//A the 16-bit value stored in a given channel registerReturns
int AS7261::getChannel(uint8_t channelRegister){
    int colorData = virtualReadRegister(channelRegister ) << 8; //High uint8_t
    colorData |= virtualReadRegister(channelRegister + 1); //Low uint8_t
    return(colorData);
}

//Returns the various calibration data
float AS7261::getCalibratedX() { return(getCalibratedValue(AS7261_X_CAL)); } 
float AS7261::getCalibratedY() { return(getCalibratedValue(AS7261_Y_CAL)); } 
float AS7261::getCalibratedZ() { return(getCalibratedValue(AS7261_Z_CAL)); } 

//Given an address, read four uint8_ts and return the floating point calibrated value
float AS7261::getCalibratedValue(uint8_t calAddress) {
    uint8_t b0, b1, b2, b3;
    b0 = virtualReadRegister(calAddress + 0);
    b1 = virtualReadRegister(calAddress + 1);
    b2 = virtualReadRegister(calAddress + 2);
    b3 = virtualReadRegister(calAddress + 3);

    //Channel calibrated values are stored big-endian
    uint32_t calBytes = 0;
    calBytes |= ((uint32_t)b0 << (8 * 3));
    calBytes |= ((uint32_t)b1 << (8 * 2));
    calBytes |= ((uint32_t)b2 << (8 * 1));
    calBytes |= ((uint32_t)b3 << (8 * 0));

    return (convertBytesToFloat(calBytes));
}

//Given 4 uint8_ts returns the floating point value
float AS7261::convertBytesToFloat(uint32_t myLong) {
    float myFloat;
    memcpy(&myFloat, &myLong, 4); //Copy uint8_ts into a float
    return (myFloat);
}

//Checks to see if DRDY flag is set in the control setup register
//TODO: was bool test retuned 2
uint8_t AS7261::dataAvailable() {
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP);
    return (value & (1 << 1)); //Bit 1 is DATA_RDY
}

//Clears the DRDY flag
//Normally this should clear when data registers are read
void AS7261::clearDataAvailable() {
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP);
    value &= ~(1 << 1); //Set the DATA_RDY bit
    virtualWriteRegister(AS726x_CONTROL_SETUP, value);
}

//Enable the onboard indicator LED
void AS7261::enableIndicator() {
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
    value |= (1 << 0); //Set the bit
    virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Disable the onboard indicator LED
void AS7261::disableIndicator() {
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL );
    value &= ~(1 << 0); //Clear the bit
    virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Set the current limit of onboard LED. Default is max 8mA = 0b11.
void AS7261::setIndicatorCurrent(uint8_t current) {
    if (current > 0b11) current = 0b11;
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL); //Read
    value &= 0b11111001; //Clear ICL_IND bits
    value |= (current << 1); //Set ICL_IND bits with user's choice
    virtualWriteRegister(AS726x_LED_CONTROL, value); //Write
}

//Enable the onboard 5700k or external incandescent bulb
void AS7261::enableBulb() {
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
    value |= (1 << 3); //Set the bit
    virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Disable the onboard 5700k or external incandescent bulb
void AS7261::disableBulb () {
    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
    value &= ~(1 << 3); //Clear the bit
    virtualWriteRegister(AS726x_LED_CONTROL, value);
}


//Set the current limit of bulb/LED.
//Current 0: 12.5mA
//Current 1: 25mA
//Current 2: 50mA
//Current 3: 100mA
void AS7261::setBulbCurrent(uint8_t current ) {
    if (current > 0b11) current = 0b11; //Limit to two bits

    //Read, mask/set, write
    uint8_t value = virtualReadRegister(AS726x_LED_CONTROL); //Read
    value &= 0b11001111; //Clear ICL_DRV bits
    value |= (current << 4); //Set ICL_DRV bits with user's choice
    virtualWriteRegister(AS726x_LED_CONTROL, value); //Write
}

//Returns the temperature in C
//Pretty inaccurate: +/-8.5C //TODO: mabe include external Termometer to improve Readings
uint8_t AS7261::getTemperature(){
    return (virtualReadRegister(AS726x_DEVICE_TEMP));
}

//Does a soft reset
//Give sensor at least 1000ms to reset
void AS7261::softReset(){
    uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
    value |= (1 << 7); //Set RST bit
    virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}


uint8_t AS7261::i2c_read_byte(uint8_t adr, uint8_t reg) {
  Wire.beginTransmission(adr); 
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(adr,1);
  if(Wire.available()) return Wire.read();
  return 0xff;
}

void AS7261::i2c_write_byte(uint8_t adr, uint8_t reg, uint8_t d) {
  Wire.beginTransmission(adr); 
  Wire.write(reg);
  Wire.write(d);
  Wire.endTransmission();
}


uint8_t AS7261::read8(uint8_t reg) {
  return i2c_read_byte(0x49, reg);
}

void AS7261::write8(uint8_t reg, uint8_t d){
  return i2c_write_byte(0x49, reg, d);
}

enum {
  AS726X_SLAVE_STATUS_REG = 0x00,
  AS726X_SLAVE_WRITE_REG = 0x01,
  AS726X_SLAVE_READ_REG = 0x02,
  AS726X_SLAVE_TX_VALID = 0x02,
  AS726X_SLAVE_RX_VALID = 0x01,
};

uint8_t AS7261::virtualReadRegister(uint8_t addr) {
  int cnt = 0;
  volatile uint8_t status, d;
  while (1) {
    // Read slave I²C status to see if the read buffer is ready.
    status = read8(AS726X_SLAVE_STATUS_REG);
    if ((status & AS726X_SLAVE_TX_VALID) == 0)
      // No inbound TX pending at slave. Okay to write now.
      break;
     //Serial.printf("r=%02X\n",status);
  }
  // Send the virtual register address (clearing bit 7 to indicate a pending read
  write8(AS726X_SLAVE_WRITE_REG, addr);
  while (1) {
    // Read the slave I²C status to see if our read data is available.
    status = read8(AS726X_SLAVE_STATUS_REG);
    if ((status & AS726X_SLAVE_RX_VALID) != 0)
      // Read data is ready.
      break;
           //Serial.printf("w=%02X\n",status);
  }
  // Read the data to complete the operation.
  d = read8(AS726X_SLAVE_READ_REG);
  return d;
}

void AS7261::virtualWriteRegister(uint8_t addr, uint8_t value) {
  volatile uint8_t status;
  while (1) {
    // Read slave I²C status to see if the write buffer is ready.
    status = read8(AS726X_SLAVE_STATUS_REG);
    if ((status & AS726X_SLAVE_TX_VALID) == 0)
      // No inbound TX pending at slave. Okay to write now.
      break;
  }
  // Send the virtual register address (setting bit 7 to indicate a pending write
  write8(AS726X_SLAVE_WRITE_REG, (addr | 0x80));
  // Serial.print("Address $"); Serial.print(addr, HEX);
  while (1) {
    // Read the slave I²C status to see if the write buffer is ready.
    status = read8(AS726X_SLAVE_STATUS_REG);
    if ((status & AS726X_SLAVE_TX_VALID) == 0)
      // No inbound TX pending at slave. Okay to write data now.
      break;
  }
  // Send the data to complete the operation.
  write8(AS726X_SLAVE_WRITE_REG, value);
  // Serial.print(" = 0x"); Serial.println(value, HEX);
}




