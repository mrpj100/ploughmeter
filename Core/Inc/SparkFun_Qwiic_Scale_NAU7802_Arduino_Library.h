/*
  This library was ported to STM32 by Mike Prior-Jones, February 2022.

  The original SparkFun copyright declaration and information is below.

  This is an Arduino library written for the NAU7802 24-bit wheatstone
  bridge and load cell amplifier.
  By Nathan Seidle @ SparkFun Electronics, March 3nd, 2019

  The NAU7802 is an I2C device that converts analog signals to a 24-bit
  digital signal. This makes it possible to create your own digital scale
  either by hacking an off-the-shelf bathroom scale or by creating your
  own scale using a load cell.

  The NAU7802 is a better version of the popular HX711 load cell amplifier.
  It uses a true I2C interface so that it can share the bus with other
  I2C devices while still taking very accurate 24-bit load cell measurements
  up to 320Hz.

  https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15242
*/

#ifndef SparkFun_Qwiic_Scale_NAU7802_Arduino_Library_h
#define SparkFun_Qwiic_Scale_NAU7802_Arduino_Library_h

//#include "Arduino.h"
//#include <Wire.h>

#include "stm32l4xx_hal.h"

//Register Map
typedef enum
{
  NAU7802_PU_CTRL = 0x00,
  NAU7802_CTRL1,
  NAU7802_CTRL2,
  NAU7802_OCAL1_B2,
  NAU7802_OCAL1_B1,
  NAU7802_OCAL1_B0,
  NAU7802_GCAL1_B3,
  NAU7802_GCAL1_B2,
  NAU7802_GCAL1_B1,
  NAU7802_GCAL1_B0,
  NAU7802_OCAL2_B2,
  NAU7802_OCAL2_B1,
  NAU7802_OCAL2_B0,
  NAU7802_GCAL2_B3,
  NAU7802_GCAL2_B2,
  NAU7802_GCAL2_B1,
  NAU7802_GCAL2_B0,
  NAU7802_I2C_CONTROL,
  NAU7802_ADCO_B2,
  NAU7802_ADCO_B1,
  NAU7802_ADCO_B0,
  NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
  NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
  NAU7802_OTP_B0,     //OTP 15:8
  NAU7802_PGA = 0x1B,
  NAU7802_PGA_PWR = 0x1C,
  NAU7802_DEVICE_REV = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
  NAU7802_PU_CTRL_RR = 0,
  NAU7802_PU_CTRL_PUD,
  NAU7802_PU_CTRL_PUA,
  NAU7802_PU_CTRL_PUR,
  NAU7802_PU_CTRL_CS,
  NAU7802_PU_CTRL_CR,
  NAU7802_PU_CTRL_OSCS,
  NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

//Bits within the CTRL1 register
typedef enum
{
  NAU7802_CTRL1_GAIN = 2,
  NAU7802_CTRL1_VLDO = 5,
  NAU7802_CTRL1_DRDY_SEL = 6,
  NAU7802_CTRL1_CRP = 7,
} CTRL1_Bits;

//Bits within the CTRL2 register
typedef enum
{
  NAU7802_CTRL2_CALMOD = 0,
  NAU7802_CTRL2_CALS = 2,
  NAU7802_CTRL2_CAL_ERROR = 3,
  NAU7802_CTRL2_CRS = 4,
  NAU7802_CTRL2_CHS = 7,
} CTRL2_Bits;

//Bits within the PGA register
typedef enum
{
  NAU7802_PGA_CHP_DIS = 0,
  NAU7802_PGA_INV = 3,
  NAU7802_PGA_BYPASS_EN,
  NAU7802_PGA_OUT_EN,
  NAU7802_PGA_LDOMODE,
  NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

//Bits within the PGA PWR register
typedef enum
{
  NAU7802_PGA_PWR_PGA_CURR = 0,
  NAU7802_PGA_PWR_ADC_CURR = 2,
  NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
  NAU7802_PGA_PWR_PGA_CAP_EN = 7,
} PGA_PWR_Bits;

//Allowed Low drop out regulator voltages
typedef enum
{
  NAU7802_LDO_2V4 = 0b111,
  NAU7802_LDO_2V7 = 0b110,
  NAU7802_LDO_3V0 = 0b101,
  NAU7802_LDO_3V3 = 0b100,
  NAU7802_LDO_3V6 = 0b011,
  NAU7802_LDO_3V9 = 0b010,
  NAU7802_LDO_4V2 = 0b001,
  NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
  NAU7802_GAIN_128 = 0b111,
  NAU7802_GAIN_64 = 0b110,
  NAU7802_GAIN_32 = 0b101,
  NAU7802_GAIN_16 = 0b100,
  NAU7802_GAIN_8 = 0b011,
  NAU7802_GAIN_4 = 0b010,
  NAU7802_GAIN_2 = 0b001,
  NAU7802_GAIN_1 = 0b000,
} NAU7802_Gain_Values;

//Allowed samples per second
typedef enum
{
  NAU7802_SPS_320 = 0b111,
  NAU7802_SPS_80 = 0b011,
  NAU7802_SPS_40 = 0b010,
  NAU7802_SPS_20 = 0b001,
  NAU7802_SPS_10 = 0b000,
} NAU7802_SPS_Values;

//Select between channel values
typedef enum
{
  NAU7802_CHANNEL_1 = 0,
  NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

//Calibration state
typedef enum
{
  NAU7802_CAL_SUCCESS = 0,
  NAU7802_CAL_IN_PROGRESS = 1,
  NAU7802_CAL_FAILURE = 2,
} NAU7802_Cal_Status;



// Public functions

  bool NAU7802_begin(I2C_HandleTypeDef wirePort, bool reset); //Check communication and initialize sensor. Default value for reset = true
  bool NAU7802_isConnected();                                      //Returns true if device acks at the I2C address

  bool NAU7802_available();                          //Returns true if Cycle Ready bit is set (conversion is complete)
  int32_t NAU7802_getReading();                      //Returns 24-bit reading. Assumes CR Cycle Ready bit (ADC conversion complete) has been checked by .available()
  int32_t NAU7802_getAverage(uint8_t samplesToTake); //Return the average of a given number of readings

  void NAU7802_calculateZeroOffset(uint8_t averageAmount); //Also called taring. Call this with nothing on the scale. Default value averageAmount = 8
  void NAU7802_setZeroOffset(int32_t newZeroOffset);           //Sets the internal variable. Useful for users who are loading values from NVM.
  int32_t NAU7802_getZeroOffset();                             //Ask library for this value. Useful for storing value into NVM.

  void NAU7802_calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount); //Call this with the value of the thing on the scale. Sets the calibration factor based on the weight on scale and zero offset. Default averageAmount = 8
  void NAU7802_setCalibrationFactor(float calFactor);                                      //Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
  float NAU7802_getCalibrationFactor();                                                    //Ask library for this value. Useful for storing value into NVM.

  float NAU7802_getWeight(bool allowNegativeWeights, uint8_t samplesToTake); //Once you've set zero offset and cal factor, you can ask the library to do the calculations for you. Default values: allowNegativeWeights = false samplesToTake = 8

  bool NAU7802_setGain(uint8_t gainValue);        //Set the gain. x1, 2, 4, 8, 16, 32, 64, 128 are available
  bool NAU7802_setLDO(uint8_t ldoValue);          //Set the onboard Low-Drop-Out voltage regulator to a given value. 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are avaialable
  bool NAU7802_setSampleRate(uint8_t rate);       //Set the readings per second. 10, 20, 40, 80, and 320 samples per second is available
  bool NAU7802_setChannel(uint8_t channelNumber); //Select between 1 and 2

  bool NAU7802_calibrateAFE();                               //Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
  void NAU7802_beginCalibrateAFE();                          //Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
  bool NAU7802_waitForCalibrateAFE(uint32_t timeout_ms); //Wait for asynchronous AFE calibration to complete with optional timeout. Default timeout_ms = 0
  NAU7802_Cal_Status NAU7802_calAFEStatus();                 //Check calibration status.

  bool NAU7802_reset(); //Resets all registers to Power Of Defaults

  bool NAU7802_powerUp();   //Power up digital and analog sections of scale, ~2mA
  bool NAU7802_powerDown(); //Puts scale into low-power 200nA mode

  bool NAU7802_setIntPolarityHigh(); //Set Int pin to be high when data is ready (default)
  bool NAU7802_setIntPolarityLow();  //Set Int pin to be low when data is ready

  uint8_t NAU7802_getRevisionCode(); //Get the revision code of this IC. Always 0x0F.

  bool NAU7802_setBit(uint8_t bitNumber, uint8_t registerAddress);   //Mask & set a given bit within a register
  bool NAU7802_clearBit(uint8_t bitNumber, uint8_t registerAddress); //Mask & clear a given bit within a register
  bool NAU7802_getBit(uint8_t bitNumber, uint8_t registerAddress);   //Return a given bit within a register

  uint8_t NAU7802_getRegister(uint8_t registerAddress);             //Get contents of a register
  bool NAU7802_setRegister(uint8_t registerAddress, uint8_t value); //Send a given value to be written to given address. Return true if successful

// Private variables
  I2C_HandleTypeDef _NAU7802_i2cPort;                   //This stores the user's requested i2c port
  const uint8_t _NAU7802_deviceAddress = 0x2A; //Default unshifted 7-bit address of the NAU7802

  //y = mx+b
  int32_t _NAU7802_zeroOffset;      //This is b
  float _NAU7802_calibrationFactor; //This is m. User provides this number so that we can output y when requested

#endif
