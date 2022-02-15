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

  https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15242
*/

#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"


//Sets up the NAU7802 for basic function
//If initialize is true (or not specified), default init and calibration is performed
//If initialize is false, then it's up to the caller to initalize and calibrate
//Returns true upon completion
bool NAU7802_begin(I2C_HandleTypeDef * wirePort, bool initialize)
{
  //Get user's options
  _NAU7802_i2cPort = wirePort;

  //Check if the device ack's over I2C
  if (NAU7802_isConnected() == false)
  {
    //There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
    if (NAU7802_isConnected() == false)
      return (false);
  }

  bool result = true; //Accumulate a result as we do the setup

  if (initialize)
  {
    result &= NAU7802_reset(); //Reset all registers

    result &= NAU7802_powerUp(); //Power on analog and digital sections of the scale

    result &= NAU7802_setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3V

    result &= NAU7802_setGain(NAU7802_GAIN_128); //Set gain to 128

    result &= NAU7802_setSampleRate(NAU7802_SPS_80); //Set samples per second to 10

    result &= NAU7802_setRegister(NAU7802_ADC, 0x30); //Turn off CLK_CHP. From 9.1 power on sequencing.

    result &= NAU7802_setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

    result &= NAU7802_calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel
  }

  return (result);
}

//Returns true if device is present
//Tests for device ack to I2C address
bool NAU7802_isConnected()
{
	HAL_StatusTypeDef status = HAL_BUSY;

	uint32_t trials = 3; // number of tries to connect before we give up
	uint32_t timeout = 1000; // number of millisconds to wait before we give up

	status = HAL_I2C_IsDeviceReady(_NAU7802_i2cPort, _NAU7802_deviceAddress<<1, trials, timeout);

	if (status != HAL_OK) {
		return false;
	} else {
		return true;
	}


/*
  _i2cPort->beginTransmission(_deviceAddress);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);    //All good
  */
}

//Returns true if Cycle Ready bit is set (conversion is complete)
bool NAU7802_available()
{
  return (NAU7802_getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL));
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool NAU7802_calibrateAFE()
{
	NAU7802_beginCalibrateAFE();
  return NAU7802_waitForCalibrateAFE(1000);
}

//Begin asynchronous calibration of the analog front end.
// Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
void NAU7802_beginCalibrateAFE()
{
	NAU7802_setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

//Check calibration status.
NAU7802_Cal_Status NAU7802_calAFEStatus()
{
  if (NAU7802_getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2))
  {
    return NAU7802_CAL_IN_PROGRESS;
  }

  if (NAU7802_getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2))
  {
    return NAU7802_CAL_FAILURE;
  }

  // Calibration passed
  return NAU7802_CAL_SUCCESS;
}

//Wait for asynchronous AFE calibration to complete with optional timeout.
//If timeout is not specified (or set to 0), then wait indefinitely.
//Returns true if calibration completes succsfully, otherwise returns false.
bool NAU7802_waitForCalibrateAFE(uint32_t timeout_ms)
{

  uint32_t begin = HAL_GetTick();
  NAU7802_Cal_Status cal_ready;

  while ((cal_ready = NAU7802_calAFEStatus()) == NAU7802_CAL_IN_PROGRESS)
  {
    if ((timeout_ms > 0) && ((HAL_GetTick() - begin) > timeout_ms))
    {
      break;
    }
    HAL_Delay(1);
  }

  if (cal_ready == NAU7802_CAL_SUCCESS)
  {
    return (true);
  }

  return (false);

}

//Set the readings per second
//10, 20, 40, 80, and 320 samples per second is available
bool NAU7802_setSampleRate(uint8_t rate)
{
  if (rate > 0b111)
    rate = 0b111; //Error check

  uint8_t value = NAU7802_getRegister(NAU7802_CTRL2);
  value &= 0b10001111; //Clear CRS bits
  value |= rate << 4;  //Mask in new CRS bits

  return (NAU7802_setRegister(NAU7802_CTRL2, value));
}

//Select between 1 and 2
bool NAU7802_setChannel(uint8_t channelNumber)
{
  if (channelNumber == NAU7802_CHANNEL_1)
    return (NAU7802_clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 1 (default)
  else
    return (NAU7802_setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); //Channel 2
}

//Power up digital and analog sections of scale
bool NAU7802_powerUp()
{
  NAU7802_setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  NAU7802_setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

  //Wait for Power Up bit to be set - takes approximately 200us
  uint8_t counter = 0;
  while (1)
  {
    if (NAU7802_getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
      break; //Good to go
    HAL_Delay(1);
    if (counter++ > 100)
      return (false); //Error
  }
  return (true);
}

//Puts scale into low-power mode
bool NAU7802_powerDown()
{
	NAU7802_clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  return (NAU7802_clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}

//Resets all registers to Power Off Defaults
bool NAU7802_reset()
{
  NAU7802_setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); //Set RR
  HAL_Delay(1);
  return (NAU7802_clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)); //Clear RR to leave reset state
}

//Set the onboard Low-Drop-Out voltage regulator to a given value
//2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
bool NAU7802_setLDO(uint8_t ldoValue)
{
  if (ldoValue > 0b111)
    ldoValue = 0b111; //Error check

  //Set the value of the LDO
  uint8_t value = NAU7802_getRegister(NAU7802_CTRL1);
  value &= 0b11000111;    //Clear LDO bits
  value |= ldoValue << 3; //Mask in new LDO bits
  NAU7802_setRegister(NAU7802_CTRL1, value);

  return (NAU7802_setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Enable the internal LDO
}

//Set the gain
//x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
bool NAU7802_setGain(uint8_t gainValue)
{
  if (gainValue > 0b111)
    gainValue = 0b111; //Error check

  uint8_t value = NAU7802_getRegister(NAU7802_CTRL1);
  value &= 0b11111000; //Clear gain bits
  value |= gainValue;  //Mask in new bits

  return (NAU7802_setRegister(NAU7802_CTRL1, value));
}

//Get the revision code of this IC
uint8_t NAU7802_getRevisionCode()
{
  uint8_t revisionCode = NAU7802_getRegister(NAU7802_DEVICE_REV);
  return (revisionCode & 0x0F);
}

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1 - which can be done with NAU7802_available()
int32_t NAU7802_getReading()
{
	HAL_StatusTypeDef status = HAL_BUSY;
	uint32_t timeout = 1000; // number of ms to wait for a reply

	uint8_t ADC_result_buffer[3]; // 3-byte temporary buffer

	// ask the NAU7802 for the ADC result
	// first we write the address and register value
	uint8_t registerAddress = NAU7802_ADCO_B2;

	status = HAL_I2C_Master_Transmit(_NAU7802_i2cPort, _NAU7802_deviceAddress<<1, &registerAddress, 1, timeout); // send one byte, which is the register address

	if (status != HAL_OK) {
		return 0; // give up if we couldn't transmit
	}

	// then we read back the three result bytes (the NAU7802 automatically sends the subsequent bytes in the register map if we keep reading)
	status = HAL_I2C_Master_Receive(_NAU7802_i2cPort, _NAU7802_deviceAddress<<1, ADC_result_buffer, sizeof(ADC_result_buffer), timeout);

	if (status != HAL_OK) {
		return 0; // again, give up if we lost comms
	}

	// now we reassemble those three bytes into a 24-bit number and perform an unsigned-to-signed conversion
	uint32_t valueRaw = (uint32_t)ADC_result_buffer[0] << 16; //MSB
	valueRaw |= (uint32_t)ADC_result_buffer[1] << 8;          //MidSB
	valueRaw |= (uint32_t)ADC_result_buffer[2];               //LSB

	// the raw value coming from the ADC is a 24-bit number, so the sign bit now
	// resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
	// value to the left, I move the sign bit to the MSB of the uint32_t container.
	// By casting to a signed int32_t container I now have properly recovered
	// the sign of the original value
	int32_t valueShifted = (int32_t)(valueRaw << 8);

	// shift the number back right to recover its intended magnitude
	int32_t value = (valueShifted >> 8);

	return (value);


	/*
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(NAU7802_ADCO_B2);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)3);

  if (_i2cPort->available())
  {
    uint32_t valueRaw = (uint32_t)_i2cPort->read() << 16; //MSB
    valueRaw |= (uint32_t)_i2cPort->read() << 8;          //MidSB
    valueRaw |= (uint32_t)_i2cPort->read();               //LSB

    // the raw value coming from the ADC is a 24-bit number, so the sign bit now
    // resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
    // value to the left, I move the sign bit to the MSB of the uint32_t container.
    // By casting to a signed int32_t container I now have properly recovered
    // the sign of the original value
    int32_t valueShifted = (int32_t)(valueRaw << 8);

    // shift the number back right to recover its intended magnitude
    int32_t value = (valueShifted >> 8);

    return (value);
  }
*/
  return (0); //Error

}

//Return the average of a given number of readings
//Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
int32_t NAU7802_getAverage(uint8_t averageAmount)
{
  long total = 0;
  uint8_t samplesAquired = 0;

  unsigned long startTime = HAL_GetTick();
  while (1)
  {
    if (NAU7802_available() == true)
    {
      total += NAU7802_getReading();
      if (++samplesAquired == averageAmount)
        break; //All done
    }
    if (HAL_GetTick() - startTime > 1000)
      return (0); //Timeout - Bail with error
    HAL_Delay(1);
  }
  total /= averageAmount;

  return (total);
}

//Call when scale is setup, level, at running temperature, with nothing on it
void NAU7802_calculateZeroOffset(uint8_t averageAmount)
{
	NAU7802_setZeroOffset(NAU7802_getAverage(averageAmount));
}

//Sets the internal variable. Useful for users who are loading values from NVM.
void NAU7802_setZeroOffset(int32_t newZeroOffset)
{
  _NAU7802_zeroOffset = newZeroOffset;
}

int32_t NAU7802_getZeroOffset()
{
  return (_NAU7802_zeroOffset);
}

//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
void NAU7802_calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount)
{
  int32_t onScale = NAU7802_getAverage(averageAmount);
  float newCalFactor = (onScale - _NAU7802_zeroOffset) / (float)weightOnScale;
  NAU7802_setCalibrationFactor(newCalFactor);
}

//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
//If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
void NAU7802_setCalibrationFactor(float newCalFactor)
{
  _NAU7802_calibrationFactor = newCalFactor;
}

float NAU7802_getCalibrationFactor()
{
  return (_NAU7802_calibrationFactor);
}

//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
float NAU7802_getWeight(bool allowNegativeWeights, uint8_t samplesToTake)
{
  int32_t onScale = NAU7802_getAverage(samplesToTake);

  //Prevent the current reading from being less than zero offset
  //This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
  //causing the weight to be negative or jump to millions of pounds
  if (allowNegativeWeights == false)
  {
    if (onScale < _NAU7802_zeroOffset)
      onScale = _NAU7802_zeroOffset; //Force reading to zero
  }

  float weight = (onScale - _NAU7802_zeroOffset) / _NAU7802_calibrationFactor;
  return (weight);
}

//Set Int pin to be high when data is ready (default)
bool NAU7802_setIntPolarityHigh()
{
  return (NAU7802_clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
}

//Set Int pin to be low when data is ready
bool NAU7802_setIntPolarityLow()
{
  return (NAU7802_setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
}

//Mask & set a given bit within a register
bool NAU7802_setBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = NAU7802_getRegister(registerAddress);
  value |= (1 << bitNumber); //Set this bit
  return (NAU7802_setRegister(registerAddress, value));
}

//Mask & clear a given bit within a register
bool NAU7802_clearBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = NAU7802_getRegister(registerAddress);
  value &= ~(1 << bitNumber); //Set this bit
  return (NAU7802_setRegister(registerAddress, value));
}

//Return a given bit within a register
bool NAU7802_getBit(uint8_t bitNumber, uint8_t registerAddress)
{
  uint8_t value = NAU7802_getRegister(registerAddress);
  value &= (1 << bitNumber); //Clear all but this bit
  return (value);
}

//Get contents of a register
uint8_t NAU7802_getRegister(uint8_t registerAddress)
{
	HAL_StatusTypeDef status = HAL_BUSY;
	uint32_t timeout = 1000; // number of ms to wait for a reply

	uint8_t register_value = 0; // return value

	// write to the NAU7802 with the register address
	status = HAL_I2C_Master_Transmit(_NAU7802_i2cPort, _NAU7802_deviceAddress<<1, &registerAddress, 1, timeout); // send one byte, which is the register address

	if (status != HAL_OK) {
		return false; // give up if we couldn't transmit
	}

	// read back one byte
	status = HAL_I2C_Master_Receive(_NAU7802_i2cPort, _NAU7802_deviceAddress<<1, &register_value, 1, timeout); // all registers are 1 byte

	if (status != HAL_OK) {
		return false; // again, give up if we lost comms
	}

	return register_value;



	/*
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(registerAddress);
  if (_i2cPort->endTransmission() != 0)
    return (-1); //Sensor did not ACK

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);

  if (_i2cPort->available())
    return (_i2cPort->read());

  return (-1); //Error
*/
}

//Send a given value to be written to given address
//Return true if successful
bool NAU7802_setRegister(uint8_t registerAddress, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_BUSY;
	uint32_t timeout = 1000; // number of ms to wait for a reply

	uint8_t data_buffer[2]; // two bytes to send

	data_buffer[0] = registerAddress;
	data_buffer[1] = value;

	status = HAL_I2C_Master_Transmit(_NAU7802_i2cPort, _NAU7802_deviceAddress<<1, data_buffer, sizeof(data_buffer), timeout);

	if (status != HAL_OK) {
		return false;
	} else {
		return true;
	}


	/*
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(registerAddress);
  _i2cPort->write(value);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
  */

}
