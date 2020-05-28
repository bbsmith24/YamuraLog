/******************************************************************************
SX1509_Library.cpp
based on SparkFun SX1509 I/O Expander Arduino Library
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SX1509_Library.h"
#include "SX1509_Registers.h"
// default constructor
SX1509::SX1509()
{
	_clkX = 0;
}

uint8_t SX1509::SX1509_Begin(uint8_t address, uint8_t resetPin)
{
	// Store the received parameters into member variables
	deviceAddress =  address;
	pinReset = resetPin;

	// If the reset pin is connected
	if (pinReset != 255)
	{
		SX1509_Reset(1);
    }
	else
    {
        SX1509_Reset(0);
    }

    if(!i2cDevice.I2CInit(address))
    {
        return 0;
    }

	// Communication test. We'll read from two registers with different
	// default values to verify communication.
	I2C_Library::I2CWord testRegisters;
	if(i2cDevice.I2CReadBytes(REG_INTERRUPT_MASK_A, &testRegisters) == 0)
	{
        return 0;
	}

	// Then read a uint8_t that should be 0x00
	if (testRegisters.intVal == 0xFF00)
	{
		// Set the clock to a default of 2MHz using internal
		clock(INTERNAL_CLOCK_2MHZ);

		return 1;
	}

	return 0;
}

void SX1509::SX1509_Reset(bool hardware)
{
	// if hardware bool is set
	if (hardware)
	{
		// Check if bit 2 of REG_MISC is set
		// if so nReset will not issue a POR, we'll need to clear that bit first
		uint8_t regMisc = i2cDevice.I2CReadByte(REG_MISC);
		if (regMisc & (1<<2))
		{
			regMisc &= ~(1<<2);
			i2cDevice.I2CWriteByte(REG_MISC, regMisc);
		}
		// Reset the SX1509, the pin is active low
		pinMode(pinReset, OUTPUT);	// set reset pin as output
		digitalWrite(pinReset, LOW);	// pull reset pin low
		delay(1);	// Wait for the pin to settle
		digitalWrite(pinReset, HIGH);	// pull reset pin back high
	}
	else
	{
		// Software reset command sequence:
		i2cDevice.I2CWriteByte(REG_RESET, 0x12);
		i2cDevice.I2CWriteByte(REG_RESET, 0x34);
	}
}

void SX1509::pinMode(uint8_t pin, uint8_t inOut)
{
	// The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
	//	0: IO is configured as an output
	//	1: IO is configured as an input
	uint8_t modeBit;
	if ((inOut == OUTPUT) || (inOut == ANALOG_OUTPUT))
	{
		modeBit = 0;
    }
	else
	{
		modeBit = 1;
    }
    I2C_Library::I2CWord tempRegDir;
    i2cDevice.I2CReadBytes(REG_DIR_B, &tempRegDir);
	if (modeBit)
	{
		tempRegDir.intVal |= (1<<pin);
	}
	else
	{
		tempRegDir.intVal &= ~(1<<pin);
	}
    i2cDevice.I2CWriteBytes(REG_DIR_B, tempRegDir);
	// If INPUT_PULLUP was called, set up the pullup too:
	if (inOut == INPUT_PULLUP)
	{
		writePin(pin, HIGH);
	}

	if (inOut == ANALOG_OUTPUT)
	{
		ledDriverInit(pin);
	}
}

void SX1509::writePin(uint8_t pin, uint8_t highLow)
{
	I2C_Library::I2CWord tempRegDir;
	I2C_Library::I2CWord tempRegData;
	I2C_Library::I2CWord tempPullUp;
	I2C_Library::I2CWord tempPullDown;

    i2cDevice.I2CReadBytes(REG_DIR_B, &tempRegDir);

	if ((0xFFFF^tempRegDir.intVal)&(1<<pin))	// If the pin is an output, write high/low
	{
		i2cDevice.I2CReadBytes(REG_DATA_B, &tempRegData);
		if (highLow)
		{
			tempRegData.intVal |= (1<<pin);
		}
		else
		{
			tempRegData.intVal &= ~(1<<pin);
		}
		i2cDevice.I2CWriteBytes(REG_DATA_B, tempRegData);
	}
	else	// Otherwise the pin is an input, pull-up/down
	{
		i2cDevice.I2CReadBytes(REG_PULL_UP_B, &tempPullUp);
		i2cDevice.I2CReadBytes(REG_PULL_DOWN_B, &tempPullDown);

		if (highLow)	// if HIGH, do pull-up, disable pull-down
		{
			tempPullUp.intVal |= (1<<pin);
			tempPullDown.intVal &= ~(1<<pin);
			i2cDevice.I2CWriteBytes(REG_PULL_UP_B, tempPullUp);
			i2cDevice.I2CWriteBytes(REG_PULL_DOWN_B, tempPullDown);
		}
		else	// If LOW do pull-down, disable pull-up
		{
			tempPullDown.intVal |= (1<<pin);
			tempPullUp.intVal &= ~(1<<pin);
			i2cDevice.I2CWriteBytes(REG_PULL_UP_B, tempPullUp);
			i2cDevice.I2CWriteBytes(REG_PULL_DOWN_B, tempPullDown);
		}
	}
}

void SX1509::digitalWrite(uint8_t pin, uint8_t highLow)
{
	writePin(pin, highLow);
}

uint8_t SX1509::readPin(uint8_t pin)
{
    I2C_Library::I2CWord regDir;
    I2C_Library::I2CWord regData;
    i2cDevice.I2CReadBytes(REG_DIR_B, &regDir);

	if (regDir.intVal & (1<<pin))	// If the pin is an input
	{
		i2cDevice.I2CReadBytes(REG_DATA_B, &regData);
        if (regData.intVal & (1<<pin))
		{
			return 1;
        }
	}
	return 0;
}

void SX1509::readPins(I2C_Library::I2CWord *pins)
{
    I2C_Library::I2CWord regDir;
    i2cDevice.I2CReadBytes(REG_DIR_B, &regDir);
	i2cDevice.I2CReadBytes(REG_DATA_B, pins);
}

uint8_t SX1509::digitalRead(uint8_t pin)
{
	return readPin(pin);
}

void SX1509::ledDriverInit(uint8_t pin, uint8_t freq /*= 1*/, bool log /*= false*/)
{
	I2C_Library::I2CWord tempWord;
	uint8_t tempuint8_t;

	// Disable input buffer
	// Writing a 1 to the pin bit will disable that pins input buffer
	i2cDevice.I2CReadBytes(REG_INPUT_DISABLE_B, &tempWord);
	tempWord.intVal |= (1<<pin);
	i2cDevice.I2CWriteBytes(REG_INPUT_DISABLE_B, tempWord);

	// Disable pull-up
	// Writing a 0 to the pin bit will disable that pull-up resistor
	i2cDevice.I2CReadBytes(REG_PULL_UP_B, &tempWord);
	tempWord.intVal &= ~(1<<pin);
	i2cDevice.I2CWriteBytes(REG_PULL_UP_B, tempWord);

	// Set direction to output (REG_DIR_B)
	i2cDevice.I2CReadBytes(REG_DIR_B, &tempWord);
	tempWord.intVal &= ~(1<<pin); // 0=output
	i2cDevice.I2CWriteBytes(REG_DIR_B, tempWord);

	// Enable oscillator (REG_CLOCK)
	tempuint8_t = i2cDevice.I2CReadByte(REG_CLOCK);
	tempuint8_t |= (1<<6);	// Internal 2MHz oscillator part 1 (set bit 6)
	tempuint8_t &= ~(1<<5);	// Internal 2MHz oscillator part 2 (clear bit 5)
	i2cDevice.I2CWriteByte(REG_CLOCK, tempuint8_t);

	// Configure LED driver clock and mode (REG_MISC)
	tempuint8_t = i2cDevice.I2CReadByte(REG_MISC);
	if (log)
	{
		tempuint8_t |= (1<<7);	// set logarithmic mode bank B
		tempuint8_t |= (1<<3);	// set logarithmic mode bank A
	}
	else
	{
		tempuint8_t &= ~(1<<7);	// set linear mode bank B
		tempuint8_t &= ~(1<<3);	// set linear mode bank A
	}

	// Use configClock to setup the clock divder
	if (_clkX == 0) // Make clckX non-zero
	{
		_clkX = 2000000.0 / (1<<(1 - 1)); // Update private clock variable

		uint8_t freq = (1 & 0x07) << 4;	// freq should only be 3 bits from 6:4
		tempuint8_t |= freq;
	}
	i2cDevice.I2CWriteByte(REG_MISC, tempuint8_t);

	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	i2cDevice.I2CReadBytes(REG_LED_DRIVER_ENABLE_B, &tempWord);
	tempWord.intVal |= (1<<pin);
	i2cDevice.I2CWriteBytes(REG_LED_DRIVER_ENABLE_B, tempWord);

	// Set REG_DATA bit low ~ LED driver started
	i2cDevice.I2CReadBytes(REG_DATA_B, &tempWord);
	tempWord.intVal &= ~(1<<pin);
	i2cDevice.I2CWriteBytes(REG_DATA_B, tempWord);
}

void SX1509::pwm(uint8_t pin, uint8_t iOn)
{
	// Write the on intensity of pin
	// Linear mode: Ion = iOn
	// Log mode: Ion = f(iOn)
	i2cDevice.I2CWriteByte(REG_I_ON[pin], iOn);
}

void SX1509::analogWrite(uint8_t pin, uint8_t iOn)
{
	pwm(pin, iOn);
}

void SX1509::blink(uint8_t pin, unsigned long tOn, unsigned long tOff, uint8_t onIntensity, uint8_t offIntensity)
{
	uint8_t onReg = calculateLEDTRegister(tOn);
	uint8_t offReg = calculateLEDTRegister(tOff);

	setupBlink(pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
}

void SX1509::breathe(uint8_t pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, uint8_t onInt, uint8_t offInt, bool log)
{
	offInt = constrain(offInt, 0, 7);

	uint8_t onReg = calculateLEDTRegister(tOn);
	uint8_t offReg = calculateLEDTRegister(tOff);

	uint8_t riseTime = calculateSlopeRegister(rise, onInt, offInt);
	uint8_t fallTime = calculateSlopeRegister(fall, onInt, offInt);

	setupBlink(pin, onReg, offReg, onInt, offInt, riseTime, fallTime, log);
}

void SX1509::setupBlink(uint8_t pin, uint8_t tOn, uint8_t tOff, uint8_t onIntensity, uint8_t offIntensity, uint8_t tRise, uint8_t tFall, bool log)
{
	ledDriverInit(pin, log);

	// Keep parameters within their limits:
	tOn &= 0x1F;	// tOn should be a 5-bit value
	tOff &= 0x1F;	// tOff should be a 5-bit value
	offIntensity &= 0x07;
	// Write the time on
	// 1-15:  TON = 64 * tOn * (255/ClkX)
	// 16-31: TON = 512 * tOn * (255/ClkX)
	i2cDevice.I2CWriteByte(REG_T_ON[pin], tOn);

	// Write the time/intensity off register
	// 1-15:  TOFF = 64 * tOff * (255/ClkX)
	// 16-31: TOFF = 512 * tOff * (255/ClkX)
	// linear Mode - IOff = 4 * offIntensity
	// log mode - Ioff = f(4 * offIntensity)
	i2cDevice.I2CWriteByte(REG_OFF[pin], (tOff<<3) | offIntensity);

	// Write the on intensity:
	i2cDevice.I2CWriteByte(REG_I_ON[pin], onIntensity);

	// Prepare tRise and tFall
	tRise &= 0x1F;	// tRise is a 5-bit value
	tFall &= 0x1F;	// tFall is a 5-bit value

	// Write regTRise
	// 0: Off
	// 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	// 16-31: TRise = 16 * (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	if (REG_T_RISE[pin] != 0xFF)
		i2cDevice.I2CWriteByte(REG_T_RISE[pin], tRise);
	// Write regTFall
	// 0: off
	// 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	// 16-31: TFall = 16 * (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	if (REG_T_FALL[pin] != 0xFF)
		i2cDevice.I2CWriteByte(REG_T_FALL[pin], tFall);
}

void SX1509::keypad(uint8_t rows, uint8_t columns, unsigned int sleepTime, uint8_t scanTime, uint8_t debounceTime)
{
	I2C_Library::I2CWord tempWord;
    uint8_t tempuint8_t;

	// If clock hasn't been set up, set it to internal 2MHz
	if (_clkX == 0)
		clock(INTERNAL_CLOCK_2MHZ);

	// Set regDir 0:7 outputs, 8:15 inputs:
	i2cDevice.I2CReadBytes(REG_DIR_B, &tempWord);
	for (int i=0; i<rows; i++)
		tempWord.intVal &= ~(1<<i);
	for (int i=8; i<(columns * 2); i++)
		tempWord.intVal |= (1<<i);
	i2cDevice.I2CWriteBytes(REG_DIR_B, tempWord);

	// Set regOpenDrain on 0:7:
	tempuint8_t = i2cDevice.I2CReadByte(REG_OPEN_DRAIN_A);
	for (int i=0; i<rows; i++)
		tempuint8_t |= (1<<i);
	i2cDevice.I2CWriteByte(REG_OPEN_DRAIN_A, tempuint8_t);

	// Set regPullUp on 8:15:
	tempuint8_t = i2cDevice.I2CReadByte(REG_PULL_UP_B);
	for (int i=0; i<columns; i++)
		tempuint8_t |= (1<<i);
	i2cDevice.I2CWriteByte(REG_PULL_UP_B, tempuint8_t);

	// Debounce Time must be less than scan time
	debounceTime = constrain(debounceTime, 1, 64);
	scanTime = constrain(scanTime, 1, 128);
	if (debounceTime >= scanTime)
	{
		debounceTime = scanTime >> 1; // Force debounceTime to be less than scanTime
	}
	debounceKeypad(debounceTime, rows, columns);

	// Calculate scanTimeBits, based on scanTime
	uint8_t scanTimeBits = 0;
	for (uint8_t i=7; i>0; i--)
	{
		if (scanTime & (1<<i))
		{
			scanTimeBits = i;
			break;
		}
	}

	// Calculate sleepTimeBits, based on sleepTime
	uint8_t sleepTimeBits = 0;
	if (sleepTime != 0)
	{
		for (uint8_t i=7; i>0; i--)
		{
			if (sleepTime & ((unsigned int)1<<(i+6)))
			{
				sleepTimeBits = i;
				break;
			}
		}
		// If sleepTime was non-zero, but less than 128,
		// assume we wanted to turn sleep on, set it to minimum:
		if (sleepTimeBits == 0)
			sleepTimeBits = 1;
	}

	// RegKeyConfig1 sets the auto sleep time and scan time per row
	sleepTimeBits = (sleepTimeBits & 0b111)<<4;
	scanTimeBits &= 0b111;	// Scan time is bits 2:0
	tempuint8_t = sleepTime | scanTimeBits;
	i2cDevice.I2CWriteByte(REG_KEY_CONFIG_1, tempuint8_t);

	// RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
	rows = (rows - 1) & 0b111;	// 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
	columns = (columns - 1) & 0b111;	// 0b000 = 1 column, ob111 = 8 columns, etc.
	i2cDevice.I2CWriteByte(REG_KEY_CONFIG_2, (rows << 3) | columns);
}

unsigned int SX1509::readKeypad()
{
	return readKeyData();
}

unsigned int SX1509::readKeyData()
{
    I2C_Library::I2CWord tmpWord;
    i2cDevice.I2CReadBytes(REG_KEY_DATA_1, &tmpWord);
	return (0xFFFF ^ tmpWord.intVal);
}

uint8_t SX1509::getRow(unsigned int keyData)
{
	uint8_t rowData = uint8_t(keyData & 0x00FF);

	for (int i=0; i<8; i++)
	{
		if (rowData & (1<<i))
			return i;
	}
	return 0;
}

uint8_t SX1509::getCol(unsigned int keyData)
{
	uint8_t colData = uint8_t((keyData & 0xFF00) >> 8);

	for (int i=0; i<8; i++)
	{
		if (colData & (1<<i))
			return i;
	}
	return 0;

}

void SX1509::sync(void)
{
	// First check if nReset functionality is set
	uint8_t regMisc = i2cDevice.I2CReadByte(REG_MISC);
	if (!(regMisc & 0x04))
	{
		regMisc |= (1<<2);
		i2cDevice.I2CWriteByte(REG_MISC, regMisc);
	}

	// Toggle nReset pin to sync LED timers
	pinMode(pinReset, OUTPUT);	// set reset pin as output
	digitalWrite(pinReset, LOW);	// pull reset pin low
	delay(1);	// Wait for the pin to settle
	digitalWrite(pinReset, HIGH);	// pull reset pin back high

	// Return nReset to POR functionality
	i2cDevice.I2CWriteByte(REG_MISC, (regMisc & ~(1<<2)));
}

void SX1509::debounceConfig(uint8_t configValue)
{
	// First make sure clock is configured
	uint8_t tempuint8_t = i2cDevice.I2CReadByte(REG_MISC);
	if ((tempuint8_t & 0x70) == 0)
	{
		tempuint8_t |= (1<<4);	// Just default to no divider if not set
		i2cDevice.I2CWriteByte(REG_MISC, tempuint8_t);
	}
	tempuint8_t = i2cDevice.I2CReadByte(REG_CLOCK);
	if ((tempuint8_t & 0x60) == 0)
	{
		tempuint8_t |= (1<<6);	// default to internal osc.
		i2cDevice.I2CWriteByte(REG_CLOCK, tempuint8_t);
	}

	configValue &= 0b111;	// 3-bit value
	i2cDevice.I2CWriteByte(REG_DEBOUNCE_CONFIG, configValue);
}

void SX1509::debounceTime(uint8_t time)
{
	if (_clkX == 0) // If clock hasn't been set up.
		clock(INTERNAL_CLOCK_2MHZ, 1); // Set clock to 2MHz.

	// Debounce time-to-uint8_t map: (assuming fOsc = 2MHz)
	// 0: 0.5ms		1: 1ms
	// 2: 2ms		3: 4ms
	// 4: 8ms		5: 16ms
	// 6: 32ms		7: 64ms
	// 2^(n-1)
	uint8_t configValue = 0;
	// We'll check for the highest set bit position,
	// and use that for debounceConfig
	for (int i=7; i>=0; i--)
	{
		if (time & (1<<i))
		{
			configValue = i + 1;
			break;
		}
	}
	configValue = constrain(configValue, 0, 7);

	debounceConfig(configValue);
}

void SX1509::debounceEnable(uint8_t pin)
{
    I2C_Library::I2CWord debounceEnableVal;
	i2cDevice.I2CReadBytes(REG_DEBOUNCE_ENABLE_B, &debounceEnableVal);
	debounceEnableVal.intVal |= (1<<pin);
	i2cDevice.I2CWriteBytes(REG_DEBOUNCE_ENABLE_B, debounceEnableVal);
}

void SX1509::debouncePin(uint8_t pin)
{
	debounceEnable(pin);
}

void SX1509::debounceKeypad(uint8_t time, uint8_t numRows, uint8_t numCols)
{
	// Set up debounce time:
	debounceTime(time);

	// Set up debounce pins:
	for (int i = 0; i < numRows; i++)
		debouncePin(i);
	for (int i = 0; i < (8 + numCols); i++)
		debouncePin(i);
}

void SX1509::enableInterrupt(uint8_t pin, uint8_t riseFall)
{
	// Set REG_INTERRUPT_MASK
	I2C_Library::I2CWord tempWord;
	i2cDevice.I2CReadBytes(REG_INTERRUPT_MASK_B, &tempWord);
	tempWord.intVal &= ~(1<<pin);	// 0 = event on IO will trigger interrupt
	i2cDevice.I2CWriteBytes(REG_INTERRUPT_MASK_B, tempWord);

	uint8_t sensitivity = 0;
	switch (riseFall)
	{
	case CHANGE:
		sensitivity = 0b11;
		break;
	case FALLING:
		sensitivity = 0b10;
		break;
	case RISING:
		sensitivity = 0b01;
		break;
	}

	// Set REG_SENSE_XXX
	// Sensitivity is set as follows:
	// 00: None
	// 01: Rising
	// 10: Falling
	// 11: Both
	uint8_t pinMask = (pin & 0x07) * 2;
	uint8_t senseRegister;

	// Need to select between two words. One for bank A, one for B.
	if (pin >= 8)	senseRegister = REG_SENSE_HIGH_B;
	else			senseRegister = REG_SENSE_HIGH_A;

	i2cDevice.I2CReadBytes(senseRegister, &tempWord);
	tempWord.intVal &= ~(0b11<<pinMask);	// Mask out the bits we want to write
	tempWord.intVal |= (sensitivity<<pinMask);	// Add our new bits
	i2cDevice.I2CWriteBytes(senseRegister, tempWord);
}

unsigned int SX1509::interruptSource(bool clear /* =true*/)
{
	I2C_Library::I2CWord intSource;
	I2C_Library::I2CWord clearVal;
	i2cDevice.I2CReadBytes(REG_INTERRUPT_SOURCE_B, &intSource);
	if (clear)
	{
	    clearVal.intVal = 0xFFFF;
		i2cDevice.I2CWriteBytes(REG_INTERRUPT_SOURCE_B, clearVal);	// Clear interrupts
	}
	return intSource.intVal;
}

bool SX1509::checkInterrupt(int pin)
{
	if (interruptSource(false) & (1<<pin))
		return true;

	return false;
}

void SX1509::clock(uint8_t oscSource, uint8_t oscDivider, uint8_t oscPinFunction, uint8_t oscFreqOut)
{
	configClock(oscSource, oscPinFunction, oscFreqOut, oscDivider);
}

void SX1509::configClock(uint8_t oscSource /*= 2*/, uint8_t oscPinFunction /*= 0*/, uint8_t oscFreqOut /*= 0*/, uint8_t oscDivider /*= 1*/)
{
	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency souce
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 ouptut
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	oscSource = (oscSource & 0b11)<<5;		// 2-bit value, bits 6:5
	oscPinFunction = (oscPinFunction & 1)<<4;	// 1-bit value bit 4
	oscFreqOut = (oscFreqOut & 0b1111);	// 4-bit value, bits 3:0
	uint8_t regClock = oscSource | oscPinFunction | oscFreqOut;
	i2cDevice.I2CWriteByte(REG_CLOCK, regClock);

	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
	oscDivider = constrain(oscDivider, 1, 7);
	_clkX = 2000000.0 / (1<<(oscDivider - 1)); // Update private clock variable
	oscDivider = (oscDivider & 0b111)<<4;	// 3-bit value, bits 6:4

	uint8_t regMisc = i2cDevice.I2CReadByte(REG_MISC);
	regMisc &= ~(0b111<<4);
	regMisc |= oscDivider;
	i2cDevice.I2CWriteByte(REG_MISC, regMisc);
}

uint8_t SX1509::calculateLEDTRegister(int ms)
{
	int regOn1, regOn2;
	float timeOn1, timeOn2;

	if (_clkX == 0)
		return 0;

	regOn1 = (float) (ms / 1000.0) / (64.0 * 255.0 / (float) _clkX);
	regOn2 = regOn1 / 8;
	regOn1 = constrain(regOn1, 1, 15);
	regOn2 = constrain(regOn2, 16, 31);

	timeOn1 = 64.0 * regOn1 * 255.0 / _clkX * 1000.0;
	timeOn2 = 512.0 * regOn2 * 255.0 / _clkX * 1000.0;

	if (abs(timeOn1 - ms) < abs(timeOn2 - ms))
		return regOn1;
	else
		return regOn2;
}

uint8_t SX1509::calculateSlopeRegister(int ms, uint8_t onIntensity, uint8_t offIntensity)
{
	int regSlope1, regSlope2;
	float regTime1, regTime2;

	if (_clkX == 0)
		return 0;

	float tFactor = ((float) onIntensity - (4.0 * (float)offIntensity)) * 255.0 / (float) _clkX;
	float timeS = float(ms) / 1000.0;

	regSlope1 = timeS / tFactor;
	regSlope2 = regSlope1 / 16;

	regSlope1 = constrain(regSlope1, 1, 15);
	regSlope2 = constrain(regSlope2, 16, 31);

	regTime1 = regSlope1 * tFactor * 1000.0;
	regTime2 = 16 * regTime1;

	if (abs(regTime1 - ms) < abs(regTime2 - ms))
		return regSlope1;
	else
		return regSlope2;
}
//
// contrain value to range
//
uint8_t SX1509::constrain(uint8_t val, uint8_t  low_limit, uint8_t high_limit)
{
    val = val < low_limit ? low_limit : val;
    val = val > high_limit ? high_limit : val;
    return val;
}
