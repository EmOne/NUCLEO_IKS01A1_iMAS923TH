#include "cayenne_lpp.h"
#include <string.h>
#include <stdlib.h>

Cayenne_t cayenne_payload;

void CayenneLPP_Init(uint8_t size)
//Initialize the payload buffer with the given maximum size.
{
    cayenne_payload.buffer = (uint8_t*) malloc(size);
    cayenne_payload.maxsize = size;
    cayenne_payload_reset();
}

void CayenneLPP_Deinit(void)
{
    free(cayenne_payload.buffer);
}

void cayenne_payload_reset(void)
//Reset the payload, to call before building a frame payload
{
    cayenne_payload.cursor = 0;
}

uint8_t cayenne_payload_getSize(void)
//Returns the current size of the payload
{
    return cayenne_payload.cursor;
}

uint8_t* cayenne_payload_getBuffer(void)
//Return the payload buffer
{
    return cayenne_payload.buffer;
}

uint8_t cayenne_payload_copy(uint8_t* dst)
{
    memcpy(dst, cayenne_payload.buffer, cayenne_payload.cursor);
    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addDigitalInput(uint8_t channel, uint8_t value)
{
    if ((cayenne_payload.cursor + LPP_DIGITAL_INPUT_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_DIGITAL_INPUT;
    cayenne_payload.buffer[cayenne_payload.cursor++] = value;
    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addDigitalOutput(uint8_t channel, uint8_t value)
{
    if ((cayenne_payload.cursor + LPP_DIGITAL_OUTPUT_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_DIGITAL_OUTPUT;
    cayenne_payload.buffer[cayenne_payload.cursor++] = value;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addAnalogInput(uint8_t channel, float value)
{
    if ((cayenne_payload.cursor + LPP_ANALOG_INPUT_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }

    int16_t val = value * 100;
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_ANALOG_INPUT;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addAnalogOutput(uint8_t channel, float value)
{
    if ((cayenne_payload.cursor + LPP_ANALOG_OUTPUT_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int16_t val = value * 100;
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_ANALOG_OUTPUT;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addLuminosity(uint8_t channel, float lux)
{
    if ((cayenne_payload.cursor + LPP_LUMINOSITY_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }

    int16_t l = lux;

    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_LUMINOSITY;
    cayenne_payload.buffer[cayenne_payload.cursor++] = l >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = l;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addPresence(uint8_t channel, uint8_t value)
{
    if ((cayenne_payload.cursor + LPP_PRESENCE_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_PRESENCE;
    cayenne_payload.buffer[cayenne_payload.cursor++] = value;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addTemperature(uint8_t channel, float celsius)
{
    if ((cayenne_payload.cursor + LPP_TEMPERATURE_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int16_t val = celsius * 10;
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_TEMPERATURE;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addRelativeHumidity(uint8_t channel, float rh)
{
    if ((cayenne_payload.cursor + LPP_RELATIVE_HUMIDITY_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }

    uint8_t val = rh;
    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_RELATIVE_HUMIDITY;
//    cayenne_payload.buffer[cayenne_payload.cursor++] = rh * 2;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addAccelerometer(uint8_t channel, int32_t x, int32_t y, int32_t z)
{
    if ((cayenne_payload.cursor + LPP_ACCELEROMETER_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int16_t vx = x * 1000;
    int16_t vy = y * 1000;
    int16_t vz = z * 1000;

    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_ACCELEROMETER;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vx >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vx;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vy >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vy;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vz >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vz;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addBarometricPressure(uint8_t channel, float hpa)
{
    if ((cayenne_payload.cursor + LPP_BAROMETRIC_PRESSURE_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int16_t val = hpa / 100;

    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_BAROMETRIC_PRESSURE;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = val;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addGyrometer(uint8_t channel, int32_t x, int32_t y, int32_t z)
{
    if ((cayenne_payload.cursor + LPP_GYROMETER_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int16_t vx = x * 100;
    int16_t vy = y * 100;
    int16_t vz = z * 100;

    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_GYROMETER;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vx >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vx;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vy >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vy;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vz >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vz;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addMagnetometer(uint8_t channel, int32_t x, int32_t y, int32_t z)
{
    if ((cayenne_payload.cursor + LPP_MAGNETOMETER_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int16_t vx = x * 100;
    int16_t vy = y * 100;
    int16_t vz = z * 100;

    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_MAGNETOMETER;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vx >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vx;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vy >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vy;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vz >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = vz;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addGPS(uint8_t channel, float latitude, float longitude, float meters)
{
    if ((cayenne_payload.cursor + LPP_GPS_SIZE) > cayenne_payload.maxsize) {
        return 0;
    }
    int32_t lat = latitude * 10000;
    int32_t lon = longitude * 10000;
    int32_t alt = meters * 100;

    cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
    cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_GPS;

    cayenne_payload.buffer[cayenne_payload.cursor++] = lat >> 16;
    cayenne_payload.buffer[cayenne_payload.cursor++] = lat >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = lat;
    cayenne_payload.buffer[cayenne_payload.cursor++] = lon >> 16;
    cayenne_payload.buffer[cayenne_payload.cursor++] = lon >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = lon;
    cayenne_payload.buffer[cayenne_payload.cursor++] = alt >> 16;
    cayenne_payload.buffer[cayenne_payload.cursor++] = alt >> 8;
    cayenne_payload.buffer[cayenne_payload.cursor++] = alt;

    return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addVoltage(uint8_t channel, float voltage)
{
  if ((cayenne_payload.cursor + LPP_VOLTAGE_SIZE) > cayenne_payload.maxsize) {
      return 0;
  }

  int16_t val = voltage * 10;
  cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
  cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_VOLTAGE;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val;

  return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addCurrent(uint8_t channel, float current)
{
  if ((cayenne_payload.cursor + LPP_CURRENT_SIZE) > cayenne_payload.maxsize) {
      return 0;
  }

  int16_t val = current * 1000;
  cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
  cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_CURRENT;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val;

  return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addPower(uint8_t channel, float power)
{
  if ((cayenne_payload.cursor + LPP_POWER_SIZE) > cayenne_payload.maxsize) {
      return 0;
  }
  int16_t val = power * 10;
  cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
  cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_POWER;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val;

  return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addEnergy(uint8_t channel, float energy)
{
  if ((cayenne_payload.cursor + LPP_ENERGY_SIZE) > cayenne_payload.maxsize) {
      return 0;
  }

  int16_t val = energy * 1000;
  cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
  cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_ENERGY;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val >> 8;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val;

  return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addPercentage(uint8_t channel, uint8_t percentage)
{
  if ((cayenne_payload.cursor + LPP_PERCENTAGE_SIZE) > cayenne_payload.maxsize) {
      return 0;
  }

  int8_t val = (percentage / 255.0f) * 100;
  cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
  cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_PERCENTAGE;
  cayenne_payload.buffer[cayenne_payload.cursor++] = val;

  return cayenne_payload.cursor;
}

uint8_t cayenne_payload_addErrorCode(uint8_t channel, uint32_t err)
{
  if ((cayenne_payload.cursor + LPP_ERR_CODE_SIZE) > cayenne_payload.maxsize) {
      return 0;
  }

  cayenne_payload.buffer[cayenne_payload.cursor++] = channel;
  cayenne_payload.buffer[cayenne_payload.cursor++] = LPP_ERR_CODE;
  cayenne_payload.buffer[cayenne_payload.cursor++] = err >> 24;
  cayenne_payload.buffer[cayenne_payload.cursor++] = err >> 16;
  cayenne_payload.buffer[cayenne_payload.cursor++] = err >> 8;
  cayenne_payload.buffer[cayenne_payload.cursor++] = err;

  return cayenne_payload.cursor;
}
