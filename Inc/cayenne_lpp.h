#ifndef __CAYENNE_LPP_H
#define __CAYENNE_LPP_H

#include <stdint.h>

typedef enum {
  LPP_CHANNEL_LUX = 0x00,
  LPP_CHANNEL_OUTER_TEMPERATURE = 0x01,
  LPP_CHANNEL_HUMIDITY = 0x02,
  LPP_CHANNEL_BAR_PRESSURE = 0x03,
  LPP_CHANNEL_GPS = 0x04,
  LPP_CHANNEL_VOLTAGE = 0x05,
  LPP_CHANNEL_CURRENT = 0x06,
  LPP_CHANNEL_POWER_FACTOR = 0x07,
  LPP_CHANNEL_POWER_CONSUMP = 0x08,
  LPP_CHANNEL_BATTERY_LVL = 0x09,
  LPP_CHANNEL_RAIN = 0x0A,
  LPP_CHANNEL_RAIN_LVL = 0x0B,
  LPP_CHANNEL_ENERGY = 0x0C,
  LPP_CHANNEL_POWER_PANEL = 0x0D,
  LPP_CHANNEL_INNER_TEMPERATURE = 0x0E,
  LPP_CHANNEL_ERR_CODE = 0x0F,
  LPP_CHANNEL_DIRECTION = 0x10,
  LPP_CHANNEL_ACCELERO = 0x11,
  LPP_CHANNEL_GYRO = 0x12,
  LPP_CHANNEL_MAGNETO = 0x13,
  LPP_CHANNEL_UNKNOWN = 0xFF
} LPP_CHANNEL_e;


#define LPP_DIGITAL_INPUT       0       // 1 byte
#define LPP_DIGITAL_OUTPUT      1       // 1 byte
#define LPP_ANALOG_INPUT        2       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT       3       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE		102     // 1 byte, 1
#define LPP_TEMPERATURE         103     // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY   104     // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER       113     // 2 bytes per axis, 0.001G
#define LPP_MAGNETOMETER           114     // 2 bytes per axis, 0.01 flux
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned
#define LPP_VOLTAGE		116     // 2 bytes 0.1 V signed
#define LPP_CURRENT		117     // 2 bytes 0.1 A signed
#define LPP_PERCENTAGE		120     // 1 bytes 1 unsigned
//#define LPP_PRESSURE 123     // 2 bytes 0.1 hPa Unsigned
#define LPP_POWER		128     // 2 bytes 2 unsigned
#define LPP_ENERGY		131     // 2 bytes 2 unsigned
#define LPP_DIRECTION		132     // 1 bytes 1 unsigned
#define LPP_GYROMETER           134     // 2 bytes per axis, 0.01 °/s

#define LPP_GPS                 136     // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01m
//#define LPP_POSITIONER   137     // 1 bytes
//#define LPP_UP_DOWN_CONTROL   144     // 1 bytes
#define LPP_ERR_CODE		255

// Data ID + Data Type + Data Size
#define LPP_DIGITAL_INPUT_SIZE       3
#define LPP_DIGITAL_OUTPUT_SIZE      3
#define LPP_ANALOG_INPUT_SIZE        4
#define LPP_ANALOG_OUTPUT_SIZE       4
#define LPP_LUMINOSITY_SIZE          4
#define LPP_PRESENCE_SIZE            3
#define LPP_TEMPERATURE_SIZE         4
#define LPP_RELATIVE_HUMIDITY_SIZE   3
#define LPP_ACCELEROMETER_SIZE       8
#define LPP_BAROMETRIC_PRESSURE_SIZE 4
#define LPP_GYROMETER_SIZE           8
#define LPP_MAGNETOMETER_SIZE        8
#define LPP_GPS_SIZE                 11
#define LPP_VOLTAGE_SIZE             4
#define LPP_CURRENT_SIZE             4
#define LPP_POWER_SIZE		     4
#define LPP_PERCENTAGE_SIZE	     3
#define LPP_ENERGY_SIZE		     4
#define LPP_ERR_CODE_SIZE	     6

void CayenneLPP_Init(uint8_t size);
void CayenneLPP_Deinit(void);

void cayenne_payload_reset(void);
uint8_t cayenne_payload_getSize(void);
uint8_t* cayenne_payload_getBuffer(void);
uint8_t cayenne_payload_copy(uint8_t* buffer);

uint8_t cayenne_payload_addDigitalInput(uint8_t channel, uint8_t value);
uint8_t cayenne_payload_addDigitalOutput(uint8_t channel, uint8_t value);

uint8_t cayenne_payload_addAnalogInput(uint8_t channel, float value);
uint8_t cayenne_payload_addAnalogOutput(uint8_t channel, float value);

uint8_t cayenne_payload_addLuminosity(uint8_t channel, float lux);
uint8_t cayenne_payload_addPresence(uint8_t channel, uint8_t value);
uint8_t cayenne_payload_addTemperature(uint8_t channel, float celsius);
uint8_t cayenne_payload_addRelativeHumidity(uint8_t channel, float rh);
uint8_t cayenne_payload_addAccelerometer(uint8_t channel, int32_t x, int32_t y, int32_t z);
uint8_t cayenne_payload_addBarometricPressure(uint8_t channel, float hpa);
uint8_t cayenne_payload_addGyrometer(uint8_t channel, int32_t x, int32_t y, int32_t z);
uint8_t cayenne_payload_addMagnetometer(uint8_t channel, int32_t x, int32_t y, int32_t z);
uint8_t cayenne_payload_addGPS(uint8_t channel, float latitude, float longitude, float meters);
uint8_t cayenne_payload_addVoltage(uint8_t channel, float voltage);
uint8_t cayenne_payload_addCurrent(uint8_t channel, float current);
uint8_t cayenne_payload_addPower(uint8_t channel, float power);
uint8_t cayenne_payload_addEnergy(uint8_t channel, float energy);
uint8_t cayenne_payload_addPercentage(uint8_t channel, uint8_t value);
uint8_t cayenne_payload_addErrorCode(uint8_t channel, uint32_t value);

typedef struct {
  uint8_t *buffer;
  uint8_t maxsize;
  uint8_t cursor;
} Cayenne_t;

extern Cayenne_t cayenne_payload;

#endif /* __CHARGER_H */
