# Psensor conversion timings
PSENSOR_CONVERSION_TIME_OSR_256 = 0.001
PSENSOR_CONVERSION_TIME_OSR_512 = 0.002
PSENSOR_CONVERSION_TIME_OSR_1024 = 0.003
PSENSOR_CONVERSION_TIME_OSR_2048 = 0.005
PSENSOR_CONVERSION_TIME_OSR_4096 = 0.009
PSENSOR_CONVERSION_TIME_OSR_8192 = 0.018

SENSOR_RESET_TIME = 0.015

# Processing constants
HSENSOR_TEMPERATURE_COEFFICIENT = -0.15
HSENSOR_CONSTANT_A = 8.1332
HSENSOR_CONSTANT_B = 1762.39
HSENSOR_CONSTANT_C = 235.66

DEW_POINT_CONSTANC_A = 17.271
DEW_POINT_CONSTANC_B = 237.7  # degC

# Coefficients for relative humidity computation
HUMIDITY_COEFF_MUL = 125
HUMIDITY_COEFF_ADD = -6

# Humidity sensor conversion timings
HSENSOR_CONVERSION_TIME_12b = 0.016
HSENSOR_CONVERSION_TIME_11b = 0.009
HSENSOR_CONVERSION_TIME_10b = 0.005
HSENSOR_CONVERSION_TIME_8b = 0.003

# PSENSOR device address
MS8607_PSENSOR_ADDR = 0x76;  # 0b1110110

# PSENSOR device commands
PSENSOR_RESET_COMMAND = 0x1E
PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_256 = 0x40
PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_512 = 0x42
PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_1024 = 0x44
PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_2048 = 0x46
PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_4096 = 0x48
PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_8192 = 0x4A

PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_256 = 0x50
PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_512 = 0x52
PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_1024 = 0x54
PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_2048 = 0x56
PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_4096 = 0x58
PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_8192 = 0x5A

PSENSOR_READ_ADC = 0x00
PSENSOR_CONVERSION_OSR_MASK = 0x0F

# PSENSOR commands
PROM_ADDRESS_READ_ADDRESS_0 = 0xA0
PROM_ADDRESS_READ_ADDRESS_1 = 0xA2
PROM_ADDRESS_READ_ADDRESS_2 = 0xA4
PROM_ADDRESS_READ_ADDRESS_3 = 0xA6
PROM_ADDRESS_READ_ADDRESS_4 = 0xA8
PROM_ADDRESS_READ_ADDRESS_5 = 0xAA
PROM_ADDRESS_READ_ADDRESS_6 = 0xAC
PROM_ADDRESS_READ_ADDRESS_7 = 0xAE

CRC_INDEX = 0

# HSENSOR device address
MS8607_HSENSOR_ADDR = 0x40;  # 0b1000000

# HSENSOR device commands
HSENSOR_RESET_COMMAND = 0xFE
HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND = 0xE5
HSENSOR_READ_HUMIDITY_NO_HOLD_COMMAND = 0xF5
HSENSOR_READ_SERIAL_FIRST_8BYTES_COMMAND = 0xFA0F
HSENSOR_READ_SERIAL_LAST_6BYTES_COMMAND = 0xFCC9
HSENSOR_WRITE_USER_REG_COMMAND = 0xE6
HSENSOR_READ_USER_REG_COMMAND = 0xE7

# PSENSOR commands
PROM_ADDRESS_READ_ADDRESS_0 = 0xA0
PROM_ADDRESS_READ_ADDRESS_1 = 0xA2
PROM_ADDRESS_READ_ADDRESS_2 = 0xA4
PROM_ADDRESS_READ_ADDRESS_3 = 0xA6
PROM_ADDRESS_READ_ADDRESS_4 = 0xA8
PROM_ADDRESS_READ_ADDRESS_5 = 0xAA
PROM_ADDRESS_READ_ADDRESS_6 = 0xAc

# HTU User Register values
# Resolution
HSENSOR_USER_REG_RESOLUTION_12b = 0x00
HSENSOR_USER_REG_RESOLUTION_11b = 0x81
HSENSOR_USER_REG_RESOLUTION_10b = 0x80
HSENSOR_USER_REG_RESOLUTION_8b = 0x01
HSENSOR_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V = 0x00
HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V = 0x40
HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE = 0x04
