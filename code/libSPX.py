import SPX16298_regs
import time


# A SPX16298 class with some methods
class SPX16298:
    # init method or constructor
    def __init__(self, debug_name, channel, address, gpio_handler):
        self.debug_name = debug_name
        self.channel = channel
        self.address = address
        self.is_connected = None

        self.gpio_handler = gpio_handler
        self.handle = None

    """
       Debug function that prints the current connected SPX (pressure, temperature and humidity) sensor
       :param self: reference to the current instance of the class
       :return: None
    """

    def debug_connection_message(self):
        print(self.debug_name + " connected at --> " + str(
            hex(self.address) + " through I2C channel " + str(self.channel)))
        time.sleep(0.1)


class PT_Sensor(SPX16298):
    def __init__(self, debug_name, channel, address, gpio_handler, localAltitude=0):
        SPX16298.__init__(self, debug_name, channel, address, gpio_handler)

        self.pressure_resolution = None
        self.pressure_conversion_time = None
        self.temperature_conversion_time = None

        self.C1 = None
        self.C2 = None
        self.C3 = None
        self.C4 = None
        self.C5 = None
        self.C6 = None

        self.D1 = None
        self.D2 = None

        self.pressure = None
        self.localAltitude = localAltitude
        self.seaLevel_pressure = None

        self.temperature = None
        self.dT = None

        try:
            self.pressure_resolution = SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_8192
            self.temperature_resolution = SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_8192
            self.handle = self.gpio_handler.i2c_open(self.channel, self.address)  # open i2c bus
            self.gpio_handler.i2c_write_byte(self.handle, SPX16298_regs.PSENSOR_RESET_COMMAND)  # send reset command
            time.sleep(SPX16298_regs.SENSOR_RESET_TIME)  # 15 mseg
            self.is_connected = True
            self.debug_connection_message()
            self.psensorReadADC()
            self.psensorReadPROM()

        except TypeError:
            pass

    """
       Function that stops the current connected SPX (pressure, temperature) sensor
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :return: sensor disconnected confirmation
    """

    def stop(self):
        if self.is_connected:
            self.gpio_handler.i2c_close(self.handle)  # close i2c bus
            self.is_connected = False
        else:
            print("Pressure & Temperature sensor not connected !!")
            pass

    """
       Function that begins the ADC conversion of the Pressure sensor
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :param resolution: Pressure sensor resolution
       :return: conversion_time according to the official datasheet 
    """

    def startPressureConversion(self):
        try:
            if self.is_connected:
                self.gpio_handler.i2c_write_byte(self.handle, self.pressure_resolution)
                if self.pressure_resolution == SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_256:
                    self.pressure_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_256
                elif self.pressure_resolution == SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_512:
                    self.pressure_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_512
                elif self.pressure_resolution == SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_1024:
                    self.pressure_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_1024
                elif self.pressure_resolution == SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_2048:
                    self.pressure_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_2048
                elif self.pressure_resolution == SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_4096:
                    self.pressure_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_4096
                elif self.pressure_resolution == SPX16298_regs.PSENSOR_START_PRESSURE_ADC_CONVERSION_OSR_8192:
                    self.pressure_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_8192
                else:
                    print("You need to select the sensor resolution !!")
                    pass
            else:
                print("Pressure & Temperature sensor not connected !!")
                pass

        except TypeError:
            pass
        except Exception as e:
            raise Exception('This is the exception you expect to handle')
            print('Caught this error: ' + repr(e))

    """
      Function that begins the ADC conversion of the Temperature sensor
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param sensor_is_connected: sensor connected confirmation
      :param resolution: Temperature  sensor resolution
      :return: conversion_time according to the official datasheet 
   """

    def startTemperatureConversion(self):
        try:
            if self.is_connected:
                self.gpio_handler.i2c_write_byte(self.handle, self.temperature_resolution)
                if self.temperature_resolution == SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_256:
                    self.temperature_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_256
                elif self.temperature_resolution == SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_512:
                    self.temperature_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_512
                elif self.temperature_resolution == SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_1024:
                    self.temperature_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_1024
                elif self.temperature_resolution == SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_2048:
                    self.temperature_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_2048
                elif self.temperature_resolution == SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_4096:
                    self.temperature_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_4096
                elif self.temperature_resolution == SPX16298_regs.PSENSOR_START_TEMPERATURE_ADC_CONVERSION_OSR_8192:
                    self.temperature_conversion_time = SPX16298_regs.PSENSOR_CONVERSION_TIME_OSR_8192
                else:
                    print("You need to select the sensor resolution !!")
                    pass
            else:
                print("Pressure & Temperature sensor not connected !!")
                pass

        except TypeError:
            pass

    """
      Function that reads the PROM Pressure & Temperature sensor memory 
      :param self: reference to the current instance of the class
      :param handle: current communication handle
      :param sensor_is_connected: sensor connected confirmation
      :param resolution: Temperature  sensor resolution
      :return: C1, C2, C3, C4, C5, C6 coefficients of PROM memory
    """

    def psensorReadPROM(self):
        if self.is_connected:
            # Read the PROM for calibrating parameters
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_1, 2)
            self.C1 = int.from_bytes(byteArray, byteorder="big", signed=False)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_2, 2)
            self.C2 = int.from_bytes(byteArray, byteorder="big", signed=False)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_3, 2)
            self.C3 = int.from_bytes(byteArray, byteorder="big", signed=False)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_4, 2)
            self.C4 = int.from_bytes(byteArray, byteorder="big", signed=False)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_5, 2)
            self.C5 = int.from_bytes(byteArray, byteorder="big", signed=False)
            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_6, 2)
            self.C6 = int.from_bytes(byteArray, byteorder="big", signed=False)

            _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle,
                                                                     SPX16298_regs.PROM_ADDRESS_READ_ADDRESS_0, 2)
            crc_eeprom = int.from_bytes(byteArray, byteorder="big", signed=False)

            eeprom = [crc_eeprom, self.C1, self.C2, self.C3, self.C4, self.C5, self.C6]

            if ~self.psensorCRC_check(eeprom):
                pass
            else:
                print("CRC checking status --> error")
                time.sleep(0.1)
                pass
        else:
            print("Pressure & Temperature sensor not connected !!")
            pass

    """
       Function that performs the Pressure & Temperature CRC
       :param self: reference to the current instance of the class
       :param n_prom: PROM coefficients
       :return: CRC status
    """

    def psensorCRC_check(self, n_prom):
        n_rem = 0x00
        crc_read = n_prom[0]
        n_prom[6] = 0
        n_prom[0] = 0x0FFF & (n_prom[0])  # Clear the CRC byte

        for cnt in range(0, 13):
            # Get next byte
            if cnt % 2 == 1:
                n_rem ^= n_prom[cnt >> 1] & 0x00FF
            else:
                n_rem ^= n_prom[cnt >> 1] >> 8

            for _ in range(8, -1, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem <<= 1

        n_rem = ((n_rem >> 12) & 0x000F) ^ 0x00
        if n_rem == crc_read >> 12:
            status = True
        else:
            status = False

        return status

    """
       Function that reads the ADC conversion of the Pressure & Temperature sensor
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :param pressure_resolution: desired pressure resolution
       :param temperature_resolution: desired temperature resolution
       :return: D1 and D2 coefficients for further calculations 
    """

    def psensorReadADC(self):
        try:
            if self.is_connected:
                # Read the ADC 
                self.startPressureConversion()
                time.sleep(self.pressure_conversion_time)
                _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, SPX16298_regs.PSENSOR_READ_ADC, 3)
                self.D1 = int.from_bytes(byteArray, byteorder="big", signed=False)

                self.startTemperatureConversion()
                time.sleep(self.temperature_conversion_time)
                _, byteArray = self.gpio_handler.i2c_read_i2c_block_data(self.handle, SPX16298_regs.PSENSOR_READ_ADC, 3)
                self.D2 = int.from_bytes(byteArray, byteorder="big", signed=False)

            else:
                print("Pressure & Temperature sensor not connected !!")
                pass

        except TypeError:
            pass

    """
       Function that calculates the current temperature
       :param self: reference to the current instance of the class
       :param D2: D2 coefficient
       :param C5: D2 coefficient
       :param C6: C6 coefficient
       :return: temperature and temperature delta
    """

    def getTemperature(self):
        self.psensorReadADC()
        self.dT = self.D2 - self.C5 * 2 ** 8  # Difference between actual and reference temperature
        self.temperature = 2000 + self.dT * self.C6 / 2 ** 23  # Actual temperature (-40…85°C with 0.01°C resolution)

        return self.temperature

    """
       Function that calculates the current pressure
       :param self: reference to the current instance of the class
       :param D1: D1 coefficient
       :param C2: C2 coefficient
       :param C3: C3 coefficient
       :param C4: C4 coefficient
       :param dT: temperature delta
       :return: current pressure
    """

    def getPressure(self):
        self.psensorReadADC()
        OFF = self.C2 * 2 ** 17 + (self.C4 * self.dT) / 2 ** 6  # Offset at actual temperature
        SENS = self.C1 * 2 ** 16 + (self.C3 * self.dT) / 2 ** 7  # Sensitivity at actual temperature
        self.pressure = (self.D1 * SENS / 2 ** 21 - OFF) / 2 ** 15

        return self.pressure

    """
       Function that calculates the second order correction according to the datasheet
       :param self: reference to the current instance of the class
       :param D1: D1 coefficient
       :param C2: C2 coefficient
       :param C3: C3 coefficient
       :param C4: C4 coefficient
       :param temperature: temperature previously calculate
       :param dT: temperature delta
       :return: corrected pressure and temperature
    """

    def secondOrderCorrection(self):
        OFF = self.C2 * 2 ** 17 + (self.C4 * self.dT) / 2 ** 6  # Offset at actual temperature
        SENS = self.C1 * 2 ** 16 + (self.C3 * self.dT) / 2 ** 7  # Sensitivity at actual temperature
        if self.temperature < 2000:
            # Low temperature
            T2 = 3 * self.dT ** 2 / 2 ** 33
            OFF2 = 61 * (self.temperature - 2000) ** 2 / 2 ** 4
            SENS2 = 29 * (self.temperature - 2000) ** 2 / 2 ** 4

            if self.temperature < - 15.0:
                OFF2 = OFF2 + 17 * (self.temperature + 1500) ** 2
                SENS2 = SENS2 + 9 * (self.temperature + 1500) ** 2

            else:
                self.temperature = self.temperature - T2
                OFF = OFF - OFF2
                SENS = SENS - SENS2
                self.pressure = (
                                        self.D1 * SENS / 2 ** 21 - OFF) / 2 ** 15  # Temperature compensated pressure (10…1200mbar with 0.01mbar resolution)

        else:
            T2 = 5 * self.dT ** 2 / 2 ** 38
            OFF2 = 0
            SENS2 = 0
            self.temperature = self.temperature - T2
            OFF = OFF - OFF2
            SENS = SENS - SENS2
            self.pressure = (
                                    self.D1 * SENS / 2 ** 21 - OFF) / 2 ** 15  # Temperature compensated pressure (10…1200mbar with 0.01mbar resolution)

        return self.temperature, self.pressure

    """
       Function that calculates the pressure according to the sea level
       :param self: reference to the current instance of the class
       :param pressure: pressure previosly calculated
       :param localAltitude: current altitude
       :return: corrected pressure referenced to sea level
    """

    def adjustToSeaLevel(self):
        return self.pressure / (1 - (self.localAltitude / 44330.0)) ** 5.255

    """
       Function that calculates current altitude change
       :param self: reference to the current instance of the class
       :param currentPressure: current pressure
       :param startingPressure: starting pressure 
       :return: current altitude change
    """

    def altitudeChange(self, currentPressure, startingPressure):
        return abs(44330.0 * (1 - ((currentPressure / startingPressure) ** (1 / 5.255))))


class H_Sensor(SPX16298):
    def __init__(self, debug_name, channel, address, gpio_handler):
        SPX16298.__init__(self, debug_name, channel, address, gpio_handler)

        self.humidity_resolution = None
        self.humidity_conversion_time = None

        self.D3 = None
        self.status_reg = None
        self.checksum = None

        self.heater_status = None
        self.enable_heater = None
        self.humidity = None
        self.compensated_humidity = None
        self.dew_point = None

        try:
            self.humidity_resolution = SPX16298_regs.HSENSOR_USER_REG_RESOLUTION_12b
            self.handle = self.gpio_handler.i2c_open(self.channel, self.address)  # open i2c bus
            self.gpio_handler.i2c_write_byte(self.handle, SPX16298_regs.HSENSOR_RESET_COMMAND)  # send reset command
            time.sleep(SPX16298_regs.SENSOR_RESET_TIME)  # 15 mseg
            self.is_connected = True
            self.debug_connection_message()
            self.hsensorReadADC()
            self.getHeaterStatus()
        except TypeError:
            pass

    """
       Function that stops the current connected SPX (humidity) sensor
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :return: sensor disconnected confirmation
    """

    def stop(self):
        if self.is_connected:
            self.gpio_handler.i2c_close(self.handle)  # close i2c bus
            self.is_connected = False
        else:
            print("Humidity sensor not connected !!")
            pass

    """
       Function that reads the ADC conversion of the Humidity sensor
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :param humidity_resolution: desired humidity resolution
       :return: D3 coefficient for further calculations 
    """

    def hsensorReadADC(self):
        try:
            if self.is_connected:
                self.gpio_handler.i2c_write_byte_data(self.handle, SPX16298_regs.HSENSOR_WRITE_USER_REG_COMMAND,
                                                      self.humidity_resolution)  # maximum
                if self.humidity_resolution == SPX16298_regs.HSENSOR_USER_REG_RESOLUTION_12b:
                    self.humidity_conversion_time = SPX16298_regs.HSENSOR_CONVERSION_TIME_12b
                elif self.humidity_resolution == SPX16298_regs.HSENSOR_USER_REG_RESOLUTION_11b:
                    self.humidity_conversion_time = SPX16298_regs.HSENSOR_CONVERSION_TIME_11b
                elif self.humidity_resolution == SPX16298_regs.HSENSOR_USER_REG_RESOLUTION_10b:
                    self.humidity_conversion_time = SPX16298_regs.HSENSOR_CONVERSION_TIME_10b
                elif self.humidity_resolution == SPX16298_regs.HSENSOR_USER_REG_RESOLUTION_8b:
                    self.humidity_conversion_time = SPX16298_regs.HSENSOR_CONVERSION_TIME_8b
                else:
                    print("You need to select the sensor resolution !!")
                    pass

                self.gpio_handler.i2c_write_byte(self.handle,
                                                 SPX16298_regs.HSENSOR_READ_HUMIDITY_NO_HOLD_COMMAND)  # no hold command for communications
                time.sleep(self.humidity_conversion_time)
                _, byteArray = self.gpio_handler.i2c_read_device(self.handle, 3)  # vacuum up those bytes
                self.D3 = int.from_bytes(byteArray[0:2], byteorder="big", signed=False)  # msb, lsb, checksum
                self.checksum = byteArray[2]

                if self.hsensorCRC_check(self.checksum):
                    pass
                else:
                    print("CRC checking status --> error")
                    time.sleep(0.1)
                    pass
            else:
                print("Humidity sensor not connected !!")
                pass

        except TypeError:
            pass

    """
       Function that performs the Humidity CRC
       :param self: reference to the current instance of the class
       :param D3: D3 coefficient
       :param checksum: current checksum
       :return: CRC status
    """

    def hsensorCRC_check(self):

        polynom = 0x988000  # x^8 + x^5 + x^4 + 1
        msb = 0x800000
        mask = 0xFF8000
        result = self.D3 << 8  # Pad with zeros as specified in spec

        while msb != 0x80:

            # Check if msb of current value is 1 and apply XOR mask
            if result & msb:
                result = ((result ^ polynom) & mask) | (result & ~mask)

            # Shift by one
            msb >>= 1
            mask >>= 1
            polynom >>= 1

        if result == self.checksum:
            status = True
        else:
            status = False

        return status

    """
       Function that informs about the battery status of the the Humidity sensor
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :return: None
    """

    def getBatteryStatus(self):
        if self.is_connected:
            # Read user Register
            self.status_reg = self.gpio_handler.i2c_read_byte_data(self.handle,
                                                                   SPX16298_regs.HSENSOR_READ_USER_REG_COMMAND)
            if self.status_reg & SPX16298_regs.HSENSOR_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V == 0x00:
                print("Battery ok")
            elif self.status_reg & SPX16298_regs.HSENSOR_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V == 0x40:
                print("Battery low")
            else:
                pass
                # self.status_reg = int.from_bytes(byteArray, byteorder=byteorder = "big",  signed = False, signed=False) # msb, lsb, checksum
        else:
            print("Humidity sensor not connected !!")
            pass

    """
        Function that informs about the heater status of the the Humidity sensor (ON or OFF)
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param sensor_is_connected: sensor connected confirmation
        :return: heater status
    """

    def getHeaterStatus(self):
        if self.is_connected:
            # Read User Register
            self.status_reg = self.gpio_handler.i2c_read_byte_data(self.handle,
                                                                   SPX16298_regs.HSENSOR_READ_USER_REG_COMMAND)
            if self.status_reg & SPX16298_regs.HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE == 0x04:
                print("Heater enabled")
                self.heater_status = True
            else:
                print("Heater disabled")
                self.heater_status = False
            # self.status_reg = int.from_bytes(byteArray, byteorder=byteorder = "big",  signed = False, signed=False) # msb, lsb, checksum
        else:
            print("Humidity sensor not connected !!")
            pass

    """
        Function that enables or disables the heater status of the the Humidity sensor 
        :param self: reference to the current instance of the class
        :param handle: current communication handle
        :param sensor_is_connected: sensor connected confirmation
        :param enable: enable or disable order
        :return: None
    """

    def enableHeater(self, enable):
        if self.is_connected:
            self.status_reg = self.gpio_handler.i2c_read_byte_data(self.handle,
                                                                   SPX16298_regs.HSENSOR_READ_USER_REG_COMMAND)
            if enable:
                self.gpio_handler.i2c_write_byte_data(self.handle, SPX16298_regs.HSENSOR_WRITE_USER_REG_COMMAND,
                                                      self.status_reg | SPX16298_regs.HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE)
            else:
                self.gpio_handler.i2c_write_byte_data(self.handle, SPX16298_regs.HSENSOR_WRITE_USER_REG_COMMAND,
                                                      self.status_reg & ~SPX16298_regs.HSENSOR_USER_REG_ONCHIP_HEATER_ENABLE)
        else:
            print("Humidity sensor not connected !!")
            pass

    """
       Function that calculates the current humidity
       :param self: reference to the current instance of the class
       :param D3: D3 coefficient
       :return: relative humidity
    """

    def getHumidity(self):
        self.hsensorReadADC()
        self.humidity = SPX16298_regs.HUMIDITY_COEFF_ADD + SPX16298_regs.HUMIDITY_COEFF_MUL * self.D3 / 2 ** 16
        return self.humidity

    """
       Function that calculates the compensated humidity based in the current temperature
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :param RH: current relative humidity
       :param temperature: current temperature
       :param heater_status: heater status
       :return: compensated humidity
    """

    def getCompensatedHumidity(self, temperature):
        if self.heater_status:
            print("Disabling the heater for compute the compensated humidity")
            self.enableHeater(False)
            time.sleep(3)
        else:
            pass

        self.compensated_humidity = self.humidity + (25 - temperature) * SPX16298_regs.HSENSOR_TEMPERATURE_COEFFICIENT

        return self.compensated_humidity

    """
       Function that calculates the dew point 
       :param self: reference to the current instance of the class
       :param handle: current communication handle
       :param sensor_is_connected: sensor connected confirmation
       :param RH: current relative humidity
       :param temperature: current temperature
       :param heater_status: heater status
       :return: dew point
    """

    def getDewPoint(self, temperature):
        if self.heater_status:
            print("Disabling the heater for compute the dew point")
            self.enableHeater(False)
            time.sleep(3)
        else:
            pass

        # dew_point = temperature - ((100 - humidity) / 5)
        g = (SPX16298_regs.DEW_POINT_CONSTANC_A * temperature / (
                SPX16298_regs.DEW_POINT_CONSTANC_B + temperature)) + np.log(self.compensated_humidity / 100.0)

        self.dew_point = (SPX16298_regs.DEW_POINT_CONSTANC_B * g) / (
                SPX16298_regs.DEW_POINT_CONSTANC_A - g)

        return self.dew_point
