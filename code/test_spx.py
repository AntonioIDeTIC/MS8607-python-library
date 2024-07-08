import libSPX
import SPX16298_regs
import pigpio
import time
import numpy as np

gpio_channel = 1
gpio_handler = pigpio.pi()

PT_Sensor = libSPX.PT_Sensor("Pressure & Temperature sensor", gpio_channel, SPX16298_regs.MS8607_PSENSOR_ADDR,
                             gpio_handler)
H_Sensor = libSPX.H_Sensor("Humidity sensor", gpio_channel, SPX16298_regs.MS8607_HSENSOR_ADDR, gpio_handler)

# Pressure and Temperature
temperature = PT_Sensor.getTemperature()
base_pressure = PT_Sensor.getPressure() / 100

starttime = time.time()
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})

while True:
    currtime = time.time()
    try:
        temperature = PT_Sensor.getTemperature()
        pressure = PT_Sensor.getPressure()
        temperature, pressure = PT_Sensor.secondOrderCorrection()
        seaLevel_pressure = PT_Sensor.adjustToSeaLevel() / 100
        temperature, pressure = temperature / 100, pressure / 100

        RH = H_Sensor.getHumidity()
        RH_compensated = H_Sensor.getCompensatedHumidity(temperature)

        if currtime - starttime >= 0.5:
            print("Temperature: {:0.2f} ºC ".format(temperature))
            print("Presure: {:0.2f} mbar ".format(pressure))
            print("Sea Level Presure: {:0.2f} mbar ".format(seaLevel_pressure))
            print("Humidity: {:0.2f} % ".format(RH_compensated))
            print("\n")

            starttime = currtime
    except KeyboardInterrupt:
        break

PT_Sensor.stop()
H_Sensor.stop()
