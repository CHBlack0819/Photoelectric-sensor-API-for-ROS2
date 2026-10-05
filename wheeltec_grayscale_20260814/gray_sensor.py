#gray_sensor.py
#Last edited by Chrono Black at 20260804
import sys
import signal
import os
import time
import serial
import serial.tools.list_ports

# the start up code of UART devices, in which the first byte refers to start up sign
# while the second one the address of your device.
# Please refer to the datasheet of your device if you want to modify them.
start_up = bytes([0x57, 0x01])

def signal_handler(signal, frame):
    print("\nExiting...")
    sys.exit(0)

class gray_sensor:
# the list of member variables
    __uart_dev = None # the number of the serial
    __baudrate = None # baudrate
    __timeout = None  # maximum time to wait for requests from slavery
    __msg_send = None 
    __msg_recv = None
    __ser = None # the UART server(?)
    __is_activated = False #state code

# the constructor of the sensor

    def __init__(self, uart_dev: str, baudrate: int = 115200, timeout: float = 0.5):
        self.__uart_dev = uart_dev
        self.__baudrate = baudrate
        self.__timeout = timeout
        self.__init_serial()

# @brief: this function launches the serial connection.
# @retval: 0: successfully launched; 1: failed to launch
# @params:
#       uart_dev: string typed, used to nominate the serial port you want to activate.
#       baudrate: int typed, the baudrate of your serial.
#       timeout: float typed, the maximum time the host can stand waiting.
    def __init_serial(self):
        try:
            self.__ser = serial.Serial(
                port=self.__uart_dev,
                baudrate=self.__baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self.__timeout
            )
            self.__is_activated = True
            print(f"[I] Serial opened on {self.__ser.name} at the baudrate of " + str(self.__baudrate))
            self.__ser.write(start_up)
        except Exception as e:
            print(f"[E] Failed to open serial: {e}")
            raise

# @brief: this function fetches data from the sensor.
# @retval: a list containing the data that indicates which sensors saw white color(marked as 0)
#          and which saw dark colors(marked as 1)
# @params: self: the sensor object itself

    def serial_recv(self):
        res = []
        if not self.__is_activated:
            print("Serial not opened.\n")
            raise

        self.__msg_send = self.__ser.write(start_up)
        self.__msg_recv = self.__ser.read(3)
        if self.__msg_recv:
            if self.__msg_recv[0] == 0x75 and self.__msg_recv[2] == 0x02:
                data = self.__msg_recv[1]
                for i in range(0,7,1):
                    res.append((data >> i) & 1)
        return res

# @brief: this function shuts down the serial.
# @retval: 0: successfully closed the port; 1: failed to shut down the port
# @params: self: the sensor object itself
    def serial_close(self):
        try:
            self.__ser.close()
        except Exception as e:
            printf(f"fail to shut down the serial port: {e}")
            raise
        return 0
                
#sensor = gray_sensor("/dev/ttyS1", 115200, 0.5)

#while True:
#    #sensor = gray_sensor("/dev/ttyS1", 115200, 0.5)
#    print(sensor.serial_recv())
#    time.sleep(0.5)
    

