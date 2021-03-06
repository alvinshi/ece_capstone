import serial
import time

class UltrasonicAndLED:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM1", 9600, write_timeout=0)
        #self.ser = serial.Serial("/dev/ttyACM1", 9600)
        print(self.ser.readline())

    def measure(self):
        self.ser.write(bytes("M", 'utf-8'))
        result = self.ser.readline()
        try:
            value = int(result)
            return value == 1
        except ValueError:
            return False

    def validate(self):
        self.ser.write(bytes("V", 'utf-8'))
        result = self.ser.readline()
        try:
            value = int(result)
            return value == 1
        except ValueError:
            return False

    def to_wait(self):
        self.ser.write(bytes("W", 'utf-8'))

    def to_track(self):
        self.ser.write(bytes("T", 'utf-8'))

    def to_player_search(self):
        self.ser.write(bytes("P", 'utf-8'))

    def to_player_approach(self):
        self.ser.write(bytes("G", 'utf-8'))

    def to_return(self):
        self.ser.write(bytes("R", 'utf-8'))

    def close(self):
        self.ser.close()

def test():
    ultra = UltrasonicAndLED()
    time.sleep(2)
    ultra.to_track()
    time.sleep(2)
    ultra.to_wait()
    time.sleep(2)
    ultra.to_track()
    time.sleep(2)
    for x in range(10):
        ultra.measure()
        time.sleep(1)
    ultra.to_wait()


if __name__ == "__main__":
    test()
