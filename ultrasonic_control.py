import serial
import time

class Ultrasonic:
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM1", 9600)
        print(self.ser.readline())

    def measure(self):
        self.ser.write(bytes("M", 'utf-8'))
        print("sent")
        result = self.ser.readline()
        print(result)
        return int(result) == 1

    def close(self):
        self.ser.close()

def test():
    ultra = Ultrasonic()
    count = 0
    while count < 20:
        print(ultra.measure())
        print("here")
        time.sleep(1)
        count+=1
    ultra.close()

if __name__ == "__main__":
    test()
