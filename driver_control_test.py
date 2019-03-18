import serial

def main():
    ser = serial.Serial("/dev/tty.usbmodem1411", 9600)
    while True:
        input_string = input("type speed command here:")
        if input_string[0] == 'R':
            ser.write(bytes(input_string, 'utf-8'))
        elif input_string[0] == 'L':
            ser.write(bytes(input_string, 'utf-8'))
        else:
            print("Invalid Input")

if __name__ == "__main__":
    main()