import serial
import time

def main():
    ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)

    try:
        while True:
        
            print("Testing LEFT motor")
            checksum = (0xFF - (0x00 + 0x07 + 0x01 + 0x00 + 0x46 + 0x50 + 0x00 + 0x32)) & 0xFF
            control_L = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x00, 0x46, 0x50, 0x00, 0x32]
            ser.write(bytes(control_L))
            time.sleep(1)

            print("Testing RIGHT motor")
            checksum = (0xFF - (0x00 + 0x07 + 0x00 + 0x00 + 0x46 + 0x50 + 0x00 + 0x32)) & 0xFF
            control_R = [0xFF, 0xFE, 0x00, 0x07, checksum, 0x01, 0x00, 0x46, 0x50, 0x00, 0x32]
            ser.write(bytes(control_R))
            time.sleep(1)


    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')

    finally:
        ser.close()

if __name__ == '__main__':
    main()
