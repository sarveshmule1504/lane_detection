import serial
import json
import time

# --- Configuration ---
# Serial port for Raspberry Pi to communicate with Pico
# /dev/serial0 is usually the default GPIO serial on Pi
# Ensure UART is enabled in raspi-config
SERIAL_PORT = '/dev/serial0' 
BAUD_RATE = 115200

def main():
    print(f"Connecting to Pico on {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Connected successfully. Waiting for telemetry data...")
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")
        print("Please check if the serial port is correct and enabled.")
        return

    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        data = json.loads(line)
                        # --- Data Analysis ---
                        dist_l = data.get('dist_left', 0)
                        dist_r = data.get('dist_right', 0)
                        action = data.get('action', 'UNKNOWN')
                        
                        print(f"[Telemetry] Action: {action:<10} | L-Dist: {dist_l:>6.1f} cm | R-Dist: {dist_r:>6.1f} cm")
                        
                        # Add your further data analysis or logging logic here
                        
                    except json.JSONDecodeError:
                        print(f"Received malformed data: {line}")
            time.sleep(0.01)
            
        except KeyboardInterrupt:
            print("\nStopping data analysis...")
            ser.close()
            break
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(1)

if __name__ == '__main__':
    main()
