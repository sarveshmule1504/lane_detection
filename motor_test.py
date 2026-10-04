from machine import Pin
import time

# Motor Driver Pins (assuming L298N or similar)
# Left Motors (Front & Back wired in parallel on driver side)
IN1 = Pin(6, Pin.OUT)
IN2 = Pin(7, Pin.OUT)

# Right Motors (Front & Back wired in parallel on driver side)
IN3 = Pin(8, Pin.OUT)
IN4 = Pin(9, Pin.OUT)

def set_motors(in1, in2, in3, in4):
    IN1.value(in1)
    IN2.value(in2)
    IN3.value(in3)
    IN4.value(in4)

def motor_test():
    print("Starting Motor Test...")
    
    print("Moving Forward for 2 seconds")
    set_motors(1, 0, 1, 0)
    time.sleep(2)
    
    print("Stopping for 1 second")
    set_motors(0, 0, 0, 0)
    time.sleep(1)
    
    print("Moving Backward for 2 seconds")
    set_motors(0, 1, 0, 1)
    time.sleep(2)
    
    print("Stopping for 1 second")
    set_motors(0, 0, 0, 0)
    time.sleep(1)

    print("Turning Left for 2 seconds")
    set_motors(0, 1, 1, 0)
    time.sleep(2)
    
    print("Stopping for 1 second")
    set_motors(0, 0, 0, 0)
    time.sleep(1)

    print("Turning Right for 2 seconds")
    set_motors(1, 0, 0, 1)
    time.sleep(2)
    
    print("Stopping motors. Test complete.")
    set_motors(0, 0, 0, 0)

if __name__ == '__main__':
    try:
        motor_test()
    except KeyboardInterrupt:
        print("\nTest interrupted, stopping motors.")
        set_motors(0, 0, 0, 0)
