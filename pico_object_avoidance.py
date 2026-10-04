from machine import Pin, time_pulse_us
import time

# Ultrasonic Sensor 1 (Left)
TRIG_LEFT_PIN = 2
ECHO_LEFT_PIN = 3

# Ultrasonic Sensor 2 (Right)
TRIG_RIGHT_PIN = 4
ECHO_RIGHT_PIN = 5

# Motor Driver Pins (using simple OUT instead of PWM for reliability)
IN1 = Pin(6, Pin.OUT)
IN2 = Pin(7, Pin.OUT)
IN3 = Pin(8, Pin.OUT)
IN4 = Pin(9, Pin.OUT)

# Initialize Ultrasonic Pins
trig_left = Pin(TRIG_LEFT_PIN, Pin.OUT)
echo_left = Pin(ECHO_LEFT_PIN, Pin.IN)
trig_right = Pin(TRIG_RIGHT_PIN, Pin.OUT)
echo_right = Pin(ECHO_RIGHT_PIN, Pin.IN)

# Threshold distances (in cm)
OBSTACLE_THRESHOLD = 40.0

# Current State
current_action = "STOP"

def get_distance_raw(trig, echo):
    trig.value(0)
    time.sleep_us(5)
    trig.value(1)
    time.sleep_us(15) # Increased pulse width for a stronger trigger ping
    trig.value(0)
    
    try:
        pulse_time = time_pulse_us(echo, 1, 30000) # 30ms timeout (~5m)
        if pulse_time < 0:
            return 999.0 # Timeout/Out of range
        distance = (pulse_time * 0.0343) / 2
        return distance
    except Exception:
        return 999.0

def get_distance(trig, echo):
    """Measures distance using median of 3 samples for much higher accuracy"""
    d1 = get_distance_raw(trig, echo)
    time.sleep(0.01) # 10ms delay between pings to prevent echo overlap
    d2 = get_distance_raw(trig, echo)
    time.sleep(0.01)
    d3 = get_distance_raw(trig, echo)
    
    # Return the median (middle) value to filter out random noise spikes
    readings = sorted([d1, d2, d3])
    return readings[1]

def check_and_print_sensors():
    """Reads both sensors and prints their status continuously to the console"""
    dist_left = get_distance(trig_left, echo_left)
    dist_right = get_distance(trig_right, echo_right)
    print("Status -> Left: {:.1f} cm | Right: {:.1f} cm".format(dist_left, dist_right))
    return dist_left, dist_right

def wait_while_checking(seconds):
    """Pauses for a set time, but continuously checks and prints sensor data"""
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < (seconds * 1000):
        check_and_print_sensors()
        time.sleep(0.1)

# --- Motor Control Functions ---
def set_motors(action, in1, in2, in3, in4):
    global current_action
    current_action = action
    IN1.value(in1)
    IN2.value(in2)
    IN3.value(in3)
    IN4.value(in4)

def move_forward():
    set_motors("FORWARD", 1, 0, 1, 0)

def move_backward():
    set_motors("BACKWARD", 0, 1, 0, 1)

def turn_left():
    set_motors("TURN_LEFT", 0, 1, 1, 0)

def turn_right():
    set_motors("TURN_RIGHT", 1, 0, 0, 1)

def stop():
    set_motors("STOP", 0, 0, 0, 0)

# --- Main Loop ---
def main():
    stop()
    print("Starting Pico Object Avoidance System...")
    time.sleep(2)
    
    while True:
        # 1. Read Sensors continuously and print to terminal
        dist_left, dist_right = check_and_print_sensors()
        
        # 2. Logic for avoidance
        if dist_left < OBSTACLE_THRESHOLD or dist_right < OBSTACLE_THRESHOLD:
            # Stop immediately when an object is detected
            print("OBSTACLE DETECTED! Stopping.")
            stop()
            wait_while_checking(2.5) # Wait 2.5s but keep checking sensors
            
            # Make a decision on which way to turn
            if dist_left < OBSTACLE_THRESHOLD and dist_right < OBSTACLE_THRESHOLD:
                print("Both blocked -> Reversing")
                move_backward()
                wait_while_checking(2.0)
            elif dist_left < OBSTACLE_THRESHOLD:
                print("Left blocked -> Turning left")
                turn_left()
                wait_while_checking(2.0)
            elif dist_right < OBSTACLE_THRESHOLD:
                print("Right blocked -> Turning right")
                turn_right()
                wait_while_checking(2.0)
                
            # Stop the motors after the turn before continuing the loop
            stop()
            time.sleep(0.1)
        else:
            # Safe zone (>20cm): Move forward
            move_forward()
            
        time.sleep(0.05)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        stop()
