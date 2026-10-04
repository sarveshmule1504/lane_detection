from machine import Pin
import time

# On standard Raspberry Pi Pico, the onboard LED is Pin 25
# On Pico W, you use "LED" instead. We'll use 25 for the standard Pico.
led = Pin(25, Pin.OUT)

def main():
    print("Starting LED Blink...")
    while True:
        led.value(1)  # Turn LED on
        time.sleep(0.5) # Wait half a second
        led.value(0)  # Turn LED off
        time.sleep(0.5) # Wait half a second

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # Ensure LED is turned off when stopping the script
        led.value(0)
        print("Blinking stopped.")
