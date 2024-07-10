import RPi.GPIO as GPIO
import time

# Define GPIO pins for switches
CW_M1_PIN = 6   # GPIO 20 for Motor 1 clockwise
CCW_M1_PIN = 12  # GPIO 21 for Motor 1 counterclockwise
CW_M2_PIN = 19  # GPIO 22 for Motor 2 clockwise
CCW_M2_PIN = 16  # GPIO 23 for Motor 2 counterclockwise
CW_M3_PIN = 20  # GPIO 26 for Motor 3 clockwise
CCW_M3_PIN = 21  # GPIO 13 for Motor 3 counterclockwise

# Define GPIO pins for motor 1
IN1_M1 = 15  # GPIO 15
IN2_M1 = 17  # GPIO 17
IN3_M1 = 18  # GPIO 18
IN4_M1 = 27  # GPIO 27

# Define GPIO pins for motor 2
IN1_M2 = 22  # GPIO 10
IN2_M2 = 23  # GPIO 9
IN3_M2 = 24  # GPIO 11
IN4_M2 = 10  # GPIO 5

# Define GPIO pins for motor 3
IN1_M3 = 2   # Example GPIO pin for Motor 3
IN2_M3 = 3  # Example GPIO pin for Motor 3
IN3_M3 = 4  # Example GPIO pin for Motor 3
IN4_M3 = 14   # Example GPIO pin for Motor 3

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Set up GPIO pins for switches - Motor 1
GPIO.setup(CW_M1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # pull-up resistor
GPIO.setup(CCW_M1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # pull-up resistor

# Set up GPIO pins for switches - Motor 2
GPIO.setup(CW_M2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # pull-up resistor
GPIO.setup(CCW_M2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # pull-up resistor

# Set up GPIO pins for switches - Motor 3
GPIO.setup(CW_M3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # pull-up resistor
GPIO.setup(CCW_M3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # pull-up resistor

# Set up GPIO pins for motor 1
GPIO.setup(IN1_M1, GPIO.OUT)
GPIO.setup(IN2_M1, GPIO.OUT)
GPIO.setup(IN3_M1, GPIO.OUT)
GPIO.setup(IN4_M1, GPIO.OUT)

# Set up GPIO pins for motor 2
GPIO.setup(IN1_M2, GPIO.OUT)
GPIO.setup(IN2_M2, GPIO.OUT)
GPIO.setup(IN3_M2, GPIO.OUT)
GPIO.setup(IN4_M2, GPIO.OUT)

# Set up GPIO pins for motor 3
GPIO.setup(IN1_M3, GPIO.OUT)
GPIO.setup(IN2_M3, GPIO.OUT)
GPIO.setup(IN3_M3, GPIO.OUT)
GPIO.setup(IN4_M3, GPIO.OUT)

# Define step sequence
sequence = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

# Function to set GPIO pins for motor 1
def set_pins_m1(pins):
    GPIO.output(IN1_M1, pins[0])
    GPIO.output(IN2_M1, pins[1])
    GPIO.output(IN3_M1, pins[2])
    GPIO.output(IN4_M1, pins[3])

# Function to set GPIO pins for motor 2
def set_pins_m2(pins):
    GPIO.output(IN1_M2, pins[0])
    GPIO.output(IN2_M2, pins[1])
    GPIO.output(IN3_M2, pins[2])
    GPIO.output(IN4_M2, pins[3])

# Function to set GPIO pins for motor 3
def set_pins_m3(pins):
    GPIO.output(IN1_M3, pins[0])
    GPIO.output(IN2_M3, pins[1])
    GPIO.output(IN3_M3, pins[2])
    GPIO.output(IN4_M3, pins[3])

# Function to rotate motor 1 clockwise
def rotate_clockwise_m1():
    while GPIO.input(CW_M1_PIN) == GPIO.LOW:
        for seq in sequence:
            set_pins_m1(seq)
            time.sleep(0.001)  # Adjust delay as needed

# Function to rotate motor 1 counterclockwise
def rotate_counterclockwise_m1():
    while GPIO.input(CCW_M1_PIN) == GPIO.LOW:
        for seq in reversed(sequence):
            set_pins_m1(seq)
            time.sleep(0.001)  # Adjust delay as needed

# Function to rotate motor 2 clockwise
def rotate_clockwise_m2():
    while GPIO.input(CW_M2_PIN) == GPIO.LOW:
        for seq in sequence:
            set_pins_m2(seq)
            time.sleep(0.001)  # Adjust delay as needed

# Function to rotate motor 2 counterclockwise
def rotate_counterclockwise_m2():
    while GPIO.input(CCW_M2_PIN) == GPIO.LOW:
        for seq in reversed(sequence):
            set_pins_m2(seq)
            time.sleep(0.001)  # Adjust delay as needed

# Function to rotate motor 3 clockwise
def rotate_clockwise_m3():
    while GPIO.input(CW_M3_PIN) == GPIO.LOW:
        for seq in sequence:
            set_pins_m3(seq)
            time.sleep(0.001)  # Adjust delay as needed

# Function to rotate motor 3 counterclockwise
def rotate_counterclockwise_m3():
    while GPIO.input(CCW_M3_PIN) == GPIO.LOW:
        for seq in reversed(sequence):
            set_pins_m3(seq)
            time.sleep(0.001)  # Adjust delay as needed

# Main function
def main():
    try:
        while True:
            # Motor 1 control
            if GPIO.input(CW_M1_PIN) == GPIO.LOW:
                print("Motor 1 Clockwise button pressed")
                rotate_clockwise_m1()
            elif GPIO.input(CCW_M1_PIN) == GPIO.LOW:
                print("Motor 1 Counterclockwise button pressed")
                rotate_counterclockwise_m1()
            else:
                set_pins_m1([0, 0, 0, 0])  # Stop motor 1

            # Motor 2 control
            if GPIO.input(CW_M2_PIN) == GPIO.LOW:
                print("Motor 2 Clockwise button pressed")
                rotate_clockwise_m2()
            elif GPIO.input(CCW_M2_PIN) == GPIO.LOW:
                print("Motor 2 Counterclockwise button pressed")
                rotate_counterclockwise_m2()
            else:
                set_pins_m2([0, 0, 0, 0])  # Stop motor 2

            # Motor 3 control
            if GPIO.input(CW_M3_PIN) == GPIO.LOW:
                print("Motor 3 Clockwise button pressed")
                rotate_clockwise_m3()
            elif GPIO.input(CCW_M3_PIN) == GPIO.LOW:
                print("Motor 3 Counterclockwise button pressed")
                rotate_counterclockwise_m3()
            else:
                set_pins_m3([0, 0, 0, 0])  # Stop motor 3

            time.sleep(0.01)  # Small delay to reduce CPU usage

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

# Entry point of the script
if __name__ == "__main__":
    main()