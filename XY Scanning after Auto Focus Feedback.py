import RPi.GPIO as GPIO
import time
import sys
import termios
import tty
import select

# Define GPIO pins for both motors
MOTOR_1_PINS = [15, 17, 18, 27]
MOTOR_2_PINS = [22, 23, 24, 10]

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in MOTOR_1_PINS + MOTOR_2_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# Function to set motor steps
def set_step(motor_pins, step):
    for pin in range(4):
        GPIO.output(motor_pins[pin], step[pin])

# Define the step sequence for the motor
STEP_SEQUENCE = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
]

# Function to rotate the motor
def rotate_motor(motor_pins, steps, delay, direction):
    sequence = STEP_SEQUENCE if direction == 'clockwise' else STEP_SEQUENCE[::-1]
    for _ in range(steps):
        if kbhit() and getch() == '\x1b':  # Esc key
            print("Process stopped by user.")
            GPIO.cleanup()
            sys.exit()
        for step in sequence:
            set_step(motor_pins, step)
            time.sleep(delay)

# Non-blocking keyboard input functions
def kbhit():
    dr, dw, de = select.select([sys.stdin], [], [], 0)
    return dr != []

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# User inputs
num_cycles = int(input("Enter the number of cycles: "))
motor1_steps = int(input("Enter the number of steps for Motor 1: "))
motor2_steps = int(input("Enter the number of steps for Motor 2: "))
rpm = int(input("Enter the RPM for both motors: "))
motor1_direction = input("Enter the direction for Motor 1 (A for counterclockwise, B for clockwise): ").strip().upper()
motor2_initial_direction = input("Enter the initial direction for Motor 2 (A for counterclockwise, B for clockwise): ").strip().upper()

# Calculate delay based on RPM
def calculate_delay(rpm):
    steps_per_revolution = 4096  # 28BYJ-48 has 4096 steps per revolution
    return 60.0 / (rpm * steps_per_revolution)

delay = calculate_delay(rpm)
motor1_direction = 'counterclockwise' if motor1_direction == 'A' else 'clockwise'
motor2_direction = 'counterclockwise' if motor2_initial_direction == 'A' else 'clockwise'

# Main loop
try:
    for cycle in range(num_cycles):
        print(f"Cycle {cycle + 1}")
        # Motor 1
        rotate_motor(MOTOR_1_PINS, motor1_steps, delay, motor1_direction)
       
        # Motor 2
        rotate_motor(MOTOR_2_PINS, motor2_steps, delay, motor2_direction)
        motor2_direction = 'counterclockwise' if motor2_direction == 'clockwise' else 'clockwise'
except KeyboardInterrupt:
    print("Process interrupted by user.")
finally:
    GPIO.cleanup()