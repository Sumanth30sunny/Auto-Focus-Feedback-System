import cv2
import numpy as np
from picamera2 import Picamera2
import threading
import RPi.GPIO as GPIO
import time
import queue
from datetime import datetime
import os

# Defining the output pins for stepper motor
IN1 = 2
IN2 = 3
IN3 = 4
IN4 = 14

# Sequence of execution for Half step rotation
seq = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
]


# Function to calculate Laplacian variance for image sharpness
def calculate_laplacian_variance(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()


# Function to setup GPIO pins for stepper motor
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)


# Cleanup after completion of the process
def cleanup_gpio():
    GPIO.cleanup()


# Full Forward movement method
def move_full_front(delay):
    steps_to_move = 2700  # 2700 steps for full movement
    for _ in range(steps_to_move):
        for step in seq:
            set_step(step[0], step[1], step[2], step[3])
            time.sleep(delay)


# Move forward by given steps
def move_forward(delay, steps):
    for _ in range(steps):
        for step in seq:
            set_step(step[0], step[1], step[2], step[3])
            time.sleep(delay)


# Move backward by given steps
def move_backward(delay, steps):
    for _ in range(steps):
        for step in reversed(seq):
            set_step(step[0], step[1], step[2], step[3])
            time.sleep(delay)


# Energizing the coils for Half step rotation
def set_step(w1, w2, w3, w4):
    GPIO.output(IN1, w1)
    GPIO.output(IN2, w2)
    GPIO.output(IN3, w3)
    GPIO.output(IN4, w4)


# Function to control motor movement and record Laplacian variance
def motor_control_with_laplacian(picam2, variance_queue, stop_event):
    try:
        setup_gpio()
        rpm = 15  # rpm should be 10 but maximum is 15
        delay = 60.0 / (rpm * 4096)  # delay time would approx. be 1ms

        # Move 2700 steps forward
        move_full_front(delay)

        best_position = 0
        best_variance = 0.0
        previous_variance = 0.0

        # Move 10 steps back and record Laplacian variance at each step
        for i in range(50):
            # Capture and calculate Laplacian variance
            frame = picam2.capture_array()
            variance = calculate_laplacian_variance(frame)

            # Add variance to the queue
            variance_queue.put((i, variance))

            # Check if this position has the highest variance
            if variance > best_variance:
                best_variance = variance
                best_position = i

            # Detect a significant increase in variance
            if i > 0 and (variance - previous_variance) > 200:  # 200 as significant increase
                print(f"Significant increase detected at position {i + 1}, stopping motor.")
                break

            # Move motor 10 steps back
            for _ in range(10):
                for step in reversed(seq):
                    set_step(step[0], step[1], step[2], step[3])
                    time.sleep(delay)

            # Update previous variance
            previous_variance = variance

            # Add a 0.5-second delay
            time.sleep(0.5)

        print(f"Best position: {best_position + 1}, Best variance: {best_variance}")
        # Get variance at the best position
        frame = picam2.capture_array()
        best_variance = calculate_laplacian_variance(frame)
        print(f"Best Position: {best_position}, Variance: {best_variance:.2f}")
        # Move one step forward and get variance
        move_forward(delay, 1)
        frame = picam2.capture_array()
        forward_variance = calculate_laplacian_variance(frame)
        print(f"Forward Position: {best_position + 0.1}, Variance: {forward_variance:.2f}")
        # Move back to best position
        move_backward(delay, 1)

        # Move one step backward and get variance
        move_backward(delay, 1)
        frame = picam2.capture_array()
        backward_variance = calculate_laplacian_variance(frame)
        print(f"Backward Position: {best_position - 0.1}, Variance: {backward_variance:.2f}")

        # Move back to best position
        move_forward(delay, 1)

        # Determine if adjustment is needed
        if forward_variance > best_variance:
            print("Moving to Forward Position for best focus.")
            move_forward(delay, 1)
            front_feedback(picam2, delay)
        elif backward_variance > best_variance:
            print("Moving to Backward Position for best focus.")
            move_backward(delay, 1)
            back_feedback(picam2, delay)
        else:
            print("Staying at Best Position for best focus.")

        # Signal that the motor control has completed
        stop_event.set()

    except KeyboardInterrupt:
        print("\nStopping motor control...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cleanup_gpio()

        # Create the output directory if it does not exist
        output_directory = "/home/pi/Desktop/Auto Focus Feedback Result"
        if not os.path.exists(output_directory):
            os.makedirs(output_directory)

        # Capture the final focused image
        frame = picam2.capture_array()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_name = f"{output_directory}/Focused_Result_{timestamp}.jpg"
        cv2.imwrite(file_name, frame)
        print(f"Focused image saved as {file_name}")


# Front feedback method
def front_feedback(picam2, delay):
    try:
        setup_gpio()

        # Get the variance at the current position
        frame = picam2.capture_array()
        best_variance = calculate_laplacian_variance(frame)

        while True:
            # Move one step forward and get variance
            move_forward(delay, 1)
            frame = picam2.capture_array()
            new_variance = calculate_laplacian_variance(frame)
            print(f"New Forward Position Variance: {new_variance:.2f}")

            if new_variance > best_variance:
                best_variance = new_variance
            else:
                # Move back to the best position
                move_backward(delay, 1)
                break

    except KeyboardInterrupt:
        print("\nStopping motor control...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cleanup_gpio()


# Back feedback method
def back_feedback(picam2, delay):
    try:
        setup_gpio()

        # Get the variance at the current position
        frame = picam2.capture_array()
        best_variance = calculate_laplacian_variance(frame)

        while True:
            # Move one step backward and get variance
            move_backward(delay, 1)
            frame = picam2.capture_array()
            new_variance = calculate_laplacian_variance(frame)
            print(f"New Backward Position Variance: {new_variance:.2f}")

            if new_variance > best_variance:
                best_variance = new_variance
            else:
                # Move forward to the best position
                move_forward(delay, 1)
                break

    except KeyboardInterrupt:
        print("\nStopping motor control...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cleanup_gpio()


def main():
    # Initialize Picamera2 instance
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888"}))
    picam2.start()

    variance_queue = queue.Queue()
    stop_event = threading.Event()

    # Create motor control thread
    motor_thread = threading.Thread(target=motor_control_with_laplacian, args=(picam2, variance_queue, stop_event))

    # Start the motor control thread
    motor_thread.start()

    try:
        while True:
            frame = picam2.capture_array()
            variance = calculate_laplacian_variance(frame)

            # Create a blank space below the frame for text
            text_space = np.zeros((50, frame.shape[1], 3), dtype=np.uint8)

            # Display the Laplacian variance on the text space
            cv2.putText(text_space, f"Laplacian Variance: {variance:.2f}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 255, 255), 2)

            # Combine the frame and text space
            combined_frame = np.vstack((frame, text_space))

            # Show the combined frame
            cv2.imshow("Camera Feed", combined_frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

            # Print variances from the queue
            while not variance_queue.empty():
                position, variance = variance_queue.get()
                print(f"Position {position + 1}: {variance:.2f}")

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        stop_event.set()
        motor_thread.join()

        # Clean up GPIO after everything is done
        cleanup_gpio()


if __name__ == "__main__":
    main()
