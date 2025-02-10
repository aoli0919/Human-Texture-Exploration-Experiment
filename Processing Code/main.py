import cv2
import numpy as np
import random
import os
import tkinter as tk
from tkinter import simpledialog, messagebox
import time
import forcesensor
import pandas as pd

def main():
    # Set up the camera
    cap = cv2.VideoCapture(0)  # Set the camera port
    width = int(cap.get(3))  # Frame width of the video stream
    height = int(cap.get(4))  # Frame height of the video stream
    size = (width, height)
    fps = 30  # Frame rate
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')  # Video encoding format for mp4 files

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Set up the force sensor
    ft_sensor = forcesensor.FTSensorSerial()
    ft_sensor.set_zero_ref()

    # Use tkinter to get user input
    root = tk.Tk()
    root.withdraw()

    messagebox.showinfo("Welcome", "Welcome to our experiment!", parent=root)
    folder_name = simpledialog.askstring("Input", "Please enter your name:", parent=root)

    if folder_name is None:
        root.destroy()
        return

    # Set the storage path
    filepath = f"/home/leo/{folder_name}"  # Replace with your actual file path
    os.makedirs(filepath, exist_ok=True)

    used_numbers = []  # List of used random numbers
    recording = False  # Flag indicating whether recording is ongoing
    start_time = None

    # Function to get a new unused random number
    def get_new_random():
        while True:
            new_random = random.randint(1, 10)
            if new_random not in used_numbers:
                used_numbers.append(new_random)
                return new_random

    random_number = get_new_random()

    print("Press Enter to display the next random number. Press Q to stop recording.")
    force_data = []

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame = cv2.flip(frame, 1)  # Flip the frame

            if recording:
                # Display recording duration
                elapsed_time = float(time.time() - start_time)
                cv2.putText(frame, f'Recorded: {elapsed_time} sec', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 0, 0),
                            2)
                out.write(frame)  # Save each frame to create a video

                # Read force sensor data
                force = ft_sensor.get_ft()
                force.append(time.time())  # Add the current system time to the force sensor data
                force_data.append(force)

                # Stop recording if it exceeds 10 seconds
                if elapsed_time >= 10:
                    recording = False
                    out.release()
                    # Write force sensor data to a CSV file
                    force_data = np.array(force_data)
                    df = pd.DataFrame(force_data, columns=["Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "Timestamp"])
                    csv_file = f"{filepath}/force_data_{random_number}.csv"
                    df.to_csv(csv_file, index=False)

                    force_data = []  # Clear the force data list for the next recording

                    if len(used_numbers) < 10:  # Get a new random number if not all numbers have been used
                        random_number = get_new_random()
                    else:
                        break  # End the loop if all numbers have been used
            cv2.imshow('frame', frame)  # Display the video frame

            # Create a blank image and add the random number to it
            random_image = np.zeros((500, 500, 3), dtype='uint8')
            cv2.putText(random_image, f'Texture {random_number}', (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 255, 255), 2)
            cv2.imshow('random number', random_image)

            key = cv2.waitKey(1)
            if key == 13 and not recording:  # Enter key to start recording
                recording = True
                start_time = time.time()  # Record the start time

                out = cv2.VideoWriter()
                out.open(f"{filepath}/video_{random_number}.mp4", fourcc, fps, size)
            elif key & 0xFF in [ord('Q'), ord('q')]:  # Press Q to exit
                if recording:
                    recording = False
                    out.release()
                    # Write force sensor data to a CSV file
                    force_data = np.array(force_data)
                    df = pd.DataFrame(force_data, columns=["Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "Timestamp"])
                    csv_file = f"{filepath}/force_data_{random_number}.csv"
                    df.to_csv(csv_file, index=False)

                    force_data = []  # Clear the force data list for the next recording

                    if len(used_numbers) < 10:  # Get a new random number if not all numbers have been used
                        random_number = get_new_random()
                    else:
                        break  # End the loop if all numbers have been used

        else:
            break

    # Release the camera and video writer after all operations
    cap.release()
    cv2.destroyAllWindows()

    print("Recording complete. Data has been saved at the following location:")
    print("Video files: " + filepath + "/video_*.mp4")
    print("Force sensor data: " + filepath + "/force_data_*.csv")

if __name__ == "__main__":
    main()
