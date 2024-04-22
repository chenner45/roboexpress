import numpy as np
import time
import os
from animation_utils import load_np, print_np, save_np, read_ascii_to_np, resize_ascii_art, center_eyes_vertically, process_image

np.set_printoptions(threshold=np.inf)

# displays frames in order corresponding to duration values in times
# assumes len(frames) == len(times)
def animate(frames, times):
    assert len(frames) == len(times), "Length of frames and times lists must be equal"

    os.system('clear')  # Clear the terminal

    for frame, display_time in zip(frames, times):
        print(f'\033[{0};{0}H', end='', flush=True)
        
        # Print the numpy array
        for row in frame:
            print("\u001b[33m" + ''.join(row) + '\u001b[0m', flush=True)
        
        # Wait for the specified time
        time.sleep(display_time)

BLINK_FILES = ["blink1", "blink2", "blink3", "blink4", "blink5", "blink6", "blink7", "blink8", "blink9", "blink10", "blink9", "blink8", "blink7", "blink6", "blink5", "blink4", "blink3", "blink2", "blink1"]
BLINK_TIMES = [0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.03, 0.03, 0.03, 0.03, 0.03, 0.5]
def blink():
    frames = load_np(BLINK_FILES)
    times = BLINK_TIMES
    animate(frames, times)

HEART_FILES = ["blink1", "heart", "blink1"]
HEART_TIMES = [0.5, 0.5, 1]
def heart_eyes():
    frames = load_np(HEART_FILES)
    times = HEART_TIMES
    animate(frames, times)



# regular eye width (each): 18
# heart size width: 27
def main():
    # read_ascii_to_np("heart")
    blink()
    blink()
    blink()
    heart_eyes()
    blink()
    heart_eyes()
    blink()
    # process_image("heart", 8, 70, 70)
    

if __name__ == "__main__":
    main()

