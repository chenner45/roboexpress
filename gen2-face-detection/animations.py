import numpy as np
import time
import os
from animation_utils import load_np, print_np, save_np, read_ascii_to_np, resize_ascii_art, center_eyes_vertically, process_image

np.set_printoptions(threshold=np.inf)

# displays frames in order corresponding to duration values in times
# assumes len(frames) == len(times)
def animate(frames, times):
    assert len(frames) == len(times), "Length of frames and times lists must be equal"

  # Clear the terminal

    def get_terminal_size():
        rows, columns = os.popen('stty size', 'r').read().split()
        return int(rows), int(columns)

    # Hide cursor
    print('\033[?25l', end='')

    # try:
    for frame, display_time in zip(frames, times):
        # Get terminal size and calculate center position
        term_rows, term_cols = get_terminal_size()
        frame_rows, frame_cols = frame.shape
        start_row = max((term_rows - frame_rows) // 2, 0)
        start_col = max((term_cols - frame_cols) // 2, 0)
        
        # Ensure the frame is within the terminal size limits
        frame_rows = min(frame_rows, term_rows)
        frame_cols = min(frame_cols, term_cols)
        
        # Position the cursor at the start of where the frame should be displayed
        print(f'\033[{start_row};{0}H', end='', flush=True)
        # breakpoint()

        # Print the numpy array
        for i in range(frame_rows):
            row_content = ''.join(frame[i][:frame_cols]) if i < len(frame) else ''
            # Print the row, centered, ensuring the entire line is overwritten
            print(' ' * start_col + "\u001b[33m" + row_content + '\u001b[0m' + ' ' * (term_cols - start_col - len(row_content)))
        
        # Wait for the specified time
        time.sleep(display_time)

    # finally:
    #     # Clear formatting and show cursor again
    #     print('\033[0m\033[?25h', end='')

BLINK_FILES = ["blink1", "blink2", "blink3", "blink4", "blink5", "blink6", "blink7", "blink8", "blink9", "blink10", "blink9", "blink8", "blink7", "blink6", "blink5", "blink4", "blink3", "blink2", "blink1"]
BLINK_TIMES = [0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.03, 0.03, 0.03, 0.03, 0.03, 0.5]
def blink():
    frames = load_np(BLINK_FILES)
    times = np.array(BLINK_TIMES) / 10
    animate(frames, times)

HEART_FILES = ["heart"]
HEART_TIMES = [0.1]
def heart_eyes():
    frames = load_np(HEART_FILES)
    times = np.array(HEART_TIMES)
    animate(frames, times)

STATIC_FILES = ["blinktest"]
STATIC_TIMES = [0.1]
def static():
    frames = load_np(STATIC_FILES)
    times = STATIC_TIMES
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

    # static()
    

if __name__ == "__main__":
    main()

