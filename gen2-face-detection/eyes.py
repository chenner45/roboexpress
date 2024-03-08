import os
import time
import numpy as np

MAX_X = 89
MAX_Y = 44

class Eyes:
    def __init__(self):
        self.center_x = 0
        self.center_y = 0
        self.eye_char = "█"

        # define size of canvas
        self.width = MAX_X
        self.height = MAX_Y

    def move_cursor(self, row, col):
        print(f'\033[{row};{col}H', end='', flush=True)

    """
    x - Value from -1 to 1
    y - value from -1 to 1
    """
    def calc_eyes(self, x_scale, y_scale):
        oval_offset_x = int(x_scale * MAX_X / 4)
        oval_offset_y = int(y_scale * MAX_Y / 2)
        
        # Define the parameters for the ovals
        oval1_center_x = MAX_X // 4
        oval1_center_y = MAX_Y // 2
        
        # oval1_center = (6, 9)
        oval1_center = (oval1_center_x + oval_offset_x, oval1_center_y + oval_offset_y)
        oval1_radius_x = 15
        oval1_radius_y = 16

        oval2_center_x = int(3 * MAX_X / 4)
        oval2_center_y = MAX_Y // 2
        # oval2_center = (25, 9)
        oval2_center = (oval2_center_x + oval_offset_x, oval2_center_y + oval_offset_y)
        oval2_radius_x = 15
        oval2_radius_y = 16

        # shift right
        if x_scale > 0:
            # x_scale = 0 --> 1, x_scale = 1 --> 0.5
            oval2_radius_x *= -0.5*x_scale + 1
            oval2_radius_y *= -0.5*x_scale + 1
            oval1_radius_x *= -0.2*x_scale + 1
            oval1_radius_y *= -0.2*x_scale + 1

        # shift left
        if x_scale < 0:
            oval2_radius_x *= 0.2*x_scale + 1
            oval2_radius_y *= 0.2*x_scale + 1
            oval1_radius_x *= 0.5*x_scale + 1
            oval1_radius_y *= 0.5*x_scale + 1

        return oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, 0, 0
        
    def draw_eyes(self, oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, x_restrict, y_restrict):
        # Draw two ovals as a 2d array that gets printed out as text that are filled in and shift them depending on center_x/center_y
        # Cut off any parts of the ovals that are outside of the x,y min/max bounds
        self.move_cursor(0, 0)
        
        # Create a 2D array filled with zeros
        canvas = np.zeros((self.height, self.width), dtype=int)

        # Generate the first oval
        for y in range(self.height):
            if y < y_restrict:
                continue
            for x in range(self.width):
                if x < x_restrict:
                    continue
                if ((x - oval1_center[0]) / oval1_radius_x) ** 2 + ((y - oval1_center[1]) / oval1_radius_y) ** 2 <= 1:
                    canvas[y, x] = 1

        # Generate the second oval
        for y in range(1, self.height):
            if y < y_restrict:
                continue
            for x in range(self.width):
                if x < x_restrict:
                    continue
                if ((x - oval2_center[0]) / oval2_radius_x) ** 2 + ((y - oval2_center[1]) / oval2_radius_y) ** 2 <= 1:
                    canvas[y, x] = 1

        # Display the canvas
        for i, row in enumerate(canvas):
            # if i == 1 or i >= len(canvas) - 3:
            #     continue
            print("\u001b[33m" + ''.join([self.eye_char if val == 1 else ' ' for val in row]) + '\u001b[0m', flush=True)


    def blink(self, x_scale, y_scale):
        oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, _, _ = self.calc_eyes(x_scale, y_scale)
        for i in range(self.height):
            self.draw_eyes(oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, 0, i)

        for i in reversed(range(self.height)):
            self.draw_eyes(oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, 0, i)


def move_eyes(dx, dy):
    pass

def main():
    # Example usage
    eyes = Eyes()
    eyes.center_x = 2
    eyes.center_y = 2
    os.system('cls' if os.name == 'nt' else 'clear')  # Clear the terminal
    # while True:
    #     # Read the (x, y) coordinate from detections
    #     # os.system('cls' if os.name == 'nt' else 'clear')  # Clear the terminal

    #     # Print eyes
    #     print('    ', end='', flush=True)
    #     eyes.draw_eyes(-0.5, 0)
    #     time.sleep(1)

    while True:
        for x in range(-10, 10, 1):
            print('    ', end='', flush=True)
            eyes.draw_eyes(x / 10, x / 10)
            time.sleep(0.02)
        for x in reversed(range(-10, 10, 1)):
            print('    ', end='', flush=True)
            eyes.draw_eyes(x / 10, x / 10)
            time.sleep(0.02)
        






if __name__ == "__main__":
    main()