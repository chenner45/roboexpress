import numpy as np
import pyperclip
from PIL import Image

def print_np(arr):
    for row in arr:
        print(''.join(row))

# assumes subfolder of 'eyes/'
def load_np(filenames):
    loaded_arrays = []
    for filename in filenames:
        loaded_arrays.append(np.load("eyes/" + filename + ".npy"))
    return loaded_arrays

def save_np(filename, arr):
    np.save("eyes/" + filename, arr)


def read_ascii_to_np(filename):
    # Get the clipboard content
    clipboard_content = pyperclip.paste()

    # Split the clipboard content into lines
    lines = clipboard_content.split('\n')

    # Determine the size of the array
    rows = len(lines)
    cols = max(len(line) for line in lines)

    # Create an empty numpy array
    arr = np.empty((rows, cols), dtype='str')

    # Populate the numpy array with the clipboard content
    for i, line in enumerate(lines):
        for j, char in enumerate(line):
            arr[i, j] = char

    print_np(arr)
    save_np(filename, arr)

def resize_ascii_art(arr, new_width = 70):
    # Define a function to convert ASCII characters to grayscale values
    def char_to_grayscale(char):
        if char == '█':  # Darkest
            return 0
        elif char == '▒':  # Medium gray
            return 127
        elif char == ' ':  # Lightest
            return 255
        return 255  # Default to white for any other characters

    # Assuming 'ascii_art' is an array where each element is one of the above characters
    # For example:
    # ascii_art = np.array([
    #     ['█', '▒', ' ', '█', ' '],
    #     [' ', '▒', '█', ' ', '▒']
    # ])

    # Convert ASCII art to a grayscale image array
    grayscale_array = np.vectorize(char_to_grayscale)(arr)

    # Convert to Pillow Image
    image = Image.fromarray(grayscale_array.astype(np.uint8))

    # Calculate new dimensions
    original_width = arr.shape[1]
    aspect_ratio = arr.shape[0] / original_width
    new_height = int(new_width * aspect_ratio)

    # Resize the image
    resized_image = image.resize((new_width, new_height), Image.NEAREST)

    # Convert back to ASCII art from the resized image
    resized_grayscale = np.array(resized_image)

    # Define a function to convert grayscale values back to ASCII characters
    def grayscale_to_char(value):
        if value < 85:  # Darker third
            return '█'
        elif value < 170:  # Middle third
            return '▒'
        else:  # Lighter third
            return ' '

    # Convert grayscale values back to ASCII characters
    resized_ascii_art_chars = np.vectorize(grayscale_to_char)(resized_grayscale)

    return resized_ascii_art_chars

def center_eyes_vertically(ascii_art, height, width):
    # Identify non-empty rows and columns
    non_empty_rows = np.array([i for i in range(len(ascii_art)) if ''.join(ascii_art[i]).strip()])
    non_empty_cols = np.array([i for i in range(len(ascii_art[0])) if ''.join(ascii_art[:,i]).strip()])
    content = ascii_art[non_empty_rows, :]
    
    # Find the eyes and measure the current distance
    left_eye_start = non_empty_cols[non_empty_cols < len(ascii_art[0]) // 2].min()
    left_eye_end = non_empty_cols[non_empty_cols < len(ascii_art[0]) // 2].max()
    right_eye_start = non_empty_cols[non_empty_cols >= len(ascii_art[0]) // 2].min()
    right_eye_end = non_empty_cols[non_empty_cols >= len(ascii_art[0]) // 2].max()
    eye_width = left_eye_end - left_eye_start
    
    # Determine the starting column for the left eye
    new_left_eye_start = (len(ascii_art[0]) - width) // 2 - eye_width

    # Create new empty array
    new_ascii_art = np.full((height, len(ascii_art[0])), ' ', dtype='<U1')

    # Calculate vertical centering
    top_padding = (height - len(content)) // 2

    # Insert the left eye content into the new array
    new_ascii_art[top_padding:top_padding + len(content), new_left_eye_start:new_left_eye_start + eye_width + 1] = content[:, left_eye_start:left_eye_start + eye_width + 1]

    # # Insert the right eye content into the new array
    new_right_eye_start = new_left_eye_start + eye_width + width

    new_ascii_art[top_padding:top_padding + len(content), new_right_eye_start:new_right_eye_start + eye_width + 1] = content[:, right_eye_start:right_eye_start + eye_width + 1]

    return new_ascii_art



def process_image(filename, width_between, canvas_height, canvas_width):
    arr = load_np([filename])[0]
    print(arr.shape)
    for line in arr:
        print(''.join(line))
    arr = resize_ascii_art(arr, canvas_width)
    print(arr.shape)

    arr = center_eyes_vertically(arr, canvas_height, width_between)
    print(arr.shape)
    for line in arr:
        print(''.join(line))
    save_np(filename, arr)