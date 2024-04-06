import cv2
import os

# File path to the input image; change to the correct directory path
FOLDER_PATH = "/home/danoon/shared/aruco-marker-detection/Media/images_oakd_camera"
FILE_NAME = "chessboard_oakd_1.jpg" # Change to the correct file name
FILE_PATH = os.path.join(FOLDER_PATH, FILE_NAME)

NEW_FOLDER_NAME = "drawn_corners" # The name of the folder where the output image will be saved

# Chessboard dimensions
NUMBER_OF_SQUARES_X = 10 # Nr of chessboard squares along the x-axis
NUMBER_OF_SQUARES_Y = 7  # Nr of chessboard squares along the y-axis
nX = NUMBER_OF_SQUARES_X - 1 # Nr of interior corners along x-axis
nY = NUMBER_OF_SQUARES_Y - 1 # Nr of interior corners along y-axis

def prepare_output_directory_and_filename(file_path, folder_name):
  """ 
  A function to prepare the output directory and filename for the new image
  
  Args:
    file_path: The path to the input image file
    folder_name: The name of the folder where the output image will be saved
    
  Returns:
    output_directory: The output directory
    file_name_without_extension: The file name without the extension
  """
  # Extract the directory path and file name
  directory, file_name = os.path.split(file_path)
  # Create the output file name by removing the file extension part
  file_name_without_extension = os.path.splitext(file_name)[0]

  # Create the output directory path & ensure the output directory exists, if not, create it
  output_directory = os.path.join(directory, folder_name)
  os.makedirs(output_directory, exist_ok=True)

  return output_directory, file_name_without_extension

def load_image(file_path):
  """ 
  A function to load an image from the file path
  If the image could not be loaded, None is returned

  Args:
    file_path: The path to the image file

  Returns:
  image: The loaded image 
  """
  try:
      # Load an image
      image = cv2.imread(file_path)
      if image is None:
          raise FileNotFoundError("Could not load image")
      return image
  except Exception as e:
      print(f"Error loading image: {e}")
      return None

def detect_corners_on_chessboard(image):
  """
  A function to detect the corners on the chessboard in the image

  Args:
    image: The input image

  Returns:
    corners_found: A boolean indicating if the corners were found
    corners: The detected corners
  """
  gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
 
  # Find the corners on the chessboard
  corners_found, corners = cv2.findChessboardCorners(gray_image, (nY, nX), None)

  return corners_found, corners

def draw_corners_on_chessboard_and_save_as_image(image, corners, output_directory, file_name_without_extension):
  """
  A function to draw the corners on the chessboard and save the image

  Args:
    image: The input image
    corners: The detected corners
    output_directory: The output directory
    file_name_without_extension: The file name without the extension
  """

  cv2.drawChessboardCorners(image, (nY, nX), corners, True)

  # Create the output file path
  new_filename = os.path.join(output_directory, file_name_without_extension + '_drawn_corners.jpg')
  
  # Save the new image
  cv2.imwrite(new_filename, image)

def display_image(image):
  """ A function to display the image until any key is pressed """
  cv2.imshow("Image", image) 
  cv2.waitKey(0) 
    
  # Close all windows
  cv2.destroyAllWindows() 

def process_image(file_path):
  """ 
  A function to process the image and detect the corners on the chessboard
  
  Args:
    file_path: The path to the image file
  """
  output_directory, file_name_without_extension = prepare_output_directory_and_filename(file_path, NEW_FOLDER_NAME)    
  image = load_image(file_path)

  if image is not None:
    corners_found, corners = detect_corners_on_chessboard(image)

    if corners_found:
      draw_corners_on_chessboard_and_save_as_image(image, corners, output_directory, file_name_without_extension)
      display_image(image)
    else:
      print("No corners found")

process_image(FILE_PATH)