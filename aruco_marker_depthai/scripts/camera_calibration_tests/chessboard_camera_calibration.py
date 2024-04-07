import cv2
import numpy as np
import glob
import os
import pickle

# For testing purposes:
DEBUG_IMAGES = False # Set to True to display the images with the detected corners
DEBUG_PARAMETERS = False # Set to True to display the key parameters in console

# Path to the folder where the pickle file will be saved; change to the correct directory path
PICKLE_PATH = '/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration_tests/calibration_values'
PICKLE_FILE_NAME = 'camera_calibration_pickle_file' # The name of the pickle file, change if needed
PICKLE_FILE_PATH = os.path.join(PICKLE_PATH, PICKLE_FILE_NAME)

# File path to the input image; change to the correct directory path
DISTORTED_IMAGE_FOLDER = '/home/danoon/shared/aruco-marker-detection/Media/images_oakd_camera'
IMAGE_EXTENSION = '.jpg' # Change to the correct file extension

# File name of the example image; this will be undistorted and saved in the output folder
DISTORTED_EXAMPLE_IMAGE = 'chessboard_oakd_1.jpg' # Change to the correct file name
DISTORTED_IMAGE_PATH = os.path.join(DISTORTED_IMAGE_FOLDER, DISTORTED_EXAMPLE_IMAGE)
NEW_FOLDER_NAME = "undistorted" # The name of the folder where the output image will be saved

# Chessboard dimensions
NUMBER_OF_SQUARES_X = 10 # Nr of chessboard squares along the x-axis
NUMBER_OF_SQUARES_Y = 7  # Nr of chessboard squares along the y-axis
nX = NUMBER_OF_SQUARES_X - 1 # Nr of interior corners along x-axis
nY = NUMBER_OF_SQUARES_Y - 1 # Nr of interior corners along y-axis
SQUARE_SIZE = 0.025 # Length of the side of a square in meters

# Store vectors of 3D points for all chessboard images (world coordinate frame)
object_points = []
 
# Store vectors of 2D points for all chessboard images (camera coordinate frame)
image_points = []

def set_termination_criteria():
  """
  A function to set the termination criteria for the corner detection algorithm
  It stops either when an accuracy is reached or when the maximum number of iterations is reached.
  
  Returns:
    termination_criteria: The termination criteria
  """
  return (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
def prepare_3D_object_points(square_size):
  """
  A function to prepare the 3D object points for the chessboard corners
  The object points are the same for all images
  Object points are (0,0,0), (1,0,0), (2,0,0) ...., (5,8,0)
  
  Args:
    square_size: The size of the square in meters
    
  Returns:
    object_points_3D: The 3D object points
  """
  object_points_3D = np.zeros((nX * nY, 3), np.float32)       
  
  # These are the x and y coordinates                                              
  object_points_3D[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2) 
  object_points_3D = object_points_3D * square_size # Multiply by the square size to get the correct size

  return object_points_3D

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
  directory, file_name = os.path.split(file_path)
  # Create the output file name by removing the file extension part
  file_name_without_extension = os.path.splitext(file_name)[0]

  # Create the output directory path & ensure the output directory exists, if not, create it
  output_directory = os.path.join(directory, folder_name)
  os.makedirs(output_directory, exist_ok=True)

  return output_directory, file_name_without_extension

def get_images_and_draw_corners_on_them(directory_path):
  """
  A function to get all images in the directory and draw the corners on them
  
  Args:
    directory_path: The path to the directory containing the images
  """
  pattern = os.path.join(directory_path, f'*{IMAGE_EXTENSION}')
  # Get a list of all files in the directory (not including subdirectories)
  images = glob.glob(pattern)
 
  # Go through each chessboard image
  for image_file in images:
    image = cv2.imread(image_file)
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
 
    corners_found, corners = cv2.findChessboardCorners(gray_image, (nY, nX), None)

    if corners_found:
      draw_corners(image, gray_image, corners)

def draw_corners(image, gray_image, corners):
  """
  A function to draw the corners on the image
  It appends the object points and image points to the respective lists
  
  Args:
    image: The input image
    gray_image: The grayscale image
    corners: The detected corners
  """
  object_points.append(object_points_3D)

  # Find more exact corner pixels       
  corners_2 = cv2.cornerSubPix(gray_image, corners, (11,11), (-1,-1), criteria)       
    
  image_points.append(corners_2)

  # Draw the corners
  cv2.drawChessboardCorners(image, (nY, nX), corners_2, True)

  # Display the image if debugging is enabled
  if DEBUG_IMAGES: 
    display_image(image)

def display_image(image):
  """ A function to display the image until any key is pressed """
  cv2.imshow("Image", image) 
  cv2.waitKey(200)

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
      image = cv2.imread(file_path)
      if image is None:
          raise FileNotFoundError("Could not load image")
      return image
  except Exception as e:
      print(f"Error loading image: {e}")
      return None
  
def load_example_distorted_image(file_path): 
  """
  A function to load an example distorted image and convert it to grayscale
  If the image could not be loaded, None is returned

  Args:
    file_path: The path to the image file
    
  Returns:
    distorted_image: The loaded distorted image
    gray_image: The grayscale image
  """
  distorted_image = load_image(file_path)

  if distorted_image is not None:
    gray_image = cv2.cvtColor(distorted_image, cv2.COLOR_BGR2GRAY)  
    return distorted_image, gray_image
  else:
    return None, None

def calibrate_camera(gray_image):
  """ 
  A function to calibrate the camera using the detected corners on the chessboard images

  Args:
    gray_image: The grayscale image

  Returns:
    mtx: The camera matrix
    dist: The distortion coefficients
    rvecs: The rotation vectors
    tvecs: The translation vectors
  """
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray_image.shape[::-1], None, None)
  return mtx, dist, rvecs, tvecs

def get_image_dimensions(distorted_image):
  """ Returns the height and width of the image """
  height, width = distorted_image.shape[:2]
  return height, width

def refine_camera_matrix(mtx, dist, height, width):
  """
  A function to refine the camera matrix and get the optimal camera matrix
  
  Args:
    mtx: The camera matrix
    dist: The distortion coefficients
    height: The height of the image
    width: The width of the image
    
  Returns:
    optimal_camera_matrix: The optimal camera matrix
  """
  optimal_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))
  return optimal_camera_matrix

def undistort_example_image(distorted_image, mtx, dist, optimal_camera_matrix):
  """
  A function to undistort the example image
  
  Args:
    distorted_image: The distorted image
    mtx: The camera matrix
    dist: The distortion coefficients
    optimal_camera_matrix: The optimal camera matrix
  
  Returns:
    undistorted_image: The undistorted image
  """
  undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, optimal_camera_matrix)

  return undistorted_image
  
def save_undistorted_example_image(output_directory, file_name_without_extension, undistorted_image):
  """
  A function to save the undistorted example image
  
  Args:
    output_directory: The output directory
    file_name_without_extension: The file name without the extension
    undistorted_image: The undistorted image  
  """
  # Create the output file path
  new_filename = os.path.join(output_directory, file_name_without_extension + '_undistorted.jpg')

  # Save the undistorted image
  cv2.imwrite(new_filename, undistorted_image)
 
def close_all_cv_windows():
  """ Close all OpenCV windows """
  cv2.destroyAllWindows()

def display_key_parameter_outputs(optimal_camera_matrix, dist, rvecs, tvecs):
  """ Display the key parameters """
  print("--- Camera Calibration Results ---")
  
  print("\nOptimal Camera matrix:")
  print(optimal_camera_matrix) 

  print("\nDistortion coefficient:") 
  print(dist) 

  print("\nRotation Vectors:") 
  print(rvecs) 

  print("\nTranslation Vectors:") 
  print(tvecs) 
 
def save_key_parameters_to_pickle_file(optimal_camera_matrix, mtx, dist, rvecs, tvecs):
  """
  A function to save the key parameters to a pickle file
  
  Args:
    optimal_camera_matrix: The optimal camera matrix
    mtx: The camera matrix
    dist: The distortion coefficients
    rvecs: The rotation vectors
    tvecs: The translation vectors
  """
  calibration_result_list = {
    "mtx": mtx,
    "optimal_camera_matrix": optimal_camera_matrix,
    "dist": dist,
    "rvecs": rvecs,
    "tvecs": tvecs
  }

  with open(f"{PICKLE_FILE_PATH}.pkl", "wb" ) as new_pickle_file:
    pickle.dump(calibration_result_list, new_pickle_file)
    print('\nFile saved to', f"{PICKLE_FILE_PATH}.pkl")



def process_images(file_path):
  """
  A function to process the image and undistort it
  
  Args:
    file_path: The path to the image file
  """
  output_directory, file_name_without_extension = prepare_output_directory_and_filename(file_path, NEW_FOLDER_NAME)
  
  distorted_image, gray_image = load_example_distorted_image(file_path)
  height, width = get_image_dimensions(distorted_image)
  
  mtx, dist, rvecs, tvecs = calibrate_camera(gray_image)
  optimal_camera_matrix = refine_camera_matrix(mtx, dist, height, width)

  undistorted_image = undistort_example_image(distorted_image, mtx, dist, optimal_camera_matrix)
  save_undistorted_example_image(output_directory, file_name_without_extension, undistorted_image)

  save_key_parameters_to_pickle_file(optimal_camera_matrix, mtx, dist, rvecs, tvecs)
  
  if DEBUG_PARAMETERS:
    display_key_parameter_outputs(optimal_camera_matrix, dist, rvecs, tvecs)

# ----------------- Main code -----------------
object_points_3D = prepare_3D_object_points(SQUARE_SIZE)
criteria = set_termination_criteria()
get_images_and_draw_corners_on_them(DISTORTED_IMAGE_FOLDER)

process_images(DISTORTED_IMAGE_PATH)

if DEBUG_IMAGES:
  close_all_cv_windows()