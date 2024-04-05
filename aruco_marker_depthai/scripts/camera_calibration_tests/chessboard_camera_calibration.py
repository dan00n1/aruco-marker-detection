import cv2
import numpy as np
import glob
import os

# For testing purposes:
DEBUG_IMAGES = True
DEBUG_PARAMETERS = True

DISTORTED_IMAGE_FOLDER = '/home/danoon/shared/aruco-marker-detection/Media/test_images_phone_camera'
DISTORTED_IMAGE = 'chessboard_input2.jpeg'
DISTORTED_IMAGE_PATH = os.path.join(DISTORTED_IMAGE_FOLDER, DISTORTED_IMAGE)

NEW_FOLDER_NAME = "undistorted"

# Chessboard dimensions
NUMBER_OF_SQUARES_X = 10 # Nr of chessboard squares along the x-axis
NUMBER_OF_SQUARES_Y = 7  # Nr of chessboard squares along the y-axis
nX = NUMBER_OF_SQUARES_X - 1 # Nr of interior corners along x-axis
nY = NUMBER_OF_SQUARES_Y - 1 # Nr of interior corners along y-axis
SQUARE_SIZE = 0.023 # Length of the side of a square in meters

# Store vectors of 3D points for all chessboard images (world coordinate frame)
object_points = []
 
# Store vectors of 2D points for all chessboard images (camera coordinate frame)
image_points = []

def set_termination_criteria():
  # We stop either when an accuracy is reached or when we have finished a certain number of iterations.
  return (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
def prepare_3D_object_points(square_size):
  # Define real world coordinates for points in the 3D coordinate frame
  # Object points are (0,0,0), (1,0,0), (2,0,0) ...., (5,8,0)
  object_points_3D = np.zeros((nX * nY, 3), np.float32)       
  
  # These are the x and y coordinates                                              
  object_points_3D[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2) 
  
  object_points_3D = object_points_3D * square_size

  return object_points_3D

def prepare_output_directory_and_filename(file_path, folder_name):
  # Extract the directory path and file name
  directory, file_name = os.path.split(file_path)
  # Create the output file name by removing the file extension part
  file_name_without_extension = os.path.splitext(file_name)[0]

  # Create the output directory path & ensure the output directory exists, if not, create it
  output_directory = os.path.join(directory, folder_name)
  os.makedirs(output_directory, exist_ok=True)

  return output_directory, file_name_without_extension

def get_images_and_draw_corners_on_them(directory_path):
  pattern = os.path.join(directory_path, '*.jpeg')

  # Get a list of all JPEG files in the directory (not including subdirectories)
  images = glob.glob(pattern)
 
  # Go through each chessboard image, one by one
  for image_file in images:
  
    # Load the image
    image = cv2.imread(image_file)
 
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
 
    # Find the corners on the chessboard
    corners_found, corners = cv2.findChessboardCorners(gray_image, (nY, nX), None)

    if corners_found:
      draw_corners(image, gray_image, corners)

def draw_corners(image, gray_image, corners):
  # Append object points
  object_points.append(object_points_3D)

  # Find more exact corner pixels       
  corners_2 = cv2.cornerSubPix(gray_image, corners, (11,11), (-1,-1), criteria)       
    
  # Append image points
  image_points.append(corners_2)

  # Draw the corners
  cv2.drawChessboardCorners(image, (nY, nX), corners_2, True)

  if DEBUG_IMAGES: 
    display_image(image)

def display_image(image):
  """ A function to display the image until any key is pressed"""
  cv2.imshow("Image", image) 
  cv2.waitKey(200)

def load_image(file_path):
    """ A function to load an image from the file path """
    try:
        # Load an image
        image = cv2.imread(file_path)
        if image is None:
            raise FileNotFoundError("Could not load image")
        return image
    except Exception as e:
        print(f"Error loading image: {e}")
        return None
    
def load_example_distorted_image(file_path): 
  distorted_image = load_image(file_path)

  if distorted_image is not None:
    gray_image = cv2.cvtColor(distorted_image, cv2.COLOR_BGR2GRAY)  
    return distorted_image, gray_image
  else:
    return None, None

def calibrate_camera(gray_image):
  # Return the camera matrix, distortion coefficients, rotation and translation vectors etc 
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray_image.shape[::-1], None, None)
  return mtx, dist, rvecs, tvecs

def get_image_dimensions(distorted_image):
  height, width = distorted_image.shape[:2]
  return height, width

def refine_camera_matrix(mtx, dist, height, width):
  """ Returns optimal camera matrix and a rectangular region of interest """
  optimal_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))
  return optimal_camera_matrix

def undistort_image(distorted_image, mtx, dist, optimal_camera_matrix):
  undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, optimal_camera_matrix)
  
  # Crop the image. Uncomment these two lines to remove black lines on the edge of the undistorted image.
  #x, y, w, h = roi
  #undistorted_image = undistorted_image[y:y+h, x:x+w]

  return undistorted_image
  
def display_key_parameter_outputs(optimal_camera_matrix, dist, rvecs, tvecs):
  print("Optimal Camera matrix:") 
  print(optimal_camera_matrix) 
 
  print("\n Distortion coefficient:") 
  print(dist) 
   
  print("\n Rotation Vectors:") 
  print(rvecs) 
   
  print("\n Translation Vectors:") 
  print(tvecs) 
 
def save_undistorted_image(output_directory, file_name_without_extension, undistorted_image):
  # Create the output file path
  new_filename = os.path.join(output_directory, file_name_without_extension + '_undistorted.jpg')

  # Save the undistorted image
  cv2.imwrite(new_filename, undistorted_image)
 
def close_all_cv_windows():
  cv2.destroyAllWindows()


object_points_3D = prepare_3D_object_points(SQUARE_SIZE)
criteria = set_termination_criteria()
get_images_and_draw_corners_on_them(DISTORTED_IMAGE_FOLDER)

def process_images(file_path):
  output_directory, file_name_without_extension = prepare_output_directory_and_filename(file_path, NEW_FOLDER_NAME)
  
  distorted_image, gray_image = load_example_distorted_image(file_path)
  height, width = get_image_dimensions(distorted_image)
  
  mtx, dist, rvecs, tvecs = calibrate_camera(gray_image)
  optimal_camera_matrix = refine_camera_matrix(mtx, dist, height, width)
  undistorted_image = undistort_image(distorted_image, mtx, dist, optimal_camera_matrix)
  save_undistorted_image(output_directory, file_name_without_extension, undistorted_image)

  if DEBUG_PARAMETERS:
    display_key_parameter_outputs(optimal_camera_matrix, dist, rvecs, tvecs)
  
process_images(DISTORTED_IMAGE_PATH)
if DEBUG_IMAGES:
  close_all_cv_windows()