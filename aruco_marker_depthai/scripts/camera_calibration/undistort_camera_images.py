import pickle
import cv2
import os

# Path to the image file; change this to the path of the image you want to undistort
IMAGE_PATH = '/home/danoon/shared/aruco-marker-detection/Media/images_oakd_camera/chessboard_oakd_1.jpg'
PICKLE_FILE_PATH = '/home/danoon/shared/aruco-marker-detection/aruco_marker_depthai/scripts/camera_calibration/calibration_values/camera_calibration_pickle_file.pkl'

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

def get_calibration_values():
    """
    A function to get the calibration values from the pickle file
    
    Returns:
      mtx: The camera matrix
      optimal_camera_matrix: The optimal camera matrix
      dist: The distortion coefficients
    """
    pickle_file = open(PICKLE_FILE_PATH, 'rb')
    calibration_values = pickle.load(pickle_file)

    mtx = calibration_values["mtx"]
    optimal_camera_matrix = calibration_values["optimal_camera_matrix"]
    dist = calibration_values["dist"]

    return mtx, optimal_camera_matrix, dist

def undistort_image():
    """ A function to undistort an image using the camera calibration values"""
    output_directory, file_name_without_extension = prepare_output_directory_and_filename(IMAGE_PATH, 'undistorted')
    distorted_image = load_image(IMAGE_PATH)

    mtx, optimal_camera_matrix, dist = get_calibration_values()
    undistorted_image = cv2.undistort(distorted_image, mtx, dist, None, optimal_camera_matrix)

    save_undistorted_example_image(output_directory, file_name_without_extension, undistorted_image)

undistort_image()