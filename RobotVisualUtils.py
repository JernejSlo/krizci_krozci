import pyrealsense2 as rs
import numpy as np
import cv2
import os

class RobotVisualUtils():

    def __init__(self):
        pass

    def capture_realsense_image(self,output_path: str):
        """
        Captures an image and depth data using the Intel RealSense D405 camera,
        disabling auto-exposure to fix high contrast issues.
        Saves the results to the specified output path.
        """
        os.makedirs(output_path, exist_ok=True)

        # Configure RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()

        # Enable streams (color and depth)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start the pipeline
        profile = pipeline.start(config)

        try:
            # Wait for a frame
            frames = pipeline.wait_for_frames()

            # Get depth and color frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                print("Failed to capture frames. Please check camera connection.")
                return

            # Convert frames to NumPy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply contrast normalization (if needed)
            color_image = cv2.normalize(color_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # Apply colormap to depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Save paths
            color_image_path = os.path.join(output_path, "color_image.png")
            depth_image_path = os.path.join(output_path, "depth_image.npy")
            depth_colormap_path = os.path.join(output_path, "depth_colormap.png")

            # Save images
            cv2.imwrite(color_image_path, color_image)
            np.save(depth_image_path, depth_image)
            cv2.imwrite(depth_colormap_path, depth_colormap)

            print(f"Saved color image: {color_image_path}")
            print(f"Saved depth data: {depth_image_path}")
            print(f"Saved depth colormap: {depth_colormap_path}")

        finally:
            # Stop the pipeline
            pipeline.stop()

    def crop_image(self, image: np.ndarray, margins: list):
        """
        Crops an image from each side based on the given margins.

        Args:
            image (np.ndarray): The input image as a NumPy array.
            margins (list): A list of four integers [x1, x2, y1, y2]
                            where:
                            - x1: pixels to remove from the left
                            - x2: pixels to remove from the right
                            - y1: pixels to remove from the top
                            - y2: pixels to remove from the bottom

        Returns:
            np.ndarray: The cropped image.
        """
        if not isinstance(image, np.ndarray):
            raise ValueError("Input must be a NumPy array (image).")

        if len(margins) != 4:
            raise ValueError("Margins must be a list of four integers: [x1, x2, y1, y2].")

        x1, x2, y1, y2 = margins
        height, width = image.shape[:2]  # Get original image dimensions

        # Ensure cropping does not exceed image dimensions
        x1 = max(0, x1)
        x2 = max(0, x2)
        y1 = max(0, y1)
        y2 = max(0, y2)

        new_width = max(1, width - x1 - x2)  # Ensure at least 1 pixel width
        new_height = max(1, height - y1 - y2)  # Ensure at least 1 pixel height

        # Crop the image
        cropped_image = image[y1:y1 + new_height, x1:x1 + new_width]

        return cropped_image

    def slice_image_into_grid(self, image: np.ndarray, grid_size=(3, 3)):
        """
        Slices an image into a grid of size grid_size (default 3x3).

        Args:
            image (np.ndarray): The input image as a NumPy array.
            grid_size (tuple): The number of rows and columns in the grid (default is (3,3)).

        Returns:
            list: A list containing the sub-images as NumPy arrays.
        """
        if not isinstance(image, np.ndarray):
            raise ValueError("Input must be a NumPy array (image).")

        rows, cols = grid_size
        height, width = image.shape[:2]  # Get original image dimensions

        # Compute dimensions of each sub-image
        sub_h = height // rows
        sub_w = width // cols

        sub_images = []

        # Slice the image into grid parts
        for i in range(rows):
            for j in range(cols):
                x_start, y_start = j * sub_w, i * sub_h
                x_end, y_end = x_start + sub_w, y_start + sub_h
                sub_image = image[y_start:y_end, x_start:x_end]
                sub_images.append(sub_image)

        return sub_images

    def count_dark_pixels(self, image: np.ndarray, threshold: int = 50):
        """
        Converts an image to grayscale and counts the number of dark pixels.

        Args:
            image (np.ndarray): The input image as a NumPy array.
            threshold (int): The pixel intensity threshold (default is 50).
                            Pixels with intensity <= threshold are considered "dark".

        Returns:
            int: The number of dark pixels in the image.
        """
        if not isinstance(image, np.ndarray):
            raise ValueError("Input must be a NumPy array (image).")

        # Convert image to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Count pixels that have intensity <= threshold
        dark_pixel_count = np.sum(gray_image <= threshold)

        return dark_pixel_count

    def determine_board(self,img):

        margins = [130, 130, 20, 60]
        cropped_img = utils.crop_image(img, margins)

        slices = self.slice_image_into_grid(cropped_img)

        board = []

        # Display the sliced images
        for idx, sub_img in enumerate(slices):
            piece = self.check_which_is_in_square(sub_img)
            board.append(piece)

        board = np.asarray(board).reshape((3,3))

        return board


    def check_which_is_in_square(self,img):
        cnt = self.count_dark_pixels(img)
        if cnt > 1000:
            return "O"
        elif cnt > 120:
            return "X"
        else:
            return ""

utils = RobotVisualUtils()
#utils.capture_realsense_image("./sense/")

image = cv2.imread("./sense/color_image.png")  # Load an image
margins = [130, 130, 20, 60]  # Crop 50px from left, 30px from right, 20px from top, 40px from bottom

print(utils.determine_board(image))

"""cropped_img = utils.crop_image(image, margins)"""

"""cv2.imshow("Cropped Image", cropped_img)  # Display the cropped image
cv2.waitKey(0)
cv2.destroyAllWindows()"""

"""# Example Usage:
slices = utils.slice_image_into_grid(cropped_img)

# Display the sliced images
for idx, sub_img in enumerate(slices):
    cv2.imshow(f"Slice {idx + 1}", sub_img)
    dark_pixels = utils.count_dark_pixels(sub_img)

    print(f"Number of dark pixels: {dark_pixels}")

cv2.waitKey(0)
cv2.destroyAllWindows()"""

