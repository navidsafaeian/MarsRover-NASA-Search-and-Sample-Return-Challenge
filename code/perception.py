import numpy as np
import cv2
import time

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def navigable_thresh(img, rgb_thresh=(160, 160, 160)):
    """Identify the image pixels that are above the provided threshold. Each
    threshold value can range between 0 and 255, with 160 doing a nice job
    of identifying ground pixels only.
    :param img: Numpy 3d array (x, y, RGB layers)
    :param rgb_threh: 3 item tuple of integers specifying the threshold for
        excluding pixel values
    :return: Numpy 2d array (x, y) of the binary image
    """
    # Create an array of zeros with the same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in rbg_thresh.
    #   Values above the threshold will now contain a boolean array with TRUE.
    above_thresh = ((img[:,:,0] > rgb_thresh[0]) &
                    (img[:,:,1] > rgb_thresh[1]) &
                    (img[:,:,2] > rgb_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    return color_select


def obstacle_thresh(img, rgb_thresh=(160, 160, 160)):
    """Identify the image pixels that are above the provided threshold. Each
    threshold value can range between 0 and 255, with 160 doing a nice job
    of identifying ground pixels only.
    :param img: Numpy 3d array (x, y, RGB layers)
    :param rgb_threh: 3 item tuple of integers specifying the threshold for
        excluding pixel values
    :return: Numpy 2d array (x, y) of the binary image
    """
    # Create an array of zeros with the same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be below all three threshold values in rbg_thresh.
    #   Values below the threshold will now contain a boolean array with TRUE.
    below_thresh = ((img[:,:,0] < rgb_thresh[0]) &
                    (img[:,:,1] < rgb_thresh[1]) &
                    (img[:,:,2] < rgb_thresh[2]))
    # Index the array of zeros with the boolean array and set to 1
    color_select[below_thresh] = 1
    return color_select


def rock_thresh(img):
    """Check the image for yellow rocks using OpenCV for determining the
    mask required.
    :param img: Numpy 3d array (x, y, RGB layers)
    :return: Numpy 2d array (x, y) of the binary image
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV, 3)
    
    # Define range of yellow colors in HSV
    lower_yellow = np.array([20, 150, 100], dtype='uint8')
    upper_yellow = np.array([50, 255, 255], dtype='uint8')
    
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    return mask 


def rover_coords(binary_img):
    """Convert the binary image to have rover centric coordinates by
    translating it so that the base of the identified area is at (0, 0). The
    rover's x axis represents the front of the rover.
    :param binary_img: Numpy 2d array (x, y) of the binary image
    :return: Tuple of Numpy 1d float arrays of the x and y pixels after the
        original pixels have been translated
    """
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Translate the pixel positions with reference to the rover position being
    #   at the center bottom of the image. Must flip the xpos and ypos.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    """Convert the cartesian coordinates (x, y) to polar coordinates (distance,
    angle). This is used when calculating the average angle of the navigable
    path, which is needed to determine the path the rover should go.
    :param x_pixel: Numpy 1d array of the x binary pixels
    :param y_pixel: Numpy 1d array of the y binary pixels
    :return: Tuple of Numpy 1d arrays of the distance and angles between the
        grid origin and the specific pixel
    """
    # Calculate distance to each pixel using pythagoream theorem
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel using arc tangent
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


def rotate_pix(xpix, ypix, yaw):
    """Apply a matrix rotation to the provided x and y pixel arrays so they
    are parallel to the world space axes. Use the specified yaw degrees to
    determine the rotation amount. Must convert the yaw degrees into yaw
    radians before performing the matrix rotation.
    :param xpix: Numpy 1d array of the x binary pixels
    :param ypix: Numpy 1d array of the y binary pixels
    :param yaw: Float of the rover's current yaw degree
    :return: Tuple of Numpy 1d arrays of the rotated x and y pixels
    """
    # Convert yaw degrees to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a matrix rotation counter-clockwise by the yaw radians
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    """Translate the rotated pixel arrays by the x and y position values
    given by the rover's location (position vector) in the world. The
    rotated x and y pixels must be scaled before being translated to
    account for the scale difference between rover space and world space.
    :param xpix_rot: Numpy 1d array of rotated x binary pixels
    :param ypix_rot: Numpy 1d array of rotated y binary pixels
    :param xpos: Float of the rover's current x position
    :param ypos: Float of the rover's current y position
    :param scale: Integer of how the rotated x and y pixel arrays should be
        scaled before being translated
    :return: Tuple of Numpy 1d arrays of the scaled and translated x and y
        pixel arrays
    """
    # Scale and translate the rotated pixel arrays
    xpix_translated = (xpos + (xpix_rot / scale))
    ypix_translated = (ypos + (ypix_rot / scale))
    return xpix_translated, ypix_translated


def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale=10):
    """Map the rover centric coordinates to the world coordinates by rotating,
    scaling, translating and clipping them.
    :param xpix: Numpy 1d array of the x rover centric binary pixels
    :param ypix: Numpy 1d array of the y rover centric binary pixels
    :param xpos: Float of the rover's current x position
    :param ypos: Float of the rover's current y position
    :param yaw: Float of the rover's current yaw degree
    :param world_size: Integer of the world size
    :param scale: Integer of the scale difference between rover coordinates
        and world coordinates
    :return: Tuple of Numpy 1d arrays of the provided x and y coordinates
        mapped to the world coordinates
    """
    # Rotate the coordinates so x and y axes are parallel to the world axes
    xpix_rot, ypix_rot = rotate_pix(xpix=xpix, ypix=ypix, yaw=yaw)
    # Translate the rotated arrays by the rover's location in the world
    xpix_tran, ypix_tran = translate_pix(xpix_rot=xpix_rot, ypix_rot=ypix_rot,
                                         xpos=xpos, ypos=ypos, scale=scale)
    # Clip the array ranges to fit within the world map
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    return x_pix_world, y_pix_world


def perspect_transform(img, src, dst):
    """Perform a perspective transformation on the raw image array to change
    the rover's camera image to a synthetic map image.
    :param img: Numpy 3d array (x, y, RGB) of the rover's camera
    :param src: Numpy 2d array of integers indicating a square in the raw image
    :param dst: Numpy 2d array of floats indicating how the square from the
        raw image should be transformed to make a perfect square for the output
    :return: Numpy 3d array (x, y, RGB) of the synthetic map image
    """
    M = cv2.getPerspectiveTransform(src, dst)
    # Keep same size as input image
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    return warped


def perception_step(Rover):
    """Calculate the Rover's current environment from the position values and
    the front camera image. Update the Rover's state after perception.
    :param Rover: Class of the Rover's state
    :return: Updated Rover class
    """
    # Camera image from the current Rover state (Rover.img)
    img = Rover.img
   
    # 1) Define source and destination points for perspective transform
    # Define calibration box in source and destination coordintates.
    #   These source and destination points are defined to warp the image
    #   to a grid where each 10x10 pixel square represents 1 square meter
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    #   is not the position of the rover but a bit in front of it
    bottom_offset = 6
    src = np.float32([[14, 140], [300, 140], [200, 96], [118, 96]])
    dst = np.float32([
        [img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
        [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
        [img.shape[1]/2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
        [img.shape[1]/2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset]])

    # 2) Apply perspective transform
    warped = perspect_transform(img=img, src=src, dst=dst)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable = navigable_thresh(img=warped, rgb_thresh=(160, 160, 160)) 
    obstacles = obstacle_thresh(img=warped, rgb_thresh=(140, 140, 140))
    rock_samples = rock_thresh(img=warped)

    # 4) Update Rover.vision_image (displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacles * 255
    Rover.vision_image[:,:,1] = rock_samples * 255
    Rover.vision_image[:,:,2] = navigable * 255

    # 5) Convert map image pixel values to rover centric coordinates
    navigable_xpix, navigable_ypix = rover_coords(binary_img=navigable)
    obstacles_xpix, obstacles_ypix = rover_coords(binary_img=obstacles)
    rocks_xpix, rocks_ypix = rover_coords(binary_img=rock_samples)

    # 6) Convert rover centric pixel values to world coordinates
    scale = dst_size * 2
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    worldmap_size = Rover.worldmap.shape[0]

    navigable_x_world, navigable_y_world = pix_to_world(
        xpix=navigable_xpix, ypix=navigable_ypix,
        xpos=xpos, ypos=ypos, yaw=yaw, world_size=worldmap_size, scale=scale)
    obstacles_x_world, obstacles_y_world = pix_to_world(
        xpix=obstacles_xpix, ypix=obstacles_ypix,
        xpos=xpos, ypos=ypos, yaw=yaw, world_size=worldmap_size, scale=scale)
    rocks_x_world, rocks_y_world = pix_to_world(
        xpix=rocks_xpix, ypix=rocks_ypix,
        xpos=xpos, ypos=ypos, yaw=yaw, world_size=worldmap_size, scale=scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    if (Rover.pitch < 0.5 or Rover.pitch > 359.5) and (Rover.roll < 0.5 or Rover.roll > 359.5):
        # Limit world map updates to only images that have limited roll and pitch
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] = 255
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    distances, angles = to_polar_coords(x_pixel=navigable_xpix,
                                        y_pixel=navigable_ypix)
    # Update Rover pixel distances and angles
    Rover.nav_dists = distances
    Rover.nav_angles = angles

    if len(rocks_xpix) > 5:
        # If a rock is identified, make the rover navigate to it
        rock_distance, rock_angle = to_polar_coords(x_pixel=rocks_xpix,
                                                    y_pixel=rocks_ypix)
        Rover.rock_dist = rock_distance
        Rover.rock_angle = rock_angle 
        if not Rover.sample_seen:
            # First frame sample has been seen, thus start the sample timer
            Rover.sample_timer = time.time()
        Rover.sample_seen = True

    if Rover.start_pos is None:
        Rover.start_pos = (Rover.pos[0], Rover.pos[1])
        print('STARTING POSITION IS: ', Rover.start_pos)

    
    return Rover