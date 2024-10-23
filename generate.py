import sys
import math

import argparse
import ast

from PIL import Image, ImageDraw


# Config
CAMERA_HORIZONTAL_FOV_DEGREES = 86
CAMERA_VERTICAL_FOV_DEGREES = 55
CAMERA_HORIZONTAL_RESOLUTION_PX = 640
CAMERA_VERTICAL_RESOLUTION_PX = 480
CAMERA_FOCAL_LENGTH_MM = 3.6

CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_PX = CAMERA_VERTICAL_RESOLUTION_PX / 2
CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_MM = 130 

# LiDAR is mounted above and behind the camera
LIDAR_VERTICAL_OFFSET_MM = 55
LIDAR_DISTANCE_OFFSET_MM = -120

# Approximates the number of pixels per mm
VERTICAL_PX_PER_MM =  CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_PX / CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_MM

# Calculates the y position of the depth map markers
MARKER_Y = CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_PX - (LIDAR_VERTICAL_OFFSET_MM / VERTICAL_PX_PER_MM)

VANISHING_POINT_Y = CAMERA_VERTICAL_RESOLUTION_PX - (CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_MM * CAMERA_FOCAL_LENGTH_MM / VERTICAL_PX_PER_MM) - (LIDAR_VERTICAL_OFFSET_MM / VERTICAL_PX_PER_MM)

def depth_to_color(value):
    """Convert a value between 0 and 1 to a color between blue (0) and red (1)."""
    if not (0 <= value <= 1):
        raise ValueError("Value should be between 0 and 1")

    # Interpolate between blue (0, 0, 255) and red (255, 0, 0)
    r = int(255 * value)
    g = 0  # green remains constant at 0
    b = int(255 * (1 - value))

    return (r, g, b)

def calculate_initial_camera_fov_distance(data):
    for angle, distance in data.items():
        if float(angle) == 0.0:
            return distance

def calculate_lidar_fov(initial_fov_degrees, distance, distance_offset):
    initial_fov_radians = math.radians(initial_fov_degrees)
    
    initial_width_half = distance * math.tan(initial_fov_radians / 2)
    
    new_distance = distance - distance_offset
    
    new_fov_half_radians = math.atan(initial_width_half / new_distance)
    
    new_fov_degrees = math.degrees(new_fov_half_radians * 2)

    min_horizontal_lidar_fov = 360 - (new_fov_degrees / 2)
    max_horizontal_lidar_fov = new_fov_degrees / 2
    
    return int(new_fov_degrees), int(min_horizontal_lidar_fov), int(max_horizontal_lidar_fov)

def process_data_in_fov(data):
    data_in_fov = []
    maximum_distance_measurement = 0

    # As the LiDAR is offset to the camera it has a different FOV to the camera
    initial_camera_fov_distance = calculate_initial_camera_fov_distance(data)
    horizontal_lidar_fov, min_horizontal_lidar_fov, max_horizontal_lidar_fov = calculate_lidar_fov(CAMERA_HORIZONTAL_FOV_DEGREES, initial_camera_fov_distance, LIDAR_DISTANCE_OFFSET_MM)
    
    # Approximates the width for each depth map marker
    horizontal_px_per_degress_fov = CAMERA_HORIZONTAL_RESOLUTION_PX / horizontal_lidar_fov

    for angle, distance in data.items():
        if (float(angle) >= min_horizontal_lidar_fov or float(angle) <= max_horizontal_lidar_fov) and distance != 0.0:
            distance = int((distance + LIDAR_DISTANCE_OFFSET_MM) * math.cos(math.radians(float(angle))))
        
            if float(angle) >= min_horizontal_lidar_fov:
                angle_key = -(360 - float(angle))
            else:
                angle_key = float(angle)

            data_in_fov.append((angle_key, distance))

            if distance > maximum_distance_measurement:
                maximum_distance_measurement = distance
                vanishing_point_x = (CAMERA_HORIZONTAL_RESOLUTION_PX / 2) + (horizontal_px_per_degress_fov * angle_key)
                vanishing_point_x_angle_key = angle_key

    data = sorted(data_in_fov, key=lambda x: x[0])

    return data, maximum_distance_measurement, vanishing_point_x, vanishing_point_x_angle_key, horizontal_px_per_degress_fov

def calculate_slope_point_y_spacing(points, vanishing_point_angle):
    num_points_positive = 0
    num_points_negative = 0

    for angle, distance in points:
        if angle <= vanishing_point_angle:
            num_points_positive += 1
        elif angle > vanishing_point_angle:
            num_points_negative += 1

    return num_points_positive, num_points_negative

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Map Overlay Generator')
    parser.add_argument('-i', '--image', help='Scene image',
                        default='input/example_scene.jpg')
    parser.add_argument('-d', '--data', help='LiDAR distance measurement data',
                        default='input/example_data.py')
    parser.add_argument('-p', '--preview', help='Show combined image after generation',
                        default=False)
    args = parser.parse_args()

    with open(args.data, 'r', encoding="utf-8") as f:
        data = ast.literal_eval(f.read())

    data_in_fov, maximum_distance_measurement, VANISHING_POINT_X, vanishing_point_x_angle_key, HORIZONTAL_PX_PER_DEGREES_FOV = process_data_in_fov(data)

    with Image.open(args.image) as im:
        grayscale_im = im.convert('L')
        im = grayscale_im.convert('RGBA')
        overlay = Image.new('RGBA', im.size, (255, 255, 255, 0)) 

        draw = ImageDraw.Draw(overlay)

        num_points_positive, num_points_negative = calculate_slope_point_y_spacing(data_in_fov, vanishing_point_x_angle_key)

        i = 0
        for angle, distance in data_in_fov:
            if angle <= vanishing_point_x_angle_key:
                t = i / num_points_positive
                x = (CAMERA_HORIZONTAL_RESOLUTION_PX / 2) + (HORIZONTAL_PX_PER_DEGREES_FOV * angle)
                y = int(MARKER_Y + t * (VANISHING_POINT_Y - MARKER_Y))

                point_opacity = int((1 - (distance / maximum_distance_measurement)) * 100)
                draw.circle((x,y), HORIZONTAL_PX_PER_DEGREES_FOV, (255, 0, 0, point_opacity))

                i += 1

        i = 0
        for angle, distance in data_in_fov:
            if angle > vanishing_point_x_angle_key:
                t = i / num_points_negative
                x = (CAMERA_HORIZONTAL_RESOLUTION_PX / 2) + (HORIZONTAL_PX_PER_DEGREES_FOV * angle)
                y = int(VANISHING_POINT_Y + t * (MARKER_Y - VANISHING_POINT_Y))

                point_opacity = int((1 - (distance / maximum_distance_measurement)) * 100)
                draw.circle((x,y), HORIZONTAL_PX_PER_DEGREES_FOV, (255, 0, 0, point_opacity))

                i += 1
        
        # Uncomment the following 3 lines to help with debugging
        # draw.circle((VANISHING_POINT_X, VANISHING_POINT_Y), HORIZONTAL_PX_PER_DEGREES_FOV, (255, 0, 0, 100))
        # draw.line((0, MARKER_Y, VANISHING_POINT_X, VANISHING_POINT_Y), fill='red', width=1)
        # draw.line((VANISHING_POINT_X, VANISHING_POINT_Y, 640, MARKER_Y,), fill='red', width=1)
            
        overlay.save('output/points_only.png')
        
        combined = Image.alpha_composite(im, overlay)

        combined.save('output/combined.png')

        if args.preview:
            combined.show()