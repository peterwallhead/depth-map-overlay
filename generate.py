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
CAMERA_FOCAL_LENGTH_MM = 3.6 # Reserved for future use

CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_PX = CAMERA_VERTICAL_RESOLUTION_PX / 2
CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_MM = 130 

# LiDAR is mounted above and behind the camera
LIDAR_VERTICAL_OFFSET_MM = 45
LIDAR_DISTANCE_OFFSET_MM = -120

# Approximates the number of pixels per mm
VERTICAL_PX_PER_MM =  CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_PX / CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_MM

# Approximates the width for each depth map marker
HORIZONTAL_PX_PER_DEGREES_FOV = CAMERA_HORIZONTAL_RESOLUTION_PX / CAMERA_HORIZONTAL_FOV_DEGREES

# Calculates the y position of the depth map markers
MARKER_Y = CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_PX - (LIDAR_VERTICAL_OFFSET_MM / VERTICAL_PX_PER_MM)

# Constrain data used from the LiDAR to the camera's FOV
MIN_HORIZONTAL_FOV = 360 - (CAMERA_HORIZONTAL_FOV_DEGREES / 2)
MAX_HORIZONTAL_FOV = CAMERA_HORIZONTAL_FOV_DEGREES / 2


def process_data_in_fov(data):
    data_in_fov = []
    maximum_distance_measurement = 0

    for angle, distance in data.items():
        if (float(angle) >= MIN_HORIZONTAL_FOV or float(angle) <= MAX_HORIZONTAL_FOV) and distance != 0.0:
            distance = (distance + LIDAR_DISTANCE_OFFSET_MM) * math.cos(math.radians(float(angle)))
        
            if distance > maximum_distance_measurement:
                maximum_distance_measurement = distance
        
            if float(angle) >= MIN_HORIZONTAL_FOV:
                angle_key = -(360 - float(angle))
            else:
                angle_key = float(angle)

            data_in_fov.append((angle_key, distance))

    return data_in_fov, maximum_distance_measurement


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Map Overlay Generator')
    parser.add_argument('-i', '--image', help='Scene image',
                        default='input/example_scene.jpg')
    parser.add_argument('-d', '--data', help='LiDAR distance measurement data',
                        default='input/example_data.py')
    args = parser.parse_args()

    with open(args.data, 'r', encoding="utf-8") as f:
        data = ast.literal_eval(f.read())

    data_in_fov, maximum_distance_measurement = process_data_in_fov(data)

    with Image.open(args.image) as im:
        grayscale_im = im.convert('L')
        im = grayscale_im.convert('RGBA')
        overlay = Image.new('RGBA', im.size, (255, 255, 255, 0)) 

        draw = ImageDraw.Draw(overlay)

        for angle, distance in data_in_fov:
            point_opacity = int((1 - (distance / maximum_distance_measurement)) * 100)
        
            draw.circle((((CAMERA_HORIZONTAL_RESOLUTION_PX / 2) + (HORIZONTAL_PX_PER_DEGREES_FOV * angle)), MARKER_Y), HORIZONTAL_PX_PER_DEGREES_FOV, (0, 255, 0, point_opacity))

        overlay.save('output/points_only.png')
        
        combined = Image.alpha_composite(im, overlay)

        combined.save('output/combined.png')