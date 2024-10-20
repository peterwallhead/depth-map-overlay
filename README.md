# Depth Map Overlay

## Objective

Overlay a graphical representation of distance readings from a 2D LiDAR scanner onto a POV image taken of a mobile robot's FOV.

![](output/combined.png)

## Install
```
pipenv install --dev
```

## Usage
### Inputs (input/)
- An image of the required scene in your preferred format matching the configured resolution below.
- A Python dictionary of raw LiDAR distance measurements.

### Outputs (output/)
- combined.jpg - Contains a grayscale version of the scene image with the relative distance measurements overlayed as green dots. The more solid a green dot is the closer that point is to the camera.
- points_only.jpg - Contains only the green dot points used in the combined image.

### Capture image from webcam (optional)
```
pipenv run python tools/capture_scene_image.py --camera_source 0 --filename input/scene.jpg
```
### Generate composite image
```
pipenv run python generate.py --image input/scene.jpg --data input/data.py
```

## Components
- 2D RPLiDAR scanner (RPLiDAR C1)
- Generic USB webcam (Hikvision DS-U02)

## Configuration Settings
Adjust in ```generate.py```.
- CAMERA_HORIZONTAL_FOV_DEGREES = 86
- CAMERA_VERTICAL_FOV_DEGREES = 55
- CAMERA_HORIZONTAL_RESOLUTION_PX = 640
- CAMERA_VERTICAL_RESOLUTION_PX = 480
- CAMERA_VERTICAL_CENTREPOINT_IN_FRAME_MM = 130
- LIDAR_VERTICAL_OFFSET_MM = 45
- LIDAR_DISTANCE_OFFSET_MM = -120

## Limitations
- The depth data is projected as a straight line across the scene and is a close fit to reality, but not an exact representation.
- For now, the camera's focal length is not taken into account when rendering the final image.
- The format and parsing of the LiDAR data file is not ideal (working with data generated for another project).
