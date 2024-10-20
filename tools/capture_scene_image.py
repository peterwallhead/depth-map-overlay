import argparse
from time import sleep

import cv2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Scene Image Capture')
    parser.add_argument('-c', '--camera_source', help='Camera source ID',
                        default=0)
    parser.add_argument('-f', '--filename', help='Filename for captured image',
                        default='input/scene.jpg')
    args = parser.parse_args()


    webcam = cv2.VideoCapture(args.camera_source)
    sleep(2)
    while True:

        try:
            check, frame = webcam.read()
            cv2.imshow('Capturing', frame)
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.imwrite(filename=args.filename, img=frame)
            webcam.release()
            cv2.destroyAllWindows()
            print('Image saved!')
            
            break
    
        except(KeyboardInterrupt):
            print('Turning off camera.')
            webcam.release()
            print('Camera off.')
            print('Program ended.')
       
            break