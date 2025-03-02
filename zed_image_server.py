#!/home/poincarepoint/anaconda3/envs/pyzed_env/bin/python

import numpy as np
import pyzed.sl as sl
import socket
import struct
from PIL import Image
import io


def main():
    host = "127.0.0.1"
    port = 5001
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 opr HD1200 video mode, depending on camera type.
    init_params.camera_fps = 30  # Set fps at 30
    # init_params.sdk_verbose = True  # Enable verbose logging
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Set the depth mode to performance (fastest)
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : " + repr(err) + ". Exit program.")
        exit()

    # Capture 50 frames and stop
    i = 0
    bgra_image = sl.Mat()
    depth_image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    while i < 50:
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(bgra_image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)
            converted_rgb_image = bgra_image.get_data()[:, :, :3][:, :, ::-1]
            pil_image = Image.fromarray(converted_rgb_image)
            # Save PIL image
            # pil_image.save("image_" + str(i) + ".png")
            # break
            img_byte_arr = io.BytesIO()
            pil_image.save(img_byte_arr, format="PNG")  # Ensuring consistent format
            image_bytes = img_byte_arr.getvalue()

            # Send the image size first (big-endian, 4 bytes)
            client_socket.sendall(struct.pack(">I", len(image_bytes)))

            # Send the image data
            client_socket.sendall(image_bytes)
            print(f"Sent image {i}")
            i += 1

    zed.close()
    client_socket.close()


if __name__ == "__main__":
    main()
