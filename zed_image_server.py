#!/home/poincarepoint/anaconda3/envs/pyzed_env/bin/python

import io
import socket
import struct
import threading

from PIL import Image
import numpy as np
import pyzed.sl as sl

# Server Configuration
HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 9999
clients = []  # List to keep track of connected clients
lock = threading.Lock()  # Lock for thread-safe operations


def handle_client(client_socket, addr):
    """Handles sending images to a connected client."""
    print(f"Client connected: {addr}")
    try:
        while True:
            with lock:
                if not clients:
                    break  # Stop sending if there are no clients

            # Capture image
            bgra_image = sl.Mat()
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(bgra_image, sl.VIEW.LEFT)
                converted_rgb_image = bgra_image.get_data()[:, :, :3][:, :, ::-1]
                pil_image = Image.fromarray(converted_rgb_image)

                # Convert image to bytes
                img_byte_arr = io.BytesIO()
                pil_image.save(img_byte_arr, format="PNG")
                image_bytes = img_byte_arr.getvalue()

                # Send image size
                client_socket.sendall(struct.pack(">I", len(image_bytes)))

                # Send image data
                client_socket.sendall(image_bytes)
    except (BrokenPipeError, ConnectionResetError):
        print(f"Client {addr} disconnected.")
    finally:
        with lock:
            clients.remove(client_socket)
        client_socket.close()


def accept_clients(server_socket):
    """Accepts new client connections and starts a thread for each."""
    while True:
        client_socket, addr = server_socket.accept()
        with lock:
            clients.append(client_socket)
        threading.Thread(target=handle_client, args=(client_socket, addr), daemon=True).start()


# Initialize ZED Camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.VGA
init_params.camera_fps = 1
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
init_params.coordinate_units = sl.UNIT.MILLIMETER

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open ZED camera. Exiting.")
    exit()

runtime_parameters = sl.RuntimeParameters()

# Start the server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
print(f"Server listening on {HOST}:{PORT}")

# Start accepting clients
threading.Thread(target=accept_clients, args=(server_socket,), daemon=True).start()

try:
    while True:
        pass  # Keep the main thread alive
except KeyboardInterrupt:
    print("\nShutting down server.")
    server_socket.close()
    zed.close()
