import io
import time
import socket
import struct

import torch
from PIL import Image
import numpy as np
import cv2
from transformers import AutoProcessor
from transformers import AutoModelForVision2Seq
import pyzed.sl as sl
import json
from time import sleep


def send_franka_eef_pose(client_socket, pose):
    try:
        pose_data = json.dumps(pose.tolist())

        # Send the length of the message first (fixed 4-byte header)
        msg_length = len(pose_data).to_bytes(4, "big")
        client_socket.sendall(msg_length)

        # Send the actual JSON data
        client_socket.sendall(pose_data.encode("utf-8"))

        # Wait for the server to acknowledge the message
        ack = client_socket.recv(4)
        if ack != b"ACK":
            print("Server did not acknowledge the message.")

    except Exception as e:
        print(f"Error sending data: {e}")


def main():
    HOST = "192.168.168.230"
    PORT = 9999
    print(f"[*] Connecting to {HOST}:{PORT}...")
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    print("[+] Connected to the server.")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 1
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera. Exiting.")
        exit()

    runtime_parameters = sl.RuntimeParameters()
    # Capture image
    bgra_image = sl.Mat()
    # OpenVLLM Initialization
    # Load the vision-language model
    processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
    vla = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b",
        attn_implementation="flash_attention_2",
        torch_dtype=torch.bfloat16,
        low_cpu_mem_usage=True,
        trust_remote_code=True,
    ).to("cuda:0")
    # prompt = "In: Pick up the mug and place it in black bin\nOut:"
    prompt = "Pickup corn"
    while 1:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(bgra_image, sl.VIEW.LEFT)
            # converted_rgb_image = bgra_image.get_data()[:, :, :3][:, :, ::-1]
            converted_rgb_image = bgra_image.get_data()[:, :, [2, 1, 0, 3]]
            pil_image = Image.fromarray(converted_rgb_image, mode="RGBA")
            inputs = processor(prompt, pil_image).to("cuda:0", dtype=torch.bfloat16)
            action = vla.predict_action(**inputs, unnorm_key="viola", do_sample=False)
            print(action)  # Print or log the predicted action
            send_franka_eef_pose(client_socket, action)
            numpy_image = np.array(pil_image)
            cv2_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("ZED", cv2_image)
            cv2.waitKey(1)
            # sleep(0.5)


if __name__ == "__main__":
    main()
