import io
import time
import socket
import struct

import torch
from PIL import Image
from transformers import AutoProcessor
from transformers import AutoModelForVision2Seq
import pyzed.sl as sl

# Server Configuration
HOST = "127.0.0.1"  # Change this to the server's IP address if needed
PORT = 9999


def receive_image(client_socket):
    """Receives a single image from the server."""
    try:
        # Receive the image size first (4 bytes)
        data = client_socket.recv(4)
        if not data:
            return None

        image_size = struct.unpack(">I", data)[0]  # Read image size (big-endian)

        # Receive the image bytes
        image_data = b""
        while len(image_data) < image_size:
            packet = client_socket.recv(314095483)  # Receive in chunks
            if not packet:
                return None
            image_data += packet

        # Convert bytes to a PIL image
        return Image.open(io.BytesIO(image_data))

    except (ConnectionResetError, BrokenPipeError):
        print("[-] Connection lost. Reconnecting...")
        return None


def main():
    """Connects to the server and continuously processes incoming images."""

    # Load the vision-language model
    processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
    vla = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b",
        attn_implementation="flash_attention_2",
        torch_dtype=torch.bfloat16,
        low_cpu_mem_usage=True,
        trust_remote_code=True,
    ).to("cuda:0")

    prompt = "In: What action should the robot take to pick up the chilli?\nOut:"

    while True:
        try:
            print(f"[*] Connecting to {HOST}:{PORT}...")
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST, PORT))
            print("[+] Connected to the server.")

            while True:
                pil_image = receive_image(client_socket)
                if pil_image is None:
                    print("No image received. Reconnecting...")
                    continue  # Reconnect if no image received
                # show image
                # pil_image.show()

                # Process the image
                inputs = processor(prompt, pil_image).to("cuda:0", dtype=torch.bfloat16)
                action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

                print(action)  # Print or log the predicted action

        except (ConnectionRefusedError, ConnectionResetError):
            print("[-] Connection failed. Retrying in 5 seconds...")
            client_socket.close()
            torch.cuda.empty_cache()  # Free GPU memory
            time.sleep(5)  # Wait before retrying


if __name__ == "__main__":
    main()
