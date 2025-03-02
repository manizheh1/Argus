from time import sleep
import socket
import struct
from PIL import Image
import io
import torch
from PIL import Image
from transformers import AutoProcessor
from transformers import AutoModelForVision2Seq


def receive_image():
    host = "0.0.0.0"
    port = 5001
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"[*] Listening on {host}:{port}...")
    conn, addr = server_socket.accept()
    print(f"[+] Connection from {addr}")

    # Receive the image size first (4 bytes)
    data = conn.recv(4)
    if not data:
        print("[-] No data received.")
        return

    image_size = struct.unpack(">I", data)[0]  # Read image size (big-endian)

    # Receive the image bytes
    image_data = b""
    while len(image_data) < image_size:
        packet = conn.recv(4096)  # Receive in chunks
        if not packet:
            break
        image_data += packet

    # Convert bytes to a PIL image
    image = Image.open(io.BytesIO(image_data))
    return image


def main():

    processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
    vla = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b",
        attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
        torch_dtype=torch.bfloat16,
        low_cpu_mem_usage=True,
        trust_remote_code=True,
    ).to("cuda:0")
    prompt = "In: What action should the robot take to pick up the cardboard box?\nOut:"
    # Predict Action (7-DoF; un-normalize for BridgeData V2)
    while 1:
        pil_image = receive_image()
        inputs = processor(prompt, pil_image).to("cuda:0", dtype=torch.bfloat16)
        action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)
        print(action)


if __name__ == "__main__":
    main()
