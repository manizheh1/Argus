import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import json
import time
from time import sleep
import numpy as np
import torch


class RDLS_Saver:
    def __init__(self):
        # Start ros subscriber
        self.episodes = []
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.zed_rgb_image_callback)
        rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.zed_depth_image_callback)
        self.rgb_data = None
        self.depth_data = None
        self.joint_states = None
        self.tf = TransformListener()
        self.loop()
        


    def joint_states_callback(self, data):
        # print("sup")
        self.joint_states = data.position

    def zed_rgb_image_callback(self, data):
        self.rgb_data = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1).tolist()
        # convert to numpy array
    
    def zed_depth_image_callback(self, data):
        self.depth_data = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1).tolist()
    
    def loop(self):
        start_time = time.time()
        while time.time() - start_time < 50.0:
            print("Recording data...")
            episode = {
                "observations": [],
                "actions": [],
                "rewards": [],
                "success": [],
                "task": "mug_pick_and_place",
                "robot": "franka_panda",
                "camera": "zed_2i",
                "input": "Pickup the mug",
                "openVLA_compatible": True,
            }
            if self.rgb_data is not None and self.depth_data is not None and self.joint_states is not None:
                episode["observations"] = {
                    "rgb": self.rgb_data,
                    "depth": self.depth_data,
                    "joint_states": self.joint_states,
                }
                # t = self.tf.getLatestCommonTime("/panda_link0", "/panda_hand")
                position, quaternion = self.tf.lookupTransform("panda_link8", "panda_link0",  rospy.Time(0))
                # Change this quaternion to euler angles
                euler = euler_from_quaternion(quaternion)
                position.extend(euler)
                if time.time() - start_time > 10.0:
                    position.extend([0.0])
                else:
                    position.extend([1.0])
                episode["actions"] = position
                episode["rewards"] = 0
                episode["success"] = False
                self.episodes.append(episode)
            sleep(0.1)


def main():
    rospy.init_node("rdls_saver")
    rdls_saver = RDLS_Saver()
    all_episodes = rdls_saver.episodes
    all_episodes[-1]["success"] = True
    all_episodes[-1]["rewards"] = 1.0
    torch.save({"episodes": all_episodes}, "panda_data_rlds.pth")
    # with open("panda_data_rlds.json", "w") as f:
    #     json.dump({"episodes": all_episodes}, f)
    rospy.loginfo("Cleanup complete.")

if __name__ == '__main__':
    main()
