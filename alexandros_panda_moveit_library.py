import rospy
import moveit_commander
import tf
import moveit_msgs.msg
from math import pi
import actionlib
import franka_gripper.msg
import franka_msgs.msg
from time import sleep
from multimethod import multimethod
import geometry_msgs.msg
import scripts.tf_scripts
from scripts.tf_scripts.tf_transformation_library import transform_pose_stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from scripts.project_constants.project_constants import LOG_ROS_TOPIC, PANDA_LIFT_OBJECTS_JOINTS
from scripts.project_constants import PANDA_HOME_JOINTS
import socket
import json


class FrankaOperator:
    def __init__(self) -> None:
        """
        This class is used to control the Franka Emika Panda robot.
        Be sure to activate ROS node and initialize moveit_commander before using this class.
        """
        self.robot = moveit_commander.RobotCommander()
        group_name = "panda_manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planning_time(10.0)
        print(f"============ End effector link: {self.move_group.get_end_effector_link()}")
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20
        )
        self.panda_recovery = rospy.Publisher(
            "/franka_control/error_recovery/goal", franka_msgs.msg.ErrorRecoveryActionGoal, queue_size=20
        )
        self.movegroup_feedback_subscriber = rospy.Subscriber(
            "/move_group/feedback", moveit_msgs.msg.MoveGroupActionFeedback, self._post_execution_status
        )
        self.event_publisher = rospy.Publisher(LOG_ROS_TOPIC, Header, queue_size=10)

        # Slow down the Panda
        self.move_group.set_max_velocity_scaling_factor(0.06)
        self.move_group.set_max_acceleration_scaling_factor(0.06)
        self.execution_status = None

    def close_gripper(self, goal_force=None) -> None:
        client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        client.wait_for_server()
        goal = franka_gripper.msg.GraspGoal()
        goal.epsilon.inner = 10.0
        goal.epsilon.outer = 10.0
        goal.speed = 0.08
        if goal_force is not None:
            goal.force = goal_force
        # goal.force = 50.0
        client.send_goal(goal)
        client.wait_for_result()

    def open_gripper(self) -> None:
        client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        client.wait_for_server()
        goal = franka_gripper.msg.MoveGoal()
        goal.width = 0.08
        goal.speed = 0.08
        client.send_goal(goal)
        client.wait_for_result()

    def _post_execution_status(self, execution_status) -> None:
        self.execution_status = execution_status.status.text
        if execution_status.status.text == "CONTROL_FAILED":
            self.event_publisher.publish(Header(stamp=rospy.Time.now(), frame_id="Panda Error: Panda Control Error"))
            print("\033[91m" + f"Panda Hit object, Please fix the errors" + "\033[0m")
        elif execution_status.status.text == "TIMED_OUT":
            self.event_publisher.publish(Header(stamp=rospy.Time.now(), frame_id="Panda Error: Panda Timed Out Error"))
            print("\033[91m" + f"Panda Timed Out" + "\033[0m")

    @multimethod
    def move_to_pose(self, target_pose: geometry_msgs.msg.PoseStamped, recursive_i=1) -> bool:
        waypoints = []
        waypoints.append(target_pose.pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        # Move Group Execute Motion Plan of the robot
        self.move_group.set_pose_target(target_pose)
        # my_pub = rospy.Publisher("/robot_pickuppose", geometry_msgs.msg.PoseStamped, queue_size=10)
        # my_pub.publish(target_pose)
        execution_status = self.move_group.go(wait=True)
        # if not execution_status:
        #     print("Execution Failed, pausing further execution")
        #     input("Fix franka errors and press Enter to continue...")
        #     self.move_to_pose(target_pose, recursive_i + 1)
        return execution_status

    @multimethod
    def move_to_pose(self, target_pose: list, recursive_i=1) -> bool:
        self.move_group.set_joint_value_target(target_pose)
        plan = self.move_group.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan[1])
        self.display_trajectory_publisher.publish(display_trajectory)
        # Move Group Execute Motion Plan of the robot
        execution_status = self.move_group.go(wait=True)
        if not execution_status:
            print("Execution Failed, pausing further execution")
            input("Fix franka errors and press Enter to continue...")
            self.move_to_pose(target_pose, recursive_i + 1)
        return execution_status

    def move_object_delta_eef(self, eef_list) -> bool:
        """
        Move the object left by the given length.
        """
        dx, dy, dz, droll, dpitch, dyaw, gripper_open_close = eef_list
        current_pose = self.move_group.get_current_pose()
        current_pose.pose.position.x += dx
        current_pose.pose.position.y += dy
        current_pose.pose.position.z += dz
        # current_pose.pose.orientation = tf.transformations.quaternion_from_euler(droll, dpitch, dyaw, axes="sxyz")
        current_quat = [
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        ]
        delta_quat = tf.transformations.quaternion_from_euler(droll, dpitch, dyaw)
        new_quat = tf.transformations.quaternion_multiply(current_quat, delta_quat)
        current_pose.pose.orientation.x = new_quat[0]
        current_pose.pose.orientation.y = new_quat[1]
        current_pose.pose.orientation.z = new_quat[2]
        current_pose.pose.orientation.w = new_quat[3]
        # if gripper_open_close > 0.7:
        #     self.close_gripper()
        return self.move_to_pose(current_pose)
    
    def start_socket_server(self):
        """
        Starts a socket server that listens on localhost:9999 for a list of joint positions.
        The received list is then used to move the robot to the specified positions.
        """
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(("0.0.0.0", 9999))
        server_socket.listen(1)
        print("Waiting for connection on port 9999...")

        while True:
            conn, addr = server_socket.accept()
            print(f"Connected by {addr}")
            while True:
                try:
                    msg_length_bytes = conn.recv(4)
                    if not msg_length_bytes:
                        continue
                    msg_length = int.from_bytes(msg_length_bytes, "big")

                    # Receive the actual JSON data
                    pose_data = conn.recv(msg_length).decode("utf-8")

                    # Convert JSON string back to a list
                    eef_positions = json.loads(pose_data)
                    print(f"Received data: {eef_positions}")
                    if isinstance(eef_positions, list) and len(eef_positions) == 7:
                        print(f"Received joint positions: {eef_positions}")
                        self.move_object_delta_eef(eef_positions)
                    else:
                        print("Invalid data received. Expected a list of 7 joint values.")

                except Exception as e:
                    print(f"Error receiving data: {e}")

                finally:
                    conn.close()
                    print("Connection closed.")


if __name__ == "__main__":
    rospy.init_node("franka_operator", anonymous=True)
    operator = FrankaOperator()
    operator.move_to_pose(PANDA_HOME_JOINTS)
    operator.open_gripper()
    operator.start_socket_server()
    
