import rclpy
from rclpy.node import Node
import numpy as np
from typing import Optional, Tuple
from numpy.typing import NDArray
import os

from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import matplotlib.pyplot as plt


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import PyKDL
from youbot_kinematics.urdf import treeFromUrdfModel
from youbot_kinematics.urdf_parser import URDF

from ament_index_python.packages import get_package_share_directory

import yaml


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert a quaternion to (roll, pitch, yaw) using the same convention as
    PyKDL.Rotation.RPY, i.e. R = Rz(yaw) Ry(pitch) Rx(roll).

    This is the intrinsic Z-Y-X conversion from Lab 1 (quat2zyx) with the
    output order reversed.
    """
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class TrajDemo(Node):
    def __init__(self):
        super().__init__("traj_demo")

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 5
        )
        self.joint_state_sub  # prevent unused variable warning
        self.traj_publisher = self.create_publisher(
            JointTrajectory, "/franka_arm_controller/joint_trajectory", 5
        )
        self.traj_publisher

        # load from urdf file
        self.declare_parameter("urdf_package", "franka_description")
        self.urdf_package = (
            self.get_parameter("urdf_package").get_parameter_value().string_value
        )
        self.urdf_package_path = get_package_share_directory(self.urdf_package)
        self.declare_parameter("urdf_path_in_package", "urdfs/fr3.urdf")
        self.urdf_path_in_package = (
            self.get_parameter("urdf_path_in_package")
            .get_parameter_value()
            .string_value
        )
        self.urdf_name_path = os.path.join(
            self.urdf_package_path, self.urdf_path_in_package
        )

        self.get_logger().info(
            f"loading robot into KDL from urdf: {self.urdf_name_path}"
        )

        robot = URDF.from_xml_file(self.urdf_name_path)

        (ok, self.kine_tree) = treeFromUrdfModel(robot)

        if not ok:
            raise RuntimeError("couldn't load URDF into KDL tree succesfully")

        self.declare_parameter("base_link", "base")
        self.declare_parameter("ee_link", "fr3_link8")
        self.base_link = (
            self.get_parameter("base_link").get_parameter_value().string_value
        )
        self.ee_link = self.get_parameter("ee_link").get_parameter_value().string_value

        self.kine_chain = self.kine_tree.getChain(self.base_link, self.ee_link)
        self.NJoints = self.kine_chain.getNrOfJoints()
        self.current_joint_position = PyKDL.JntArray(self.NJoints)
        self.current_joint_velocity = PyKDL.JntArray(self.NJoints)
        # KDL solvers
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.kine_chain)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # loading trajectory structure

        self.declare_parameter("traj_cfg_pkg", "trajectory_demo")
        self.traj_cfg_pkg = (
            self.get_parameter("traj_cfg_pkg").get_parameter_value().string_value
        )
        self.traj_cfg_pkg_path = get_package_share_directory(self.traj_cfg_pkg)
        self.declare_parameter("traj_cfg_path_within_pkg", "cfg/traj_waypoints.yaml")
        self.traj_cfg_path_within_pkg = (
            self.get_parameter("traj_cfg_path_within_pkg")
            .get_parameter_value()
            .string_value
        )

        self.traj_cfg_path = os.path.join(
            self.traj_cfg_pkg_path, self.traj_cfg_path_within_pkg
        )

        with open(self.traj_cfg_path) as stream:
            self.traj_cfg = yaml.safe_load(stream)

        assert type(self.traj_cfg["joint_names"]) == list, type(
            self.traj_cfg["joint_names"]
        )
        assert len(self.traj_cfg["joint_names"]) == self.NJoints

        self.created_traj = False

        self.get_logger().info(f"got traj cfg:\n{self.traj_cfg}")


        self.log = {
            "t": [],         
            "q": [],         
            "qd": [],         
            "ee_t": [],      
            "ee_pos": [],    
            "ee_rpy": [],    
        }
        self.waypoint_positions = []    
        self.waypoint_orientations = []  

        self.traj_start_time = None
        self.traj_planned_duration = 0.0
        self.traj_measured_duration = None
        self.declare_parameter("goal_tolerance", 0.002)  # [m]
        self.goal_tolerance = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def joint_state_callback(self, msg: JointState):
        """Callback for the joint states of the robot arm. It will get joint positions and velocities, and eventually the cartesian position of the end-effector and save this. It allows initializes the trajectory once the first joint state message comes through.

        Args:
            msg (JointState): ROS Joint State Message.
        """
       
        joint_names = self.traj_cfg["joint_names"]
        for i in range(len(msg.name)):
            n = msg.name[i]
            if n not in joint_names:
                continue
            k = joint_names.index(n)
            self.current_joint_position[k] = msg.position[i]
            if len(msg.velocity) > i:
                self.current_joint_velocity[k] = msg.velocity[i]

        joint_pos, joint_vel = self.kdl_to_np(
            self.current_joint_position
        ), self.kdl_to_np(self.current_joint_velocity)

        # we do this after we have our first callback to have current joint positions
        if not self.created_traj:
            joint_traj = self.create_traj()
            self.traj_publisher.publish(joint_traj)
            self.created_traj = True
            self.traj_start_time = self.now_sec()
            self.get_logger().info(
                f"published trajectory with {len(joint_traj.points)} points, "
                f"planned duration {self.traj_planned_duration:.2f} s"
            )

        t = self.now_sec() - self.traj_start_time

        
        self.log["t"].append(t)
        self.log["q"].append(joint_pos)
        self.log["qd"].append(joint_vel)

        # save the end-effector pose using TF2
        ee = self.get_ee_pos_ros()
        if ee is not None:
            pos, rpy = ee
            self.log["ee_t"].append(t)
            self.log["ee_pos"].append(pos)
            self.log["ee_rpy"].append(rpy)

            
            if self.traj_measured_duration is None and self.waypoint_positions:
                err = np.linalg.norm(np.array(pos) - np.array(self.waypoint_positions[-1]))
                still = np.max(np.abs(joint_vel)) < 1e-3
                if t >= self.traj_planned_duration and err < self.goal_tolerance and still:
                    self.traj_measured_duration = t
                    self.get_logger().info(
                        f"trajectory finished: measured {t:.2f} s "
                        f"(planned {self.traj_planned_duration:.2f} s), "
                        f"final position error {1000 * err:.2f} mm"
                    )

    def kdl_to_np(self, data: PyKDL.JntArray) -> NDArray:
        """Helper Function to go from KDL arrays to numpy arrays

        Args:
            data (PyKDL.JntArray): desired KDL array to convert

        Returns:
            NDArray: converted NP Array
        """
        is_1d = data.columns() == 1
        np_shape = (data.rows(), data.columns()) if not is_1d else (data.rows(),)
        mat = np.zeros(np_shape)
        for i in range(data.rows()):
            if not is_1d:
                for j in range(data.columns()):
                    mat[i, j] = data[i, j]
            else:
                mat[i] = data[i]
        return mat

    def get_ee_pos_ros(self):
        """FK function that uses TF2 ROS to do forward kinematics and return world coordinates of the end effector.

        Returns:
            (position, rpy) of the end effector in the base frame, or None if the
            transform is not available yet.
        """
        try:
            # latest available transform from the base frame to the end-effector frame
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().debug(
                f"could not transform {self.ee_link} to {self.base_link}: {ex}"
            )
            return None

        tr = tf.transform.translation
        rot = tf.transform.rotation
        pos = [tr.x, tr.y, tr.z]
        rpy = list(quat_to_rpy(rot.x, rot.y, rot.z, rot.w))
        return pos, rpy

    def get_joint_pos(
        self,
        current_angles: Tuple[float],
        target_position: Tuple[float],
        target_orientation: Optional[Tuple[float]] = None,
    ) -> Tuple[float]:
        """Helper function to solve inverse kinematics using the KDL library

        Args:
            current_angles (Tuple[float]): current joint angles of the robot arm
            target_position (Tuple[float]): target cartesian position, 3 dimensional, of the end effector in world coordinates
            target_orientation (Optional[Tuple[float]], optional): target orientation in RPY format, optional. Defaults to None.

        Raises:
            RuntimeError: raises error if IK fails to solve. Check the workspace of the robot if this happens.

        Returns:
            Tuple[float]: joint angles that were found to satisfy targets
        """

        assert len(target_position) == 3
        assert target_orientation is None or len(target_orientation) == 3
        assert len(current_angles) == self.NJoints

        pos = PyKDL.Vector(target_position[0], target_position[1], target_position[2])
        if target_orientation is not None:
            # Constructs a rotation by first applying a rotation of r around the x-axis, then a rotation of p around the original y-axis, and finally a rotation of y around the original z-axis
            rot = PyKDL.Rotation.RPY(
                target_orientation[0], target_orientation[1], target_orientation[2]
            )

        seed_array = PyKDL.JntArray(self.NJoints)
        for i in range(self.NJoints):
            seed_array[i] = current_angles[i]

        if target_orientation is not None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self.NJoints)

        if self.ik_solver.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = list(result_angles)
            return result
        else:
            raise RuntimeError(
                f"Did not solve for goal_pose: {goal_pose} with initial seed {seed_array}"
            )

    def create_traj(self) -> JointTrajectory:
        """Helper function to generate the trajectory to send to the arm.  Loops over cartesian waypoints from the config file, processing them into joint positions using the IK Solver

        Returns:
            JointTrajectory: Ros JointTrajectory to publish
        """
        cartesian_waypoints = self.traj_cfg["waypoints"]["cartesion"]

        cur_joint_pos = self.kdl_to_np(self.current_joint_position)

        goal_positions = []
        goal_times = []

        time_since_start = 0.0

        for waypoint in cartesian_waypoints:
            pos = waypoint["position"]
            # "time" is the duration of this segment; add it to the running total.
            # (The original code added the running total twice, so the
            # timestamps grew much faster than intended.)
            segment_time = float(waypoint["time"])
            assert len(pos) == 3
            time_since_start += segment_time
            goal_times.append(time_since_start)

            # optional orientation target [roll, pitch, yaw] in radians (step 3)
            rpy = waypoint.get("orientation", None)

            # seed the IK with the previous solution so consecutive waypoints stay
            # on the same IK branch (smooth motion, see Q2)
            cur_joint_pos = self.get_joint_pos(cur_joint_pos, pos, rpy)
            goal_positions.append(cur_joint_pos)

            self.waypoint_positions.append(list(pos))
            self.waypoint_orientations.append(rpy)

        self.traj_planned_duration = time_since_start

        # instantiate the JointTrajectory message and populate header and points
        joint_traj = JointTrajectory()
        joint_traj.header.stamp = self.get_clock().now().to_msg()
        joint_traj.header.frame_id = self.base_link
        joint_traj.joint_names = list(self.traj_cfg["joint_names"])

        for q, t in zip(goal_positions, goal_times):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q]
            point.time_from_start = Duration(seconds=t).to_msg()
            joint_traj.points.append(point)

        # the message is published by joint_state_callback (avoids publishing twice)
        return joint_traj

    def plot_joint_traj(self):
        """Helper function to plot the desired results from following the trajectory. Will be called once the process is interupted via a Keyboard Interupt."""
        if len(self.log["ee_pos"]) == 0:
            # print instead of the ROS logger: rclpy may already be shut down after ctrl+c
            print("no end-effector data recorded, nothing to plot")
            return

        ee = np.array(self.log["ee_pos"])
        ee_t = np.array(self.log["ee_t"])
        wp = np.array(self.waypoint_positions)

        # Figure 1: cartesian path of the end effector in 3D
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], label="end-effector path")
        ax.scatter(wp[:, 0], wp[:, 1], wp[:, 2], color="red", marker="x",
                   label="waypoints")
        ax.scatter(*ee[0], color="green", label="start")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        title = "End-effector trajectory"
        if self.traj_measured_duration is not None:
            title += f" (time to goal: {self.traj_measured_duration:.2f} s)"
        ax.set_title(title)
        ax.legend()
        fig.savefig("ee_trajectory_3d.png", dpi=200)

        # Figure 2: end-effector position against time
        fig2, ax2 = plt.subplots()
        for k, name in enumerate(["x", "y", "z"]):
            ax2.plot(ee_t, ee[:, k], label=name)
        ax2.set_xlabel("time [s]")
        ax2.set_ylabel("position [m]")
        ax2.set_title("End-effector position")
        ax2.legend()
        ax2.grid(True)
        fig2.savefig("ee_position.png", dpi=200)

        # Figure 3: end-effector orientation against time (step 3)
        rpy = np.unwrap(np.array(self.log["ee_rpy"]), axis=0)
        fig3, ax3 = plt.subplots()
        for k, name in enumerate(["roll", "pitch", "yaw"]):
            ax3.plot(ee_t, rpy[:, k], label=name)
        ax3.set_xlabel("time [s]")
        ax3.set_ylabel("angle [rad]")
        ax3.set_title("End-effector orientation (RPY)")
        ax3.legend()
        ax3.grid(True)
        fig3.savefig("ee_orientation.png", dpi=200)

        # Figure 4: joint positions against time
        q = np.array(self.log["q"])
        fig4, ax4 = plt.subplots()
        for k in range(q.shape[1]):
            ax4.plot(self.log["t"], q[:, k], label=self.traj_cfg["joint_names"][k])
        ax4.set_xlabel("time [s]")
        ax4.set_ylabel("joint angle [rad]")
        ax4.set_title("Joint trajectories")
        ax4.legend(fontsize="small")
        ax4.grid(True)
        fig4.savefig("joint_positions.png", dpi=200)

        if self.traj_measured_duration is not None:
            print(f"time to reach the final waypoint: {self.traj_measured_duration:.2f} s "
                  f"(planned {self.traj_planned_duration:.2f} s)")
        print("saved ee_trajectory_3d.png, ee_position.png, ee_orientation.png, "
              "joint_positions.png in the current directory")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    traj_demo = TrajDemo()

    try:
        rclpy.spin(traj_demo)
    except KeyboardInterrupt:
        pass

    traj_demo.plot_joint_traj()
    traj_demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()