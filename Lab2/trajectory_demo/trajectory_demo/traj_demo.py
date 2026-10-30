import rclpy
from rclpy.node import Node
import numpy as np
from typing import Optional, Tuple
from numpy.typing import NDArray
import os

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
import matplotlib.pyplot as plt


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import PyKDL
from youbot_kinematics.urdf import treeFromUrdfModel
from youbot_kinematics.urdf_parser import URDF
# from utils import quat2zyx

from ament_index_python.packages import get_package_share_directory

import yaml


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

        # self.declare_parameter("base_link", "base")
        self.declare_parameter("base_link", "fr3_link0")
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

        # TODO: modify to create a datastructure to store joint positions and cartesian positions along the trajectory
        self.joint_pos_all = []
        self.joint_vel_all = []
        self.ee_x_all = []
        self.ee_y_all = []
        self.ee_z_all = []
        self.times_all = []

        self.traj_start_ns = None   
        self.traj_end_ns = None 
        self.traj_planned_duration = None   


    def joint_state_callback(self, msg: JointState):
        """Callback for the joint states of the robot arm. It will get joint positions and velocities, and eventually the cartesian position of the end-effector and save this. It allows initializes the trajectory once the first joint state message comes through.

        Args:
            msg (JointState): ROS Joint State Message.
        """
        for i in range(len(msg.name)):
            t = self.get_clock().now().nanoseconds
            n = msg.name[i]
            pos = msg.position[i]
            vel = msg.velocity[i]

            self.current_joint_position[i] = pos
            self.current_joint_velocity[i] = vel

        joint_pos, joint_vel = self.kdl_to_np(
            self.current_joint_position
        ), self.kdl_to_np(self.current_joint_velocity)

        # TODO: modify to save position and velocities into some array to plot later
        self.joint_pos_all.append(joint_pos)
        self.joint_vel_all.append(joint_vel)
        self.times_all.append(t)


        # TODO: modify to save the position of end effector using get_ee_pos_ros
        ee_pos = self.get_ee_pos_ros()
        if ee_pos:
            self.ee_x_all.append(ee_pos[0])
            self.ee_y_all.append(ee_pos[1])
            self.ee_z_all.append(ee_pos[2])

            if self.traj_start_ns is not None and self.traj_end_ns is None:
                final_wp = self.traj_cfg["waypoints"]["cartesion"][-1]["position"]
                dist = np.linalg.norm(np.array(ee_pos)[:3] - np.array(final_wp))
                if dist < 0.01:
                    self.traj_end_ns = self.get_clock().now().nanoseconds
                    self.get_logger().info(
                        f"Trajectory finished in {(self.traj_end_ns - self.traj_start_ns) * 1e-9:.2f} s"
                    )

            # we do this after we have our first callback to have current joint positions
            if not self.created_traj:
                joint_traj = self.create_traj()
                self.traj_publisher.publish(joint_traj)
                self.traj_start_ns = self.get_clock().now().nanoseconds
                self.created_traj = True

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


    def quat2zyx(self, q: Quaternion) -> NDArray:
        # Function that transforms a ROS quaternion representation to three Euler
        # angles corresponding to the intrinsic Tait-Bryan Z-Y-X representation
        
        # The returned variable should contain a 3 element array of the Euler angles

        # TODO: implement this
        # raise NotImplementedError

        #Normalise the quaternion
        norm_q = np.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        q_normalised = Quaternion(w=q.w/norm_q, x=q.x/norm_q, y=q.y/norm_q, z=q.z/norm_q)

        qn_w = q_normalised.w
        qn_x = q_normalised.x
        qn_y = q_normalised.y
        qn_z = q_normalised.z

        phi = np.arctan2(2*(qn_w*qn_x + qn_y*qn_z), 1 - 2*(qn_x**2 + qn_y**2))
        theta = -np.pi/2 + 2*np.arctan2(np.sqrt(1 + 2*(qn_w*qn_y - qn_x*qn_z)), np.sqrt(1 - 2*(qn_w*qn_y - qn_x*qn_z)))
        psi = np.arctan2(2*(qn_w*qn_z + qn_x*qn_y), 1 - 2*(qn_y**2 + qn_z**2))

        # print(q_normalised)
        # norm_qn = np.sqrt(q_normalised.w**2 + q_normalised.x**2 + q_normalised.y**2 + q_normalised.z**2)
        # print(q_normalised)
        # print(norm_qn)

        T = np.array([phi, theta, psi])
        return T


    def get_ee_pos_ros(self) -> Tuple[float]:
        """FK function that uses TF2 ROS to do forward kinematics and return world coordinates of the end effector.

        Returns:
            Tuple[float]: position, 3 dimensional.
        """
        # TODO: Modify this to get current position of the robot's end effector using the TF2 Library

        try:
           trans = self.tf_buffer.lookup_transform(
            #    "fr_link0",
               self.base_link,
            #    "fr_link8",
               self.ee_link,
               rclpy.time.Time(),
               timeout=rclpy.duration.Duration(seconds=1.0)
            )   
        
        except(LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            return None

        ee_pos_x, ee_pos_y, ee_pos_z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
        ee_rot_qw, ee_rot_qx, ee_rot_qy, ee_rot_qz = trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z
        
        qrot = Quaternion(w=ee_rot_qw, x=ee_rot_qx, y=ee_rot_qy, z=ee_rot_qz)
        rot_arr = self.quat2zyx(q=qrot)
        ee_rot_phi, ee_rot_theta, ee_rot_psi = rot_arr[0], rot_arr[1], rot_arr[2]
                
        return (ee_pos_x, ee_pos_y, ee_pos_z, ee_rot_phi, ee_rot_theta, ee_rot_psi)


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
        # breakpoint()

        cur_joint_pos = self.kdl_to_np(self.current_joint_position)

        goal_positions = []
        goal_times = []

        time_since_start = 0.0

        for waypoint in cartesian_waypoints:
            
            # pos = waypoint["position"]
            pos = waypoint.get("position", None)
            ori = waypoint.get("orientation", None)
                
            time_sec = waypoint["time"] + time_since_start
            print(f"Waypoint pos: {pos}")
            print(f"Waypoint time: {time_sec}")
            assert type(time_sec) == float, type(time)
            assert len(pos) == 3
            goal_times.append(time_sec)
            # time_since_start += time_sec
            time_since_start = time_sec

            goal_joint_pos = None
            
            if ori:
                goal_joint_pos = self.get_joint_pos(cur_joint_pos, pos, ori)
            else:
                goal_joint_pos = self.get_joint_pos(cur_joint_pos, pos)
            
            goal_positions.append(goal_joint_pos)

        # TODO: instantiate a JointTrajectory message, populate it, and publish it via the publisher class member
        # note, populate the header with the relevant frame id and timestamp, in addition to the trajectory

        joint_traj = JointTrajectory()
        joint_traj.header.stamp = self.get_clock().now().to_msg()
        joint_traj.header.frame_id = self.base_link
        joint_traj.joint_names = self.traj_cfg["joint_names"]
        
        for i, gpos in enumerate(goal_positions):
            joint_traj.points.append(JointTrajectoryPoint(positions=gpos, time_from_start=Duration(seconds=float(goal_times[i])).to_msg()))

        self.traj_planned_duration = goal_times[-1]
        print(f"Planned Trajectory Duration: {self.traj_planned_duration}")

        return joint_traj

    def plot_joint_traj(self):
        """Helper function to plot the desired results from following the trajectory. Will be called once the process is interupted via a Keyboard Interupt."""

        # TODO: populate this method to use the datastructure you created for joint positions
        # time_arr = np.array(self.)
        ee_x_arr = np.array(self.ee_x_all)
        ee_y_arr = np.array(self.ee_y_all)
        ee_z_arr = np.array(self.ee_z_all)

        # self.joint_pos_all = []
        # self.joint_vel_all = []

        if self.traj_start_ns is not None and self.traj_end_ns is not None:
            print(f"Measured duration: {(self.traj_end_ns - self.traj_start_ns) * 1e-9:.2f} s")
            print(f"Planned duration: {self.traj_planned_duration:.2f} s")

        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.plot(ee_x_arr, ee_y_arr, ee_z_arr, label="EE path", c='m')

        # overlay the commanded cartesian waypoints 
        wps = np.array([w["position"] for w in self.traj_cfg["waypoints"]["cartesion"]])    
        ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2], c="k", marker="x", s=60, label="waypoints")    
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.set_title("End-effector trajectory")
        ax.legend()

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    traj_demo = TrajDemo()

    try:
        rclpy.spin(traj_demo)
    except KeyboardInterrupt:
        pass

    traj_demo.plot_joint_traj()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traj_demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()
