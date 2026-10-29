from geometry_msgs.msg import Quaternion

import numpy as np
from numpy.typing import NDArray
import math

def rotmat2q(T: NDArray) -> Quaternion:
    if T.shape != (3, 3):
        raise ValueError("Expected a 3x3 rotation matrix")

    tr = np.trace(T)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2          # S = 4*qw
        qw = 0.25 * S
        qx = (T[2, 1] - T[1, 2]) / S
        qy = (T[0, 2] - T[2, 0]) / S
        qz = (T[1, 0] - T[0, 1]) / S
    elif T[0, 0] > T[1, 1] and T[0, 0] > T[2, 2]:
        S = np.sqrt(1.0 + T[0, 0] - T[1, 1] - T[2, 2]) * 2   # S = 4*qx
        qw = (T[2, 1] - T[1, 2]) / S
        qx = 0.25 * S
        qy = (T[0, 1] + T[1, 0]) / S
        qz = (T[0, 2] + T[2, 0]) / S
    elif T[1, 1] > T[2, 2]:
        S = np.sqrt(1.0 + T[1, 1] - T[0, 0] - T[2, 2]) * 2   # S = 4*qy
        qw = (T[0, 2] - T[2, 0]) / S
        qx = (T[0, 1] + T[1, 0]) / S
        qy = 0.25 * S
        qz = (T[1, 2] + T[2, 1]) / S
    else:
        S = np.sqrt(1.0 + T[2, 2] - T[0, 0] - T[1, 1]) * 2   # S = 4*qz
        qw = (T[1, 0] - T[0, 1]) / S
        qx = (T[0, 2] + T[2, 0]) / S
        qy = (T[1, 2] + T[2, 1]) / S
        qz = 0.25 * S

    return Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))



def quat2rodrigues(q: Quaternion) -> NDArray:
    # Function that transforms a ROS quaternion representation to the Rodrigues
    
    # The returned variable should contain a 3 element array in the direction
    # of the rotation axis with magnitude equal to the rotation angle

    # TODO: implement this
    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w

    theta = 2 * np.arccos(np.clip(qw, -1, 1))
    print(f"theta: {theta}")
    s_half_theta = np.sin(theta/2)
    print(f"s_half_theta: {s_half_theta}")
    
    if np.isclose(s_half_theta, 0):
        T = np.zeros(3).reshape(-1, 1)
        return T

    ux = qx / s_half_theta
    uy = qy / s_half_theta
    uz = qz / s_half_theta

    u = np.array([[ux], [uy], [uz]])
    norm_u = np.sqrt(ux**2 + uy**2 + uz**2)
    s = theta / norm_u
    T = u * s

    # raise NotImplementedError
    return T



def quat2zyx(q: Quaternion) -> NDArray:
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







# def rotmat2q(T: NDArray) -> Quaternion:
#     # Function that transforms a 3x3 rotation matrix to a ros quaternion representation
    
#     if T.shape != (3, 3):
#         raise ValueError

#     # theta = math.acos((T[0,0] + T[1,1] + T[2,2] - 1)/2)
#     theta = np.arccos((T[0,0] + T[1,1] + T[2,2] - 1)/2)
#     s_theta = math.sin(theta)
#     u = (1/(2*s_theta)) * np.array([T[2,1] - T[1,2], T[0,2] - T[2,0], T[1,0] - T[0,1]])

#     s_half_theta = math.sin(theta/2)
#     c_half_theta = math.cos(theta/2)

#     qx = u[0] * s_half_theta
#     qy = u[1] * s_half_theta
#     qz = u[2] * s_half_theta
#     qw = c_half_theta

#     q = Quaternion(x=qx, y=qy, z=qz, w=qw)

#     print(q)
#     return q


if __name__ == "__main__":

    # R = np.eye(3)
    # R = np.array([[1,0,0],[0,-1,0],[0,0,-1]])


    # q = rotmat2q(R)
    # q = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)
    # q = Quaternion(x=0.0, y=0.0, z=0.0, w=-1.0)
    # q = Quaternion(x=0.0, y=0.0, z=0.04998, w=0.99875)
    print(quat2zyx(Quaternion(w=np.cos(np.pi/4), x=0.0, y=0.0, z=np.sin(np.pi/4))))

    
    # print(q)
    
    # print(quat2rodrigues(q))

    # rotmat2q(np.array([[0,-1,0],
    #                    [1,0,0],
    #                    [0,0,-1]]))
    # rotmat2q(np.array([[1,0,0],[0,0,-1],[0,1,0]]))

    # print(np.clip(1.0001, -1, 1))
