from geometry_msgs.msg import Quaternion

import numpy as np
from numpy.typing import NDArray



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