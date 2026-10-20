from geometry_msgs.msg import Quaternion

import numpy as np
from numpy.typing import NDArray

def rotmat2q(T: NDArray) -> Quaternion:
    # Function that transforms a 3x3 rotation matrix to a ros quaternion representation
    

    if T.shape != (3, 3):
        raise ValueError

    # TODO: implement this
    trace = T[0, 0] + T[1, 1] + T[2, 2]

    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)          # s = 4w
        w = 0.25 * s
        x = (T[2, 1] - T[1, 2]) / s
        y = (T[0, 2] - T[2, 0]) / s
        z = (T[1, 0] - T[0, 1]) / s
    elif T[0, 0] > T[1, 1] and T[0, 0] > T[2, 2]:
        s = 2.0 * np.sqrt(1.0 + T[0, 0] - T[1, 1] - T[2, 2])   # s = 4x
        w = (T[2, 1] - T[1, 2]) / s
        x = 0.25 * s
        y = (T[0, 1] + T[1, 0]) / s
        z = (T[0, 2] + T[2, 0]) / s
    elif T[1, 1] > T[2, 2]:
        s = 2.0 * np.sqrt(1.0 + T[1, 1] - T[0, 0] - T[2, 2])   # s = 4y
        w = (T[0, 2] - T[2, 0]) / s
        x = (T[0, 1] + T[1, 0]) / s
        y = 0.25 * s
        z = (T[1, 2] + T[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + T[2, 2] - T[0, 0] - T[1, 1])   # s = 4z
        w = (T[1, 0] - T[0, 1]) / s
        x = (T[0, 2] + T[2, 0]) / s
        y = (T[1, 2] + T[2, 1]) / s
        z = 0.25 * s

    # normalise to remove small numerical errors
    norm = np.sqrt(w * w + x * x + y * y + z * z)

    q = Quaternion()
    q.x = float(x / norm)
    q.y = float(y / norm)
    q.z = float(z / norm)
    q.w = float(w / norm)

    return q

def quat2rodrigues(q: Quaternion) -> NDArray:
    # Function that transforms a ROS quaternion representation to the Rodrigues
    
    # The returned variable should contain a 3 element array in the direction
    # of the rotation axis with magnitude equal to the rotation angle

    # TODO: implement this
    w, x, y, z = q.w, q.x, q.y, q.z
    if w < 0:
        w, x, y, z = -w, -x, -y, -z

    v = np.array([x, y, z], dtype=float)
    v_norm = np.linalg.norm(v)
    if v_norm < 1e-12:
        return np.zeros(3)

    theta = 2.0 * np.arctan2(v_norm, w)
    axis = v / v_norm
    T = axis * theta
    return T

def quat2zyx(q: Quaternion) -> NDArray:
    # Function that transforms a ROS quaternion representation to three Euler
    # angles corresponding to the intrinsic Tait-Bryan Z-Y-X representation
    # Returned as [angle_z, angle_y, angle_x], i.e. R = Rz(angle_z) Ry(angle_y) Rx(angle_x)

    w, x, y, z = q.w, q.x, q.y, q.z

    norm = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    r11 = 1.0 - 2.0 * (y * y + z * z)   # =  cos(z) cos(y)
    r21 = 2.0 * (w * z + x * y)         # =  sin(z) cos(y)
    r31 = 2.0 * (x * z - w * y)         # = -sin(y)
    r12 = 2.0 * (x * y - w * z)
    r22 = 1.0 - 2.0 * (x * x + z * z)
    r13 = 2.0 * (x * z + w * y)
    r23 = 2.0 * (y * z - w * x)


    cos_y = np.hypot(r11, r21)
    angle_y = np.arctan2(-r31, cos_y)

    # 修复 test 3 / 10:万向锁时偏航和横滚无法区分,按惯例令偏航为 0
    if cos_y < 1e-10: 
        angle_z = 0.0
    else:
        angle_z = np.arctan2(r21, r11)

    cz, sz = np.cos(angle_z), np.sin(angle_z)
    angle_x = np.arctan2(sz * r13 - cz * r23, cz * r22 - sz * r12)

    T = np.array([angle_z, angle_y, angle_x])

    return T


if __name__ == "__main__":
    print(quat2zyx(Quaternion(x=0.0, y=0.0, z=np.sin(np.pi/4),w=np.cos(np.pi/4))))