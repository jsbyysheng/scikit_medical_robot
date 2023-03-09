import numpy as np
import cv2

class handeye_calibration:
    def __init__(self) -> None:
        pass

    @staticmethod
    def calibrate(T_Aurora_arr, T_robot_arr):
        R_world2cam = T_Aurora_arr[:, :3, :3]
        t_world2cam = T_Aurora_arr[:, :3, 3:]
        R_base2gripper = T_robot_arr[:, :3, :3]
        t_base2gripper = T_robot_arr[:, :3, 3:]
        output = cv2.calibrateRobotWorldHandEye(R_base2gripper, t_base2gripper, R_world2cam, t_world2cam, method=cv2.CALIB_HAND_EYE_TSAI)
        T_base2world = np.eye(4)
        T_gripper2cam = np.eye(4)
        T_gripper2cam[:3, :3] = output[0]
        T_gripper2cam[:3, 3:] = output[1]
        T_base2world[:3, :3] = output[2]
        T_base2world[:3, 3:] = output[3]
        return T_gripper2cam, T_base2world
    
    @staticmethod
    def simple_verifivation():
        pass

    @staticmethod
    def evaluate():
        pass