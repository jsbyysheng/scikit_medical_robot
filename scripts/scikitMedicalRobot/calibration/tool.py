import numpy as np
import pickle
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit, least_squares
from .sample_helper import jupyter_quick_sample
from ..utilities import generate_timedate_cache_file

class trajectory_recorder:
    def __init__(self, robot_joints_sample_func, name='trajectory_recorder', caches_dir='exp', child_dir='trajectory_recorder') -> None:
        self.robot_joints_sample_func = robot_joints_sample_func
        self.name = name
        self.caches_dir = caches_dir
        self.child_dir = child_dir

        self.sampler = jupyter_quick_sample(
            name, 
            self.sample, 
            self.evaluate, 
            self.clear_samples, 
            self.save_samples
        )
        
    def sample(self, _):
        jnts = self.robot_joints_sample_func()
        if jnts is not None:
            print(f'joint_state rad: \n{jnts}')
            print(f'joint_state deg: \n{list(np.array(jnts) * 180 / np.pi)}')
        return jnts

    def evaluate(self, datas):
        pass

    def clear_samples(self, datas):
        pass

    def save_samples(self, datas, name):
        f_name = generate_timedate_cache_file(caches_dir=self.caches_dir, child_dir=self.child_dir, filetype='pickle', prefix=self.name)
        with open(f_name, "wb") as f:
            pickle.dump(np.array(datas), f)
        print(f"saved to {f_name}")

    def run(self):
        return self.sampler.run_sample()

class ftsensor_calibration_recorder:
    def __init__(self, ftsensor_sample_func, trans_sample_func, name='ftsensor_recorder', caches_dir='exp', child_dir='ftsensor_recorder') -> None:
        self.ftsensor_sample_func = ftsensor_sample_func
        self.trans_sample_func = trans_sample_func

        self.name = name
        self.caches_dir = caches_dir
        self.child_dir = child_dir

        self.sampler = jupyter_quick_sample(
            name, 
            self.sample, 
            self.evaluate, 
            self.clear_samples, 
            self.save_samples
        )

    def sample(self, _):
        trans = self.trans_sample_func()
        ft = self.ftsensor_sample_func()
        if trans is not None and ft is not None:
            print(f"trans: \n{trans}")
            print(f'ft: \n{ft}')
        return trans, ft

    def evaluate(self, datas):
        if len(datas) >= 3:
            trans_list = []
            ft_list = []
            for d in datas:
                trans_list.append(d[0])
                ft_list.append(d[1])
            ret = ftsensor_calibration.calibration(ft_list, trans_list)
            print(ret)
            print(ftsensor_calibration.evaluate(ft_list, trans_list, ret, no_plot=True))
        return None

    def clear_samples(self, datas):
        pass

    def save_samples(self, datas, name):
        f_name = generate_timedate_cache_file(caches_dir=self.caches_dir, child_dir=self.child_dir, filetype='pickle', prefix=self.name)
        trans_list = []
        ft_list = []
        for d in datas:
            trans_list.append(d[0])
            ft_list.append(d[1])

        with open(f_name, "wb") as f:
            pickle.dump([np.array(trans_list), np.array(ft_list)], f)
        print(f"saved to {f_name}")

    def run(self):
        return self.sampler.run_sample()

class ftsensor_calibration:
    def __init__(self) -> None:
        pass

    @staticmethod
    def readme():
        text = '''1. 在力传感器末端安装好待标定工具；\n2. 控制机械臂运行到不同的位姿，记录下机器人的姿态以及力传感器数值；\n3. 标定完成后，一次即可得到当前工具在传感器坐标系下的质心位置以及重量、传感器的零飘。'''
        print(text)

    @staticmethod
    def calibration(torque_exp_data, trans_exp_data, print_info=True):
        torque_exp_data = np.array(torque_exp_data)
        trans_exp_data = np.array(trans_exp_data)
        (n_exp, DoF_ft) = torque_exp_data.shape
        A = np.zeros((3 * n_exp, 6))
        b = np.zeros((3 * n_exp, 1))
        for n in range(0, n_exp):
            A[3 * n, 1] = torque_exp_data[n, 2]
            A[3 * n, 2] = -torque_exp_data[n, 1]
            A[3 * n, 3] = 1
            A[3 * n + 1, 0] = -torque_exp_data[n, 2]
            A[3 * n + 1, 2] = torque_exp_data[n, 0]
            A[3 * n + 1, 4] = 1
            A[3 * n + 2, 0] = torque_exp_data[n, 1]
            A[3 * n + 2, 1] = -torque_exp_data[n, 0]
            A[3 * n + 2, 5] = 1
            b[3 * n] = torque_exp_data[n, 3]
            b[3 * n + 1] = torque_exp_data[n, 4]
            b[3 * n + 2] = torque_exp_data[n, 5]
        x = np.linalg.inv(A.T.dot(A)).dot(A.T).dot(b)
    
        Lx = x[0, 0]
        Ly = x[1, 0]
        Lz = x[2, 0]
        FTa = x[3, 0]
        FTb = x[4, 0]
        FTc = x[5, 0]
    
        (n_exp, _, _) = trans_exp_data.shape
        A = np.zeros((3 * n_exp, 6))
        b = np.zeros((3 * n_exp, 1))
        for n in range(0, n_exp):
            A[3 * n:3 * n + 3, 0:3] = trans_exp_data[n, 0:3, 0:3].T
            A[3 * n, 3] = 1
            A[3 * n + 1, 4] = 1
            A[3 * n + 2, 5] = 1

            b[3 * n] = torque_exp_data[n, 0]
            b[3 * n + 1] = torque_exp_data[n, 1]
            b[3 * n + 2] = torque_exp_data[n, 2]

        x = np.linalg.inv(A.T.dot(A)).dot(A.T).dot(b)

        Sx = x[0, 0]
        Sy = x[1, 0]
        Sz = x[2, 0]
        Fx0 = x[3, 0]
        Fy0 = x[4, 0]
        Fz0 = x[5, 0]

        Mx0 = FTa - Fy0 * Lz + Fz0 * Ly
        My0 = FTb - Fz0 * Lx + Fx0 * Lz
        Mz0 = FTc - Fx0 * Ly + Fy0 * Lx
        if print_info:
            print('Lx {:.4f}, Ly {:.4f}, Lz {:.4f}, FTa {:.4f}, FTb {:.4f}, FTc {:.4f}'.format(x[0, 0], x[1, 0], x[2, 0], x[3, 0], x[4, 0], x[5, 0]))
            print('Sx {:.8f}, Sy {:.8f}, Sz {:.8f}, Fx0 {:.4f}, Fy0 {:.4f}, Fz0 {:.4f}'.format(x[0, 0], x[1, 0], x[2,   0], x[3, 0], x[4, 0], x[5, 0]))
            print('Mx0 {:.4f}, My0 {:.4f}, Mz0 {:.4f}'.format(Mx0, My0, Mz0))

        return {'origin': [Lx, Ly, Lz], 'zero_offset': [Fx0, Fy0, Fz0, Mx0, My0, Mz0], 'gravity': [Sx, Sy, Sz]}

    '''
    The force from the sensor is compensated by the tool and offset.
    The returned force is in tool link.
    '''
    @staticmethod
    def apply_compensation(torque_data, trans_data, calibration):
        gravity = np.array(calibration['gravity']).reshape(3, 1)
        Fx0 = calibration['zero_offset'][0]
        Fy0 = calibration['zero_offset'][1]
        Fz0 = calibration['zero_offset'][2]
        Mx0 = calibration['zero_offset'][3]
        My0 = calibration['zero_offset'][4]
        Mz0 = calibration['zero_offset'][5]
        x = calibration['origin'][0]
        y = calibration['origin'][1]
        z = calibration['origin'][2]
        Lr = np.array([x, y, z])
        R = trans_data[0:3, 0:3]
        Fg = R.T.dot(gravity).flatten()
        Mg = np.cross(Lr, Fg)
        Fgx = Fg[0]
        Fgy = Fg[1]
        Fgz = Fg[2]
        Mgx = Mg[0]
        Mgy = Mg[1]
        Mgz = Mg[2]
        Fx = torque_data[0] - Fx0 - Fgx
        Fy = torque_data[1] - Fy0 - Fgy
        Fz = torque_data[2] - Fz0 - Fgz
        Tx = torque_data[3] - Mx0 - Mgx
        Ty = torque_data[4] - My0 - Mgy
        Tz = torque_data[5] - Mz0 - Mgz
        return [Fx, Fy, Fz, Tx, Ty, Tz]

    @staticmethod
    def evaluate(torque_data, trans_data, calibration, no_plot=True):
        torque_data = np.array(torque_data)
        trans_data = np.array(trans_data)
        assert len(torque_data) == len(trans_data)
        gravity = np.array(calibration['gravity']).reshape(3, 1)
        Fx0 = calibration['zero_offset'][0]
        Fy0 = calibration['zero_offset'][1]
        Fz0 = calibration['zero_offset'][2]
        Mx0 = calibration['zero_offset'][3]
        My0 = calibration['zero_offset'][4]
        Mz0 = calibration['zero_offset'][5]
        x = calibration['origin'][0]
        y = calibration['origin'][1]
        z = calibration['origin'][2]
        Lr = np.array([x, y, z])
        residual_error = np.zeros((len(torque_data), 6))
        for idx in range(len(torque_data)):
            R = trans_data[idx][0:3, 0:3]
            Fg = R.T.dot(gravity).flatten()
            Mg = np.cross(Lr, Fg)
            Fgx = Fg[0]
            Fgy = Fg[1]
            Fgz = Fg[2]
            Mgx = Mg[0]
            Mgy = Mg[1]
            Mgz = Mg[2]
            # Mgx = Fgz * y - Fgy * z
            # Mgy = Fgx * z - Fgz * x
            # Mgz = Fgy * x - Fgx * y
            residual_error[idx, 0] = torque_data[idx, 0] - Fx0 - Fgx
            residual_error[idx, 1] = torque_data[idx, 1] - Fy0 - Fgy
            residual_error[idx, 2] = torque_data[idx, 2] - Fz0 - Fgz
            residual_error[idx, 3] = torque_data[idx, 3] - Mx0 - Mgx
            residual_error[idx, 4] = torque_data[idx, 4] - My0 - Mgy
            residual_error[idx, 5] = torque_data[idx, 5] - Mz0 - Mgz
        
        if not no_plot:
            fig, axes = plt.subplots(2, 1)
            axes[0].plot(residual_error[:, 0])
            axes[0].plot(residual_error[:, 1])
            axes[0].plot(residual_error[:, 2])
            axes[1].plot(residual_error[:, 3])
            axes[1].plot(residual_error[:, 4])
            axes[1].plot(residual_error[:, 5])
            plt.show()

        text = f'''residual_error:
        mean: fx: {np.mean(residual_error[:, 0])} fy: {np.mean(residual_error[:, 1])} fz: {np.mean(residual_error[:, 2])}
              tx: {np.mean(residual_error[:, 3])} ty: {np.mean(residual_error[:, 4])} tz: {np.mean(residual_error[:, 5])}
        std:  fx: {np.std(residual_error[:, 0])} fy: {np.std(residual_error[:, 1])} fz: {np.std(residual_error[:, 2])}
              tx: {np.std(residual_error[:, 3])} ty: {np.std(residual_error[:, 4])} tz: {np.std(residual_error[:, 5])}
        max:  fx: {np.max(residual_error[:, 0])} fy: {np.max(residual_error[:, 1])} fz: {np.max(residual_error[:, 2])}
              tx: {np.max(residual_error[:, 3])} ty: {np.max(residual_error[:, 4])} tz: {np.max(residual_error[:, 5])}'''
        return text

class tcp_calibration:
    def __init__(self) -> None:
        pass

    @staticmethod
    def readme():
        text = '''1. 控制机械臂移动工具从不同方位触碰空间中某个固定点，记录N组数据（n ⩾ 3）；\n2. 计算获得工具末端点相对机械臂末端点的位置变换'''
        print(text)

    @staticmethod
    def tip2tip(transforms, pinv=False):
        translations = []
        rotations = []
        for t in transforms:
            translations.append(np.array([[t[0, 3]],[t[1, 3]],[t[2, 3]]]))
            rotations.append(t[0:3, 0:3])

        trans_data = []
        rotms_data = []
        for i in range(len(transforms) - 1):
            trans_data.append(translations[i + 1] - translations[i])
            rotms_data.append(rotations[i] - rotations[i + 1])

        L = np.array(np.zeros((3, 3)))
        R = np.array(np.zeros((3, 1)))
        for i in range(len(trans_data)):
            L = L + np.dot(rotms_data[i], rotms_data[i])
            R = R + np.dot(rotms_data[i], trans_data[i])

        if pinv:
            return np.linalg.pinv(L).dot(R)
        else:
            return np.linalg.inv(L).dot(R)
        
    @staticmethod
    def tip2tip_optim(transforms, tcp_cali_x0=None):
        if tcp_cali_x0 is None:
            tcp_cali_x0 = tcp_calibration.tip2tip(transforms)
        center, _, _ = tcp_calibration.fit_data_sphere(transforms)
        def fun(calibration_result, transforms, center):
            err, _ = tcp_calibration.evaluate1(calibration_result, transforms, center)
            return err
        solution = least_squares(fun, x0=tcp_cali_x0.flatten(), args=(transforms, center), method='lm')
        return solution.x.reshape(3, 1), solution

    @staticmethod
    def fit_data_sphere(transforms):
        xdata = transforms[:, 0:3, 3] 
        ydata = np.zeros(len(xdata))
    
        def func(data, p0, p1, p2, r):
            return np.power((data[:, 0] - p0), 2) + np.power((data[:, 1] - p1), 2) + np.power((data[:, 2] - p2), 2) - np.power(r, 2)

        popt, pcov = curve_fit(func, xdata, ydata)
        center = popt[0:3]
        r = popt[3]
        
        return center, r, pcov

    @staticmethod
    def evaluate1(calibration_result, transforms, center=None):
        # input / output unit: meter
        # calibration_result: 3 x 1 matrix
        if center is None:
            center, _, _ = tcp_calibration.fit_data_sphere(transforms)

        cali_trans = np.ones((4, 1))
        cali_trans[0:3, :] = calibration_result.reshape(3, 1)
        reprojection = []
        for t in transforms:
            reprojection.append(t.dot(cali_trans))
        err = np.array([0.0, 0.0, 0.0])
        for idx in range(len(reprojection)):
            x0 = reprojection[idx][0, 0]
            y0 = reprojection[idx][1, 0]
            z0 = reprojection[idx][2, 0]
            err[0] = err[0] + np.abs(x0 - center[0])
            err[1] = err[1] + np.abs(y0 - center[1])
            err[2] = err[2] + np.abs(z0 - center[2])
        err = err / len(reprojection)
        return err, reprojection

    @staticmethod
    def evaluate2(calibration_result, transforms):
        # input / output unit: meter
        # calibration_result: 3 x 1 matrix
        cali_trans = np.ones((4, 1))
        cali_trans[0:3, :] = calibration_result

        reprojection = []
        for t in transforms:
            reprojection.append(t.dot(cali_trans))
        err = 0

        for idx in range(len(reprojection) - 1):
            x0 = reprojection[idx][0, 0]
            y0 = reprojection[idx][1, 0]
            z0 = reprojection[idx][2, 0]
            x1 = reprojection[idx + 1][0, 0]
            y1 = reprojection[idx + 1][1, 0]
            z1 = reprojection[idx + 1][2, 0]
            err = err + np.linalg.norm(np.array([x0, y0, z0]) - np.array([x1, y1, z1]))
        err = err / (len(reprojection) - 1)
        return err, reprojection

class tcf_calibration:
    def __init__(self) -> None:
        pass

    def readme(self):
        text = '''1. 完成位置标定；\n2. 控制工具末端点分别沿x方向和z方向移动一定距离，工具末端点只在该方向上有移动，其它方向上无位移，同时固定初始姿态保持不变。实际操作上可以设置三个固定点（三个固定点满足上述要求，点2和点3相对点1只有一个方向上的移动），使工具末端点分别触碰这三个点然后记录下机械臂末端位姿；\n3. 计算获得工具坐标系相对机械臂末端坐标系的姿态变换。'''
        print(text)

    def cal_rotm(self, transforms, t):
        # center
        P_otcp_To_B = transforms[0][:3, :3] @ t + transforms[0][:3, 3:]

        # cal the dircction vector of x
        P_xtcp_To_B = transforms[1][:3, :3] @ t + transforms[1][:3, 3:]
        vector_X = P_xtcp_To_B - P_otcp_To_B
        dire_vec_x_o = np.linalg.inv(transforms[1][:3, :3]) @ vector_X / np.linalg.norm(vector_X)

        # cal the dircction vector of z
        P_ztcp_To_B = transforms[2][:3, :3] @ t + transforms[1][:3, 3:]
        vector_Z = P_ztcp_To_B - P_otcp_To_B
        dire_vec_z_o = np.linalg.inv(transforms[0][:3, :3]) @ vector_Z / np.linalg.norm(vector_Z)

        # cal the dircction vector of y
        dire_vec_y_o = np.cross(dire_vec_z_o.T, dire_vec_x_o.T)

        # modify the dircction vector of z 
        dire_vec_z_o = np.cross(dire_vec_x_o.T, dire_vec_y_o)

        # cal rotation matrix
        tool_rot = np.zeros((3, 3))
        tool_rot[:, 0] = dire_vec_x_o.T
        tool_rot[:, 1] = dire_vec_y_o
        tool_rot[:, 2] = dire_vec_z_o
        return tool_rot