import numpy as np
import pickle
from scikitMedicalRobot.ros import tf2_helper
from scikitMedicalRobot.calibration.tool import tcp_calibration
from .sample_helper import jupyter_quick_sample
from ..utilities import generate_timedate_cache_file

OUTER_MARKER_ID = 999999
OUTER_MARKER_COLOR = (0.0, 1.0, 0.0, 0.1)
OUTER_MARKER_SCALE = (0.005, 0.005, 0.005)

CENTER_MARKER_ID = 99999
CENTER_MARKER_COLOR = (0.0, 0.0, 1.0, 1.0)
CENTER_MARKER_SCALE = (0.01, 0.01, 0.01)

PICKED_MARKER_ID = 9999
PICKED_MARKER_COLOR = (0.0, 1.0, 0.0, 1.0)
PICKED_MARKER_SCALE = (0.005, 0.005, 0.005)
TOOLTIP_MARKER_COLOR = (1.0, 0.0, 0.0, 1.0)
TOOLTIP_MARKER_SCALE = (0.005, 0.005, 0.005)

class position_recorder:
    
    def __init__(self, th: tf2_helper, position_recorder, name='position_recorder', caches_dir='exp', child_dir='position_recorder') -> None:
        self.th = th
        self.position_recorder = position_recorder
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

    def sample(self, N):
        trans = self.th.sample(target_link=self.position_recorder)
        trans = self.th.matrix_from_transform(trans.transform)
        if trans is not None:
            position, quat = self.th.matrix_to_position_quat(trans)
            self.th.add_sphere_marker(position, marker_id=N+PICKED_MARKER_ID, rgba=PICKED_MARKER_COLOR, scale=PICKED_MARKER_SCALE)
            print(f"position: \n{position}")
            return position
        return None

    def evaluate(self, _):
        return f'no evaluation'

    def clear_samples(self, datas):
        # th.delete_all_markers()
        for idx, _ in enumerate(datas):
            self.th.add_sphere_marker([0, 0, 0], marker_id=idx+PICKED_MARKER_ID, action=False)

    def save_samples(self, datas, _):
        f_name = generate_timedate_cache_file(caches_dir=self.caches_dir, child_dir=self.child_dir, filetype='pickle', suffix=self.name)
        with open(f_name, "wb") as f:
            pickle.dump(np.array(datas), f)
        print(f"saved to {f_name}")

    def run(self):
        return self.sampler.run_sample()

class pose_recorder:
    
    def __init__(self, th: tf2_helper, pose_link_name, name='pose_recorder', caches_dir='exp', child_dir='pose_recorder') -> None:
        self.th = th
        self.pose_link_name = pose_link_name
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

    def sample(self, N):
        trans = self.th.sample(target_link=self.pose_link_name)
        trans = self.th.matrix_from_transform(trans.transform)
        if trans is not None:
            position, quat = self.th.matrix_to_position_quat(trans)
            self.th.add_sphere_marker(position, marker_id=N+PICKED_MARKER_ID, rgba=PICKED_MARKER_COLOR, scale=PICKED_MARKER_SCALE)
            print(f"Trans: \n{trans}")
        return trans

    def evaluate(self, _):
        return f'no evaluation'

    def clear_samples(self, datas):
        # th.delete_all_markers()
        for idx, trans in enumerate(datas):
            self.th.add_sphere_marker([0, 0, 0], marker_id=idx+PICKED_MARKER_ID, action=False)

    def save_samples(self, datas, _):
        f_name = generate_timedate_cache_file(caches_dir=self.caches_dir, child_dir=self.child_dir, filetype='pickle', suffix=self.name)
        with open(f_name, "wb") as f:
            pickle.dump(np.array(datas), f)
        print(f"saved to {f_name}")

    def run(self):
        return self.sampler.run_sample()

class tool_tip_calibration_recoder:

    def __init__(self, th: tf2_helper, ee_link_name='tool0', tool_calibration_link_name='tool_calibration_link',name='tool_tip_calibration', caches_dir='exp', child_dir='tool_tip_calibration') -> None:
        self.th = th
        self.ee_link_name = ee_link_name
        self.tool_calibration_link_name = tool_calibration_link_name
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

    def sample(self, N):
        trans = self.th.sample(target_link=self.ee_link_name)
        trans = self.th.matrix_from_transform(trans.transform)
        if trans is not None:
            position, quat = self.th.matrix_to_position_quat(trans)
            self.th.add_sphere_marker(position, marker_id=N, rgba=TOOLTIP_MARKER_COLOR, scale=TOOLTIP_MARKER_SCALE)
            print(f"Trans: \n{trans}")
            
        return trans

    def evaluate(self, datas):
        if len(datas) >= 4:
            center, r, pcov = tcp_calibration.pivot(np.array(datas))
            self.th.add_sphere_marker(center, marker_id=OUTER_MARKER_ID, rgba=OUTER_MARKER_COLOR, scale=(2 * r, 2 * r, 2 * r))
            self.th.add_sphere_marker(center, marker_id=CENTER_MARKER_ID, rgba=CENTER_MARKER_COLOR, scale=CENTER_MARKER_SCALE)
            ret = tcp_calibration.tip2tip(datas)
            print(f"\nfrom tool0 -> real_tool: \n{ret}")
            self.th.pub_static_tf(
                [ret[0, 0], ret[1, 0], ret[2, 0]], 
                [0, 0, 0, 1], 
                frame=self.tool_calibration_link_name, 
                parent_frame=self.ee_link_name
            )

            err, reprojection = tcp_calibration.evaluate(datas, ret)
            print(f"reprojection err: {err * 1000} mm")
            center_x = np.mean(np.array(reprojection)[:, 0])
            center_y = np.mean(np.array(reprojection)[:, 1])
            center_z = np.mean(np.array(reprojection)[:, 2])
            print(f'center: {[center_x, center_y, center_z]}')
            return ret, (center, r, pcov), (err, reprojection), (center_x, center_y, center_z)
        else:
            return None

    def clear_samples(self, datas):
        self.th.delete_all_markers()
        # for idx, trans in enumerate(datas):
        #     th.add_sphere_marker([0, 0, 0], marker_id=idx,    action=False)

        self.th.add_sphere_marker([0, 0, 0], marker_id=CENTER_MARKER_ID, action=False)
        self.th.add_sphere_marker([0, 0, 0], marker_id=OUTER_MARKER_ID,  action=False)

    def save_samples(self, datas, name):
        f_name = generate_timedate_cache_file(caches_dir=self.caches_dir, child_dir=self.child_dir, filetype='pickle', suffix=self.name)
        with open(f_name, "wb") as f:
            pickle.dump(np.array(datas), f)
        print(f"saved to {f_name}")

    def run(self):
        return self.sampler.run_sample()