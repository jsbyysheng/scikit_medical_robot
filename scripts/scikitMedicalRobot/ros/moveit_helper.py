import sys
import copy
from tqdm.auto import tqdm
# math
import numpy as np
import tf_conversions.posemath as pm
# plot
import matplotlib as mpl
import matplotlib.pyplot as plt
# moveit
import rospy
import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
# toppra
import toppra as ta
# ta.setup_logging("INFO")
import toppra.constraint as constraint
import toppra.algorithm as algo
import toppra.interpolator as interpolator
# PyKDL
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

class robot_control:
    def __init__(self, argv=sys.argv, group_name='manipulator', base_link_name='base_link', ee_link_name='tool0') -> None:
        moveit_commander.roscpp_initialize(argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
    
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        # This is very significant for cartesian plan !
        self.move_group.set_end_effector_link(ee_link_name)
        # move_group.get_jacobian_matrix is not affected, so you cannot use it directly
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.current_state = self.robot.get_current_state()

        # pilz_industrial_motion_planner: PTP, LIN, CIRC
        # ompl: manipulator[RRTConnect]
        self._plan_conf_pipline_id = 'ompl'
        self._plan_conf_planner_id = 'manipulator[RRTConnect]'
        self._plan_conf_planning_time = 1.0
        self._plan_conf_attempts = 5
        self._plan_conf_max_v = 0.1
        self._plan_conf_max_a = 0.1
        self._plan_conf_looking = True
        self._plan_conf_replanning = True

        self.move_group.set_planning_pipeline_id(self._plan_conf_pipline_id)
        self.move_group.set_planner_id(self._plan_conf_planner_id)
        self.move_group.set_planning_time(self._plan_conf_planning_time)
        self.move_group.set_num_planning_attempts(self._plan_conf_attempts)
        self.move_group.set_max_velocity_scaling_factor(self._plan_conf_max_v)
        self.move_group.set_max_acceleration_scaling_factor(self._plan_conf_max_a)
        self.move_group.allow_looking(self._plan_conf_looking)
        self.move_group.allow_replanning(self._plan_conf_replanning)

        self.rkdl = robot_kdl_kinematics(param_robot_description='robot_description', chain_link=(base_link_name, ee_link_name))
        self.toppra_postprocessor = toppra_post_processor(self.rkdl.fk_vel)

    @property
    def plan_conf_pipline_id(self):
        self._plan_conf_pipline_id = self.move_group.get_planning_pipeline_id()
        return self._plan_conf_pipline_id

    @plan_conf_pipline_id.setter
    def plan_conf_pipline_id(self, val: str):
        self.move_group.set_planning_pipeline_id(val)
        self._plan_conf_pipline_id = self.move_group.get_planning_pipeline_id()

    @property
    def plan_conf_planner_id(self):
        self._plan_conf_planner_id = self.move_group.get_planner_id()
        return self._plan_conf_planner_id

    @plan_conf_planner_id.setter
    def plan_conf_planner_id(self, val: str):
        self.move_group.set_planner_id(val)
        self._plan_conf_planner_id = self.move_group.get_planner_id()  

    @property
    def plan_conf_planning_time(self):
        self._plan_conf_planning_time = self.move_group.get_planning_time()
        return self._plan_conf_planning_time

    @plan_conf_planning_time.setter
    def plan_conf_planning_time(self, val: float):
        self.move_group.set_planning_time(val)
        self._plan_conf_planning_time = self.move_group.get_planning_time()

    @property
    def plan_conf_attempts(self):
        return self._plan_conf_attempts

    @plan_conf_attempts.setter
    def plan_conf_attempts(self, val: int):
        self.move_group.set_num_planning_attempts(val)
        self._plan_conf_attempts = val

    @property
    def plan_conf_max_v(self):
        return self._plan_conf_max_v

    @plan_conf_max_v.setter
    def plan_conf_max_v(self, val:float):
        self.move_group.set_max_velocity_scaling_factor(val)
        self._plan_conf_max_v = val

    @property
    def plan_conf_max_a(self):
        return self._plan_conf_max_a

    @plan_conf_max_a.setter
    def plan_conf_max_a(self, val: float):
        self.move_group.set_max_acceleration_scaling_factor(val)
        self._plan_conf_max_a = val

    @property
    def plan_conf_looking(self):
        return self._plan_conf_looking

    @plan_conf_looking.setter
    def plan_conf_looking(self, val: bool):
        self.move_group.allow_looking(val)
        self._plan_conf_looking = val

    @property
    def plan_conf_replanning(self):
        return self._plan_conf_replanning
    
    @plan_conf_replanning.setter
    def plan_conf_replanning(self, val: bool):
        self.move_group.allow_replanning(val)
        self._plan_conf_replanning = val

    def quick_plan(self, waypoints, method='cartesian', toppra_paras=None, interpolate_resolution=0.0005, jump_threshold=0.0):
        # self.move_group.stop()
        self.move_group.clear_pose_targets()
        if method == 'cartesian':
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold)
            return plan, (fraction)
        elif method == 'toppra':
            assert toppra_paras is not None
            (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, interpolate_resolution, jump_threshold)
            points, (replaned, new_ts, new_qs, new_qds, new_qdds) = self.toppra_postprocessor.quick(
                plan, 
                toppra_paras['robot_traj_sample_rate'], 
                toppra_paras['jnt_vlims'], 
                toppra_paras['jnt_alims'],
                toppra_paras['cartesian_lims'])
            plan.joint_trajectory.points.clear()
            plan.joint_trajectory.points = points
            return plan, (replaned, new_ts, new_qs, new_qds, new_qdds)
        else:
            self.move_group.set_pose_target(waypoints[0])
            (result, plan, fraction, others) = self.move_group.plan()
            return plan, (result, fraction, others)

    def quick_plan_cartesian_concatenate(self, waypoints_list, interpolate_resolution=0.0005, jump_threshold=0.0, no_tqdm=True, robot_state=None):
        # waypoints_list = [{'toppra': toppra, 'waypoints': waypoints}]
        # return others = [{'toppra_paras':toppra_paras, 'plan': plan}, ...]
        if robot_state == None:
            robot_state = self.move_group.get_current_state()

        self.move_group.set_start_state(robot_state)
        others = []

        plan, _ = self.quick_plan([waypoints_list[0]['waypoints'][0]], method='cartesian', interpolate_resolution=interpolate_resolution, jump_threshold=jump_threshold)
        others.append({'toppra_paras':waypoints_list[0]['toppra'], 'plan': plan})
        last_joints_position = plan.joint_trajectory.points[-1].positions
        pbar = None
        if not no_tqdm:
            pbar = tqdm(total=len(waypoints_list))
        for idx in range(0, len(waypoints_list)):
            if pbar is not None:
                pbar.update(1)
            waypoints = waypoints_list[idx]['waypoints']
            toppra_paras = waypoints_list[idx]['toppra']
            if len(waypoints) < 1:
                rospy.logwarn(f'quick_plan_cartesian_concatenate receive len(waypoints) < 1, continue.')
                continue
            if len(waypoints) == 1 and idx == 0:
                continue
            if idx == 0:
                waypoints = waypoints[1:]
            # Update robot model
            robot_state.joint_state.position = last_joints_position
            self.move_group.set_start_state(robot_state)
            # Cartesian Plan
            plan, _ = self.quick_plan(waypoints, method='cartesian', interpolate_resolution=interpolate_resolution, jump_threshold=jump_threshold)
            others.append({'toppra_paras':toppra_paras, 'plan': plan})
            last_joints_position = plan.joint_trajectory.points[-1].positions

        return others

    def quick_plan_toppra_concatenate(self, waypoints_list, concatenate_delay=0, interpolate_resolution=0.0005, jump_threshold=0.0, no_tqdm=True, robot_state=None):
        # waypoints_list = [{'toppra': toppra, 'waypoints': waypoints}]
        if robot_state == None:
            robot_state = self.move_group.get_current_state()

        self.move_group.set_start_state(robot_state)

        plan, (replaned, all_ts, all_qs, all_qds, all_qdds) = self.quick_plan([waypoints_list[0]['waypoints'][0]], method='toppra', toppra_paras=waypoints_list[0]['toppra'], interpolate_resolution=interpolate_resolution, jump_threshold=jump_threshold)
        pbar = None
        if not no_tqdm:
            pbar = tqdm(total=len(waypoints_list))
        for idx in range(0, len(waypoints_list)):
            if pbar is not None:
                pbar.update(1)
            waypoints = waypoints_list[idx]['waypoints']
            toppra_paras = waypoints_list[idx]['toppra']
            if len(waypoints) < 1:
                rospy.logwarn(f'quick_plan_toppra_concatenate receive len(waypoints) < 1, continue.')
                continue
            if len(waypoints) == 1 and idx == 0:
                continue

            if idx == 0:
                waypoints = waypoints[1:]

            # Update robot model
            robot_state.joint_state.position = all_qs[-1].tolist()
            self.move_group.set_start_state(robot_state)
            # Toppra Plan
            plan, (replaned, new_ts, new_qs, new_qds, new_qdds) = self.quick_plan(waypoints, method='toppra', toppra_paras=toppra_paras, interpolate_resolution=interpolate_resolution, jump_threshold=jump_threshold)

            # Concatenate Results
            new_ts = new_ts +  all_ts[-1] + concatenate_delay
            all_ts = np.concatenate((all_ts, new_ts))
            all_qs = np.concatenate((all_qs, new_qs))
            all_qds = np.concatenate((all_qds, new_qds))
            all_qdds = np.concatenate((all_qdds, new_qdds))

        points = self.toppra_postprocessor.generate_joint_trajectory_points(all_ts, all_qs, all_qds, all_qdds)
        plan.joint_trajectory.points = points
        return plan, (replaned, all_ts, all_qs, all_qds, all_qdds)

    def move_plan(self, plan, wait=True):
        return self.move_group.execute(plan, wait=wait)

    def move_relative(self, direction, val, wait=True, method='cartesian', interpolate_resolution=0.0005, jump_threshold=0.0):
        wpose = self.move_group.get_current_pose().pose
        if direction == 'x':
            wpose.position.x += val
        elif direction == 'y':
            wpose.position.y += val
        elif direction == 'z':
            wpose.position.z += val

        waypoints = []
        waypoints.append(copy.deepcopy(wpose))
        plan, _ = self.quick_plan(waypoints, method=method, interpolate_resolution=interpolate_resolution, jump_threshold=jump_threshold)
        return self.move_plan(plan, wait=wait)

    def move_absolute(self, direction, val, wait=True, method='cartesian', interpolate_resolution=0.0005, jump_threshold=0.0):
        wpose = self.move_group.get_current_pose().pose
        if direction == 'x':
            wpose.position.x = val
        elif direction == 'y':
            wpose.position.y = val
        elif direction == 'z':
            wpose.position.z = val

        waypoints = []
        waypoints.append(copy.deepcopy(wpose))
        plan, _ = self.quick_plan(waypoints, method=method, interpolate_resolution=interpolate_resolution, jump_threshold=jump_threshold)
        return self.move_plan(plan, wait=wait)

    def __delattr__(self, __name: str) -> None:
        # del self.move_group
        moveit_commander.roscpp_shutdown()

class robot_kdl_kinematics:
    def __init__(self, param_robot_description='robot_description', chain_link=('base_link', 'tool0')) -> None:
        _, self.urdf_tree = treeFromParam(param_robot_description)
        self.urdf_chain = self.urdf_tree.getChain(chain_link[0], chain_link[1])
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)

    def fk_vel(self, q, dq):
        # moveit method
        # jaco = move_group.get_jacobian_matrix(q.tolist())
        # v = jaco @ dq
        # return np.linalg.norm(v[:3]), np.linalg.norm(v[3:])
        jq = kdl.JntArrayVel(self.urdf_chain.getNrOfJoints())
        for i in range(self.urdf_chain.getNrOfJoints()):
            jq.q[i] = q[i]
            jq.qdot[i] = dq[i]
        framevel = kdl.FrameVel()
        result = self.fk_solver_vel.JntToCart(jq, framevel)
        if 0 != result:
            raise Exception(f"Error solving TCP velocity: Error code = {result}")
        twist = framevel.GetTwist()
        return twist.vel.Norm(), twist.rot.Norm()

    def fk(self, q):
        jq = kdl.JntArray(self.urdf_chain.getNrOfJoints())
        for i in range(self.urdf_chain.getNrOfJoints()):
            jq[i] = q[i]
        frame = kdl.Frame()
        result = self.fk_solver_pos.JntToCart(jq, frame)
        return pm.toMatrix(frame), result

class toppra_post_processor:
    def __init__(self, compute_fk_vel) -> None:
        self.compute_fk_vel = compute_fk_vel
        self.ParametrizeSpline = 'ParametrizeSpline'
        self.ParametrizeConstAccel = 'ParametrizeConstAccel'
        self.Solver_seidel = 'seidel'
        self.Solver_hotqpoases = 'hotqpoases'
        self.Solver_qpoases = 'qpoases'
        self.Solver_cvxpy = 'cvxpy'
        self.Solver_cvxopt = 'cvxopt'
        self.Solver_ecos = 'ecos'

    def quick(self, plan, robot_traj_sample_rate, jnt_vlims, jnt_alims, cartesian_lims, plot=False):
        duration, ts, jnts = self.extract_moveit_plan_content(plan)
        replaned = False
        try:
            new_ts, new_qs, new_qds, new_qdds = self.toppra_trajactory(
                ts, jnts, jnt_vlims, jnt_alims, 
                cartesian_lims=cartesian_lims, 
                robot_traj_sample_rate=robot_traj_sample_rate, 
                parametrizer=self.ParametrizeConstAccel, 
                solver_wrapper='hotqpoases',
                max_err_threshold = 1e-5,
                min_nb_points=int(len(jnts) * 2)
            )
        except Exception as e:
            # rospy.logwarn(f"For the exception of {e}, the hotqpoases solver is replaced by the ecos solver for replanning.")
            waypoints = []
            waypoints.append(jnts[0])
            waypoints.append(jnts[-1])
            new_ts, new_qs, new_qds, new_qdds = self.toppra_trajactory(
                [0, 1], waypoints, jnt_vlims, jnt_alims, 
                cartesian_lims=cartesian_lims, 
                robot_traj_sample_rate=robot_traj_sample_rate, 
                parametrizer=self.ParametrizeConstAccel, 
                solver_wrapper='ecos',
                max_err_threshold = 1e-5,
                min_nb_points=32
            )
            replaned = True

        if plot:
            self.plot_toppra_results(new_ts, new_qs, new_qds, new_qdds)

        points = self.generate_joint_trajectory_points(new_ts, new_qs, new_qds, new_qdds)
        return points, (replaned, new_ts, new_qs, new_qds, new_qdds)

    def quick_concatenate(self, others):
        # others = [{'toppra_paras':toppra_paras, 'plan': plan}, ...]
        other = others[0]
        toppra_paras = other['toppra_paras']
        points, (replaned, all_ts, all_qs, all_qds, all_qdds) = self.quick(
            other['plan'], 
            toppra_paras['robot_traj_sample_rate'], 
            toppra_paras['jnt_vlims'], 
            toppra_paras['jnt_alims'],
            toppra_paras['cartesian_lims'])
        concatenate_delay = 1 / toppra_paras['robot_traj_sample_rate']

        for idx, other in enumerate(others[1:]):
            toppra_paras = other['toppra_paras']
            points, (replaned, new_ts, new_qs, new_qds, new_qdds) = self.quick(
                other['plan'], 
                toppra_paras['robot_traj_sample_rate'], 
                toppra_paras['jnt_vlims'], 
                toppra_paras['jnt_alims'],
                toppra_paras['cartesian_lims'])

            # Concatenate Results
            new_ts = new_ts +  all_ts[-1] + concatenate_delay
            all_ts = np.concatenate((all_ts, new_ts))
            all_qs = np.concatenate((all_qs, new_qs))
            all_qds = np.concatenate((all_qds, new_qds))
            all_qdds = np.concatenate((all_qdds, new_qdds))

        points = self.generate_joint_trajectory_points(all_ts, all_qs, all_qds, all_qdds)
        other['plan'].joint_trajectory.points = points
        return other['plan']

    # parametrizer: ParametrizeSpline ParametrizeConstAccel
    # solver_wrapper: seidel, hotqpoases, qpoases, cvxpy, cvxopt, ecos
    def toppra_trajactory(
            self,
            ts, jnts, jnt_vlims, jnt_alims, 
            robot_traj_sample_rate=125, 
            sd_start=0, sd_end=0, 
            desired_time=0, 
            cartesian_lims=(0, 0), 
            parametrizer="ParametrizeConstAccel", 
            solver_wrapper='hotqpoases', 
            max_err_threshold = 1e-4,
            max_iteration = 100,
            max_seg_length = 0.05,
            min_nb_points = 100):
        path = ta.SplineInterpolator(ts, jnts)
        vlim = np.vstack((-jnt_vlims, jnt_vlims)).T
        alim = np.vstack((-jnt_alims, jnt_alims)).T
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim, discretization_scheme=constraint.DiscretizationType.  Interpolation)
        consts = [pc_vel, pc_acc]

        if cartesian_lims[0] != 0 or cartesian_lims[1] != 0:
            pc_cart_vel = CartesianSpeedConstraint(self.compute_fk_vel, cartesian_lims[0], cartesian_lims[1], 6)
            consts.append(pc_cart_vel)
            # print(f'With CartesianSpeedConstraint: {cartesian_lims[0]}, {cartesian_lims[1]}')
        else:
            pass
            # print(f'With no CartesianSpeedConstraint')

        gridpoints = interpolator.propose_gridpoints(path, max_err_threshold, max_iteration, max_seg_length,    min_nb_points)
        # return path, max_err_threshold, max_iteration, max_seg_length, min_nb_points
        if desired_time > 0:
            instance = algo.TOPPRAsd(consts, path, parametrizer=parametrizer, solver_wrapper=solver_wrapper,    gridpoints=gridpoints)
            instance.set_desired_duration(desired_time)
        else:
            instance = algo.TOPPRA(consts, path, parametrizer=parametrizer, solver_wrapper=solver_wrapper,  gridpoints=gridpoints)

        jnt_traj = instance.compute_trajectory(sd_start, sd_end)
        new_ts = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration * robot_traj_sample_rate))
        qs = jnt_traj.eval(new_ts)
        qds = jnt_traj.evald(new_ts)
        qdds = jnt_traj.evaldd(new_ts)
        return new_ts, qs, qds, qdds

    def plot_toppra_results(self, new_ts, qs, qds, qdds):
        # Extract TCP speeds in order to plot
        fkv = np.vectorize(self.compute_fk_vel, signature='(n),(n)->(),()')
        linear_spd, angular_spd = fkv(qs, qds)
        fig, axs = plt.subplots(4, 1, sharex=True, figsize = (50, 20))
        for i in range(6):
            # plot the i-th joint trajectory
            axs[0].plot(new_ts, qs[:, i], c="C{:d}".format(i))
            axs[1].plot(new_ts, qds[:, i], c="C{:d}".format(i))
            axs[2].plot(new_ts, qdds[:, i], c="C{:d}".format(i))

        # Plot the cartesian linear speed and angular speed of the TCP
        axs[3].plot(new_ts, linear_spd)
        axs[3].plot(new_ts, angular_spd)

        axs[0].set_ylabel("Position (rad)")
        axs[1].set_ylabel("Velocity (rad/s)")
        axs[2].set_ylabel("Acceleration (rad/s2)")
        axs[3].set_ylabel("Cartesian velocity (m/s)")
        axs[3].set_xlabel("Time (s)")
        plt.show()

    def extract_moveit_plan_content(self, plan):
        ts = []
        jnts = []

        for w in plan.joint_trajectory.points:
            ts.append(w.time_from_start.secs + w.time_from_start.nsecs * 1e-9)
            jnts.append(w.positions)
        ts = np.array(ts)
        duration = ts[-1]
        ts = ts / ts[-1]
        jnts = np.array(jnts)
        return duration, ts, jnts

    def generate_joint_trajectory_points(self, ts, qs, qds, qdds):
        points = []
        for idx, t in enumerate(ts):
            point = JointTrajectoryPoint()
            point.positions = qs[idx].tolist()
            point.velocities = qds[idx].tolist()
            point.accelerations = qdds[idx].tolist()
            point.effort = []
            point.time_from_start.secs = int(t)
            point.time_from_start.nsecs = int((t - point.time_from_start.secs) * 1e9)
            points.append(point)
        return points

class CartesianSpeedConstraint(constraint.SecondOrderConstraint):
    """
    This class implements a constraint on the magnitudes of the linear & angular
    Cartesian velocity vectors of one of the robot's parts (link, joint, etc).
    
    The forward kinematic velocity is be provided via a callback, which makes
    this constraint agnostic of the robot's geometry and agnostic of whatever
    FK algorithm is used.
    """

    def __init__(self, fk_vel, linear_speed_max, angular_speed_max, dof):
        """Initialize the constraint.
        Parameters
        ----------
        fk: (np.ndarray, np.ndarray) -> (float, float)
            The "FK" function that receives joint positions and velocities as
            inputs and outputs the magnitude of the linear and angular
            velocity vectors for some the monitored part of the robot.
        linear_speed_max: float
            The max linear speed allowed for the monitored part.
        angular_speed_max: float
            The max angular speed allowed for the monitored part.
        dof: int
            The dimension of the joint position.
        """
        super(CartesianSpeedConstraint, self).__init__(self.invdyn, self.constraintf, self.constraintg, dof)
        self.fk_vel = fk_vel
        self.linear_speed_max = linear_speed_max
        self.angular_speed_max = angular_speed_max

    def invdyn(self, q, dq, ddq):
        linear_speed, angular_speed = self.fk_vel(q, dq)
        return np.array([linear_speed**2, angular_speed**2])

    def constraintf(self, q):
        return np.identity(2)

    def constraintg(self, q):
        return np.array([self.linear_speed_max**2, self.angular_speed_max**2])