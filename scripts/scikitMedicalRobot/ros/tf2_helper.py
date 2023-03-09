import rospy
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import geometry_msgs.msg
import visualization_msgs
from nav_msgs.msg import Path
import numpy as np

'''
!!!Note!!!

Axes 4-string: e.g. ‘sxyz’ or ‘ryxy’
    first character : rotations are applied to ‘s’tatic or ‘r’otating frame
    remaining characters : successive rotation axis ‘x’, ‘y’, or ‘z’

Quaternions ix+jy+kz+w are represented as [x, y, z, w].
''' 

class tf2_helper:
    def __init__(self) -> None:
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.vis_pub = rospy.Publisher('visualization_marker', visualization_msgs.msg.Marker, queue_size=0)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.path_msg = Path()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.transformation = tf_conversions.transformations

    def sample(self, ref_link='base_link', target_link='tool0'):
        trans = None
        try:
            trans = self.tfBuffer.lookup_transform(ref_link, target_link, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
       
        return trans

    def pub_static_tf(self, position, quat, frame, parent_frame='base_link'):
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame
        static_transformStamped.child_frame_id = frame

        static_transformStamped.transform.translation.x = position[0]
        static_transformStamped.transform.translation.y = position[1]
        static_transformStamped.transform.translation.z = position[2]

        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(static_transformStamped)

    def transform_pose(self, position, quat, from_frame, to_frame='base_link'):
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        try:
            output_pose_stamped = self.tfBuffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            output_position = [output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y, output_pose_stamped.pose.position.z]
            output_quat = [output_pose_stamped.pose.orientation.x, output_pose_stamped.pose.orientation.y, output_pose_stamped.pose.orientation.z,  output_pose_stamped.pose.orientation.w]
            return np.array(output_position), np.array(output_quat)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def delete_all_markers(self):
        marker = visualization_msgs.msg.Marker()
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.vis_pub.publish(marker)

    def delete_sphere_marker(self, marker_id):
        self.add_sphere_marker(np.array([0, 0, 0]), marker_id=marker_id, action=False)

    def add_sphere_marker(self, position, marker_id=0, action=True, scale=(0.01, 0.01, 0.01), rgba=(0.0, 1.0, 0.0, 1.0), frame='base_link'):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""
        marker.id = marker_id
        marker.type = visualization_msgs.msg.Marker.SPHERE
        if action:
            marker.action = visualization_msgs.msg.Marker.ADD
        else:
            marker.action = visualization_msgs.msg.Marker.DELETE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = rgba[3]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        self.vis_pub.publish(marker)

    def delete_arrow_marker(self, marker_id):
        self.add_arrow_marker(np.array([0, 0, 0]), np.array([0, 0, 0]), action=False, marker_id=marker_id)

    def add_arrow_marker(self, position_start, position_end, marker_id=0, action=True, shaft_diameter=0.002, head_diameter=0.01, head_length=0.01, rgba=(0.0, 1.0, 0.0, 1.0), frame='base_link'):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""
        marker.id = marker_id
        marker.type = visualization_msgs.msg.Marker.ARROW
        if action:
            marker.action = visualization_msgs.msg.Marker.ADD
        else:
            marker.action = visualization_msgs.msg.Marker.DELETE

        p0 = geometry_msgs.msg.Point()
        p0.x = position_start[0]
        p0.y = position_start[1]
        p0.z = position_start[2]
        marker.points.append(p0)
        p1 = geometry_msgs.msg.Point() 
        p1.x = position_end[0]
        p1.y = position_end[1]
        p1.z = position_end[2]
        marker.points.append(p1)
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = shaft_diameter
        marker.scale.y = head_diameter
        marker.scale.z = head_length
        marker.color.a = rgba[3]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        self.vis_pub.publish(marker)

    def add_text_marker(self, text, position, marker_id=0, action=True, text_size=0.01, rgba=(0.0, 1.0, 0.0, 1.0), frame='base_link'):
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""
        marker.id = marker_id
        marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
        if action:
            marker.action = visualization_msgs.msg.Marker.ADD
        else:
            marker.action = visualization_msgs.msg.Marker.DELETE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = text_size
        marker.color.a = rgba[3]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.text = str(text)
        self.vis_pub.publish(marker)

    def generate_path_point(self, positon, quat, frame_id, stamp):
        self.path_msg.header.stamp = stamp
        self.path_msg.header.frame_id = frame_id

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id
        pose.pose.position.x = positon[0]
        pose.pose.position.y = positon[1]
        pose.pose.position.z = positon[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def publish_path(self):
        self.path_pub.publish(self.path_msg)

    def clear_path(self):
        self.path_msg.poses = []

    @staticmethod
    def transform_to_pose(transform: geometry_msgs.msg.Transform):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation.x = transform.rotation.x
        pose.orientation.y = transform.rotation.y
        pose.orientation.z = transform.rotation.z
        pose.orientation.w = transform.rotation.w
        return pose

    @staticmethod
    def transform_to_matrix(transform: geometry_msgs.msg.Transform):
        m = tf_conversions.transformations.quaternion_matrix([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        m[0, 3] = transform.translation.x
        m[1, 3] = transform.translation.y
        m[2, 3] = transform.translation.z
        return m

    @staticmethod
    def transform_to_quat_position(transform: geometry_msgs.msg.Transform):
        position = [transform.translation.x, transform.translation.y, transform.translation.z]
        quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        return position, quat

    @staticmethod
    def position_quat_to_matrix(position, quat):
        m = tf_conversions.transformations.quaternion_matrix(quat)
        m[0, 3] = position[0]
        m[1, 3] = position[1]
        m[2, 3] = position[2]
        return m

    @staticmethod
    def position_quat_to_pose(position, quat):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    @staticmethod
    def position_quat_from_pose(pose: geometry_msgs.msg.Pose):
        return tf2_helper.pose_to_position_quat(pose)

    @staticmethod
    def pose_to_transform(pose: geometry_msgs.msg.Pose):
        transform = geometry_msgs.msg.Transform()
        transform.translation.x = pose.position.x
        transform.translation.y = pose.position.y
        transform.translation.z = pose.position.z
        transform.rotation.x = pose.orientation.x
        transform.rotation.y = pose.orientation.y
        transform.rotation.z = pose.orientation.z
        transform.rotation.w = pose.orientation.w
        return transform

    @staticmethod
    def pose_to_matrix(pose: geometry_msgs.msg.Pose):
        m = tf_conversions.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        m[0, 3] = pose.position.x
        m[1, 3] = pose.position.y
        m[2, 3] = pose.position.z
        return m

    @staticmethod
    def pose_to_position_quat(pose: geometry_msgs.msg.Pose):
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return position, quat

    @staticmethod
    def pose_to_position_euler(pose: geometry_msgs.msg.Pose, axes='sxyz'):
        position, quat = tf2_helper.pose_to_position_quat(pose)
        euler_angle = tf_conversions.transformations.euler_from_quaternion(quat, axes)
        return position, euler_angle

    @staticmethod
    def pose_from_position_quat(position, quat):
        return tf2_helper.position_quat_to_pose(position, quat)

    @staticmethod
    def matrix_to_position_quat(matrix):
        quat = tf_conversions.transformations.quaternion_from_matrix(matrix)
        position = matrix[0:3, 3]
        return position, quat

    @staticmethod
    def matrix_to_quat(m):
        if m.shape == (3, 3):
            new_m = np.eye(4)
            new_m[:3, :3] = m
            m = new_m
        return tf_conversions.transformations.quaternion_from_matrix(m)

    @staticmethod
    def matrix_from_transform(transform: geometry_msgs.msg.Transform):
        return tf2_helper.transform_to_matrix(transform)

    @staticmethod
    def matrix_from_pose(pose: geometry_msgs.msg.Pose):
        return tf2_helper.pose_to_matrix(pose)

    @staticmethod
    def matrix_from_position_quat(position, quat):
        return tf2_helper.position_quat_to_matrix(position, quat)

    @staticmethod
    def matrix_to_transform(m):
        transform = geometry_msgs.msg.Transform()
        transform.translation.x = m[0, 3]
        transform.translation.y = m[1, 3]
        transform.translation.z = m[2, 3]
        quat = tf_conversions.transformations.quaternion_from_matrix(m)
        transform.rotation.x = quat[0]
        transform.rotation.y = quat[1]
        transform.rotation.z = quat[2]
        transform.rotation.w = quat[3]
        return transform

    @staticmethod
    def matrix_to_pose(m):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = m[0, 3]
        pose.position.y = m[1, 3]
        pose.position.z = m[2, 3]
        quat = tf_conversions.transformations.quaternion_from_matrix(m)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    @staticmethod
    def euler_to_matrix(euler, axes='sxyz'):
        return tf_conversions.transformations.euler_matrix(*euler, axes=axes)

    @staticmethod
    def position_euler_to_pose(position, euler, axes='sxyz'):
        quat = tf_conversions.transformations.quaternion_from_euler(*euler, axes)
        return tf2_helper.position_quat_to_pose(position, quat)

    @staticmethod
    def position_euler_to_position_quat(position, euler, axes='sxyz'):
        quat = tf_conversions.transformations.quaternion_from_euler(*euler, axes)
        return position, quat

    @staticmethod
    def plan_robot_tool_tip_6D(g_tool_in_base: np.ndarray, tool2ee: np.ndarray):
        # base -> tool @ tool -> robot endeffector
        return g_tool_in_base @ tool2ee

    @staticmethod
    def plan_robot_tool_tip_3D(c_tool_rotation_in_base: np.ndarray, g_tool_position_in_base: np.ndarray, tool2ee: np.ndarray):
        # base -> tool @ tool -> robot endeffector
        g_tool = np.eye(4)
        g_tool[:3, :3] = c_tool_rotation_in_base
        g_tool[:3, 3:] = g_tool_position_in_base.reshape(3, 1)
        return tf2_helper.plan_robot_tool_tip_6D(g_tool, tool2ee)

    @staticmethod
    def plan_robot_tool_tip_5D(enter_vec_in_ee: np.ndarray, g_tool_position_in_base: np.ndarray, tool2ee: np.ndarray, base2ee: np.ndarray, reference_point_in_ee=np.array([0, 0, 0]), unknown_axis='x'):
        p1_in_ee = np.linalg.inv(tool2ee)[0:3, 3]
        p0_in_ee = p1_in_ee + enter_vec_in_ee
        g_tool_in_ee = tf2_helper.create_5D_tool(
            p0_in_ee, p1_in_ee, 
            unknown_axis=unknown_axis, reference_point=reference_point_in_ee
        )
        # goal in ee -> goal in base
        g_tool_in_base = base2ee @ g_tool_in_ee
        g_tool_in_base[0:3, 3] = g_tool_position_in_base
        return tf2_helper.plan_robot_tool_tip_6D(g_tool_in_base, tool2ee)

    @staticmethod
    def create_5D_tool(point_end: np.ndarray, point_start: np.ndarray, reference_point=np.array([0, 0, 0]), unknown_axis='x'):
        # point_end: end, point_start(tool tip): start
        if unknown_axis == 'y':
            vec_o = point_end - point_start
            vec_o = vec_o / np.linalg.norm(vec_o)
            vec_a = np.cross(point_end - reference_point, point_start - reference_point)
            vec_a = vec_a / np.linalg.norm(vec_a)
            vec_n = np.cross(vec_o, vec_a)
            vec_n = vec_n / np.linalg.norm(vec_n)
        elif unknown_axis == '-y':
            vec_o = point_start - point_end
            vec_o = vec_o / np.linalg.norm(vec_o)
            vec_a = np.cross(point_start- reference_point, point_end - reference_point)
            vec_a = vec_a / np.linalg.norm(vec_a)
            vec_n = np.cross(vec_o, vec_a)
            vec_n = vec_n / np.linalg.norm(vec_n)
        elif unknown_axis == 'x':
            vec_n = point_end - point_start
            vec_n = vec_n / np.linalg.norm(vec_n)
            vec_o = np.cross(point_end - reference_point, point_start - reference_point)
            vec_o = vec_o / np.linalg.norm(vec_o)
            vec_a = np.cross(vec_n, vec_o)
            vec_a = vec_a / np.linalg.norm(vec_a)
        elif unknown_axis == '-x':
            vec_n = point_start - point_end
            vec_n = vec_n / np.linalg.norm(vec_n)
            vec_o = np.cross(point_start - reference_point, point_end - reference_point)
            vec_o = vec_o / np.linalg.norm(vec_o)
            vec_a = np.cross(vec_n, vec_o)
            vec_a = vec_a / np.linalg.norm(vec_a)
        elif unknown_axis == 'z':
            vec_a = point_end - point_start
            vec_a = vec_a / np.linalg.norm(vec_a)
            vec_n = np.cross(point_end - reference_point, point_start - reference_point)
            vec_n = vec_n / np.linalg.norm(vec_n)
            vec_o = np.cross(vec_a, vec_n)
            vec_o = vec_o / np.linalg.norm(vec_o)
        elif unknown_axis == '-z':
            vec_a = point_start - point_end
            vec_a = vec_a / np.linalg.norm(vec_a)
            vec_n = np.cross(point_start - reference_point, point_end - reference_point)
            vec_n = vec_n / np.linalg.norm(vec_n)
            vec_o = np.cross(vec_a, vec_n)
            vec_o = vec_o / np.linalg.norm(vec_o)
        else:
             raise Exception

        R = np.eye(4)
        R[0:3, 0] = vec_n
        R[0:3, 1] = vec_o
        R[0:3, 2] = vec_a
        return R

    @staticmethod
    def create_tip_guide_tool(tool_tip_in_ee: np.ndarray, ee_reference_point=np.array([0, 0, 0]), unknown_axis='x'):
        # point0_in_ee: end, point1_in_ee(tool tip): start
        point1_in_ee = tool_tip_in_ee
        if unknown_axis == 'x':
            point0_in_ee = np.array([1, 0, 0]) + point1_in_ee
        elif unknown_axis == '-x':
            point0_in_ee = point1_in_ee - np.array([1, 0, 0])
        if unknown_axis == 'y':
            point0_in_ee = np.array([0, 1, 0]) + point1_in_ee
        elif unknown_axis == '-y':
            point0_in_ee = point1_in_ee - np.array([0, 1, 0])
        if unknown_axis == 'z':
            point0_in_ee = np.array([0, 0, 1]) + point1_in_ee
        elif unknown_axis == '-z':
            point0_in_ee = point1_in_ee - np.array([0, 0, 1])
        return tf2_helper.create_5D_tool(point0_in_ee, point1_in_ee, ee_reference_point, unknown_axis)

    @staticmethod
    def angleaxis_to_quat(axis_vector, angle):
        # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
        axis_normed = [axis_vector[0] / angle, axis_vector[1] / angle, axis_vector[2] / angle]
        s = np.sin(angle / 2)
        return [s * axis_normed[0], s * axis_normed[1], s * axis_normed[2], np.cos(angle / 2)]  # q = [x y z w]

    @staticmethod
    def rotv_to_quat(rotv):
        angle = np.linalg.norm(rotv)
        return tf2_helper.angleaxis_to_quat(rotv, angle)

    @staticmethod
    def quat_to_angleaxis(q):
        # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Unit_quaternions
        # q = [x y z w]
        angle = 2 * np.atan2(np.linalg.norm([q[0], q[1], q[2]]), q[3])
        if abs(angle) > 1e-6:
            axis_normed = [q[0] / np.sin(angle / 2), q[1] / np.sin(angle / 2), q[2] / np.sin(angle / 2)]
        else:
            axis_normed = 0.0
        return np.array([axis_normed[0], axis_normed[1], axis_normed[2]]), angle

    @staticmethod
    def quat_to_rotv(q):
        axis_vector, angle = tf2_helper.quat_to_angleaxis(q)
        return axis_vector * angle

    @staticmethod
    def vec_rotate_vec(vec, rotation_axis, theta):
        # ref: https://math.stackexchange.com/questions/511370/how-to-rotate-one-vector-about-another
        vec_parallel = (np.dot(vec, rotation_axis) / np.dot(rotation_axis, rotation_axis)) * rotation_axis
        vec_vertical = vec - vec_parallel
        w = np.cross(rotation_axis, vec_vertical)
        x1 = np.cos(theta) / np.linalg.norm(vec_vertical)
        x2 = np.sin(theta) / np.linalg.norm(w)
        vec_vertical_theta = np.linalg.norm(vec_vertical) * (x1 * vec_vertical + x2 * w)
        vec_rotation = vec_vertical_theta + vec_parallel
        return vec_rotation

class medical_robot_helper(tf2_helper):
    def __init__(self, tool_link, tool_axis='x', ee_link='tool0', base_link='base_link', medical_link='medical_link', ndi_link='ndi_link') -> None:
        super(medical_robot_helper, self).__init__()
        self.tool_link = tool_link
        self.base_link = base_link
        self.ee_link = ee_link
        self.medical_link = medical_link
        self.ndi_link = ndi_link
        self.tool_axis = tool_axis

    def quick_plan_robot_tool_tip_3D(self, position, position_link):
        c_tool_rotation = self.matrix_from_transform(self.sample(target_link=self.ee_link, ref_link=self.base_link).transform)[:3, :3]
        g_tool_position = self.transform_pose(position, [0, 0, 0, 1], position_link, to_frame=self.base_link)[0]
        tool2ee = self.matrix_from_transform(self.sample(target_link=self.ee_link, ref_link=self.tool_link).transform)
        pose_msg = self.matrix_to_pose(self.plan_robot_tool_tip_3D(c_tool_rotation, g_tool_position, tool2ee))
        return pose_msg

    def quick_plan_robot_tool_tip_5D(self, position, position_link, enter_vec, enter_vec_link, tool_axis_rotation=None, ee_reference_point=np.array([0, 0, 0])):
        # p0: end, p1: start
        ee2evlink = self.matrix_from_transform(self.sample(ref_link=self.ee_link, target_link=enter_vec_link).transform)
        enter_vec_in_ee = ee2evlink[:3, :3] @ enter_vec
        g_tool_position_in_base = self.transform_pose(position, [0, 0, 0, 1], position_link, to_frame=self.base_link)[0]
        tool2ee = self.matrix_from_transform(self.sample(target_link=self.ee_link, ref_link=self.tool_link).transform)
        ee2tool = self.matrix_from_transform(self.sample(target_link=self.tool_link, ref_link=self.ee_link).transform)
        base2ee = self.matrix_from_transform(self.sample(target_link=self.ee_link, ref_link=self.base_link).transform)
        base2tool = self.matrix_from_transform(self.sample(target_link=self.tool_link, ref_link=self.base_link).transform)

        # init plan
        pose_msg = self.matrix_to_pose(self.plan_robot_tool_tip_5D(enter_vec_in_ee, g_tool_position_in_base, tool2ee, base2ee, ee_reference_point, self.tool_axis))
        
        if tool_axis_rotation is None:
            return pose_msg
        else:
            g_tool_init_matrix = self.matrix_from_pose(pose_msg) @ ee2tool

            delta = np.linalg.pinv(base2tool).dot(g_tool_init_matrix)
            if self.tool_axis == '-x' or self.tool_axis == '-y' or self.tool_axis == '-z':
                tool_axis_rotation = -tool_axis_rotation

            if self.tool_axis == 'x' or self.tool_axis == '-x':
                delta_euler = tf_conversions.transformations.euler_from_matrix(delta, 'ryzx')
                delta_euler = (delta_euler[0], delta_euler[1], tool_axis_rotation)
                new_rot = tf_conversions.transformations.euler_matrix(*delta_euler, 'ryzx')
            elif self.tool_axis == 'y' or self.tool_axis == '-y':
                delta_euler = tf_conversions.transformations.euler_from_matrix(delta, 'rzxy')
                delta_euler = (delta_euler[0], delta_euler[1], tool_axis_rotation)
                new_rot = tf_conversions.transformations.euler_matrix(*delta_euler, 'rzxy')
            elif self.tool_axis == 'z' or self.tool_axis == '-z':
                delta_euler = tf_conversions.transformations.euler_from_matrix(delta, 'rxyz')
                delta_euler = (delta_euler[0], delta_euler[1], tool_axis_rotation)
                new_rot = tf_conversions.transformations.euler_matrix(*delta_euler, 'rxyz')

            new_rot[:, 3:] = delta[:, 3:]
            ret = base2tool @ new_rot @ tool2ee
            return self.matrix_to_pose(ret)

    @staticmethod
    def calc_tool_tip_access(entry_position, depth=None, target_position=None, enter_vec=None):
        assert (target_position is not None) ^ (enter_vec is not None)
        if target_position is not None:
            enter_vec = target_position - entry_position

        if depth is None:
            depth = np.linalg.norm(enter_vec)

        if depth < 0:
            position = entry_position - enter_vec * (-depth / np.linalg.norm(enter_vec))
        else:
            position = entry_position + enter_vec * (depth / np.linalg.norm(enter_vec))
        return position, enter_vec

    def quick_plan_robot_tool_tip_in_medical(self, position, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        if enter_vec is not None:
            return self.quick_plan_robot_tool_tip_5D(
                position, self.medical_link, 
                enter_vec, self.medical_link,
                0,
                ee_reference_point)
        else:
            return self.quick_plan_robot_tool_tip_3D(position, self.medical_link)

    def quick_plan_robot_tool_waypoints_in_medical(self, positions, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        waypoints = []
        for p in positions:
            pose_msg = self.quick_plan_robot_tool_tip_in_medical(p, enter_vec, ee_reference_point)
            waypoints.append(pose_msg)
        return waypoints

    def quick_tool_tip_access_in_medical(self, entry_position, depth=None, target_position=None, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        position, enter_vec = self.calc_tool_tip_access(entry_position, depth, target_position, enter_vec)
        return self.quick_plan_robot_tool_tip_in_medical(position, enter_vec, ee_reference_point)

    def quick_plan_robot_tool_tip_in_ndi(self, position, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        if enter_vec is not None:
            return self.quick_plan_robot_tool_tip_5D(
                position, self.ndi_link, 
                enter_vec, self.ndi_link,
                0,
                ee_reference_point)
        else:
            return self.quick_plan_robot_tool_tip_3D(position, self.ndi_link)

    def quick_tool_tip_access_in_ndi(self, entry_position, depth=None, target_position=None, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        position, enter_vec = self.calc_tool_tip_access(entry_position, depth, target_position, enter_vec)
        return self.quick_plan_robot_tool_tip_in_ndi(position, enter_vec, ee_reference_point)

    def quick_plan_robot_tool_tip_in_base(self, position, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        if enter_vec is not None:
            return self.quick_plan_robot_tool_tip_5D(
                position, self.base_link, 
                enter_vec, self.base_link,
                0,
                ee_reference_point)
        else:
            return self.quick_plan_robot_tool_tip_3D(position, self.ndi_link)

    def quick_tool_tip_access_in_base(self, entry_position, depth=None, target_position=None, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        position, enter_vec = self.calc_tool_tip_access(entry_position, depth, target_position, enter_vec)
        return self.quick_plan_robot_tool_tip_in_base(position, enter_vec, ee_reference_point)
    
    def quick_tool_tip_access_from_current_in_base(self, depth, ee_reference_point=np.array([0, 0, 0])):
        entry_position, enter_vec = self.get_current_rotation_axis_vec(self.base_link)
        return self.quick_tool_tip_access_in_base(entry_position, depth=depth, enter_vec=enter_vec, ee_reference_point=ee_reference_point)

    def quick_plan_robot_tool_waypoints_in_base(self, positions, enter_vec=None, ee_reference_point=np.array([0, 0, 0])):
        waypoints = []
        for p in positions:
            pose_msg = self.quick_plan_robot_tool_tip_in_base(p, enter_vec, ee_reference_point)
            waypoints.append(pose_msg)
        return waypoints
    
    def get_current_rotation_axis_vec(self, ref_link_name):
        curr_transform = self.sample(target_link=self.tool_link, ref_link=ref_link_name).transform
        position = [curr_transform.translation.x, curr_transform.translation.y, curr_transform.translation.z]
        m = self.matrix_from_transform(curr_transform)
        if self.tool_axis == 'x':
            rotation_axis_vec = m[0:3, 0]
        elif self.tool_axis == '-x':
            rotation_axis_vec = -m[0:3, 0]
        elif self.tool_axis == 'y':
            rotation_axis_vec = m[0:3, 1]
        elif self.tool_axis == '-y':
            rotation_axis_vec = -m[0:3, 1]
        elif self.tool_axis == 'z':
            rotation_axis_vec = m[0:3, 2]
        elif self.tool_axis == '-z':
            rotation_axis_vec = -m[0:3, 2]
        return position, rotation_axis_vec