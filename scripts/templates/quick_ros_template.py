#!/usr/bin/env python3
import rospy
import math
from ros_helper import set_debug_output, create_service_client
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool


class ros_package:
    def __init__(self) -> None:
        self._tag = self.__class__.__name__
        rospy.init_node('xxx_node')
        set_debug_output('DEBUG_OUTPUT')

        self._from_rosparam = rospy.get_param(f"{rospy.get_name()}/parent_key/child_key")

        self._param1 = rospy.get_param(f"{rospy.get_name()}/param1_key")
        self._param2 = int(rospy.get_param(f"{rospy.get_name()}/param2_key"))
        loop_update_Rate = int(rospy.get_param(f"{rospy.get_name()}/loop_update_Rate_key"))

        rospy.logdebug(f"{self._tag} - __init__()")

        # publisher
        self._pub = rospy.Publisher(self._tag, String, queue_size=10)

        # subscriber
        self._sub = rospy.Subscriber(f"{rospy.get_namespace()}topic/name", Float32, self._callback_for_sub)

        # service client
        topic = f"{rospy.get_namespace()}service/topic/name"
        create_service_client(topic, SetBool)

        rospy.on_shutdown(self._shutdown)
        self._loop_rate = rospy.Rate(loop_update_Rate)

    def _callback_for_sub(self, data):
        rospy.loginfo(data.data)

    def _shutdown(self):
        rospy.logdebug(f"{self._tag} - _shutdown()")
        self._pub.unregister()
        # TODO

    def execute(self):
        # TODO
        while not rospy.is_shutdown():
            try:
                msg = String()
                msg.data = 'Hello World!'
                self._pub.publish(msg)
            except Exception as e:
                rospy.logerr(f"{self._tag} - Error: {e}")
            self._loop_rate.sleep()
        # or just using
        # rospy.spin()


if __name__ == '__main__':
    node = ros_package()
    node.execute()
