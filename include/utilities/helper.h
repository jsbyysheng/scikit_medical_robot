//
// Created by riseyoung on 2021/12/3.
//

#ifndef ROS_HELPER_H
#define ROS_HELPER_H

namespace ros_utilities {
    class helper {
    public:
        static bool startsWith(std::string s, std::string sub) {
            return s.find(sub) == 0;
        }

        static bool endsWith(std::string s, std::string sub) {
            return s.rfind(sub) == (s.length() - sub.length());
        }

        static std::string create_topic_name(const std::string& topic) {
            if (!startsWith(topic, "/")) {
                return ros::this_node::getName() + "/" + topic;
            }
            else {
                return ros::this_node::getName() + topic;
            }
        }

        template<typename T>
        static void loadParam(const ros::NodeHandle& nodeHandle, const std::string& key, T& i) {
            if (!nodeHandle.getParam(key, i)) {
                ROS_ERROR_STREAM("param " << nodeHandle.getNamespace() << "/" << key << " cannot be loaded.");
                ros::shutdown();
            }
            std::ostringstream oss;
            oss << i;
            ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
        }

        template<typename T>
        static void loadParam(const ros::NodeHandle& nodeHandle, const std::string& key, std::vector<T>& i) {
            if (!nodeHandle.getParam(key, i)) {
                ROS_ERROR_STREAM("param " << nodeHandle.getNamespace() << "/" << key << " cannot be loaded.");
                ros::shutdown();
            }
            std::ostringstream oss;
            oss << "[ ";
            for (auto item : i) {
                oss << item << " ";
            }
            oss << "]";
            ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
        }

        template<typename T>
        static void loadParam(const ros::NodeHandle& nodeHandle, const std::string& key, std::map<std::string, T>& i) {
            if (!nodeHandle.getParam(key, i)) {
                ROS_ERROR_STREAM("param " << nodeHandle.getNamespace() << "/" << key << " cannot be loaded.");
                ros::shutdown();
            }
            std::ostringstream oss;
            oss << "[ ";
            for (const auto& kv : i) {
                oss << kv.first << " has value " << kv.second;
            }
            oss << "]";
            ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
        }

        template<typename T>
        static void loadParam(const ros::NodeHandle& nodeHandle, const std::string& key, T& i, const T& defaultVal) {
            nodeHandle.param(key, i, defaultVal);
            std::ostringstream oss;
            oss << i;
            ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
        }

        static void control_debug_output(const ros::NodeHandle& nodeHandle) {
            bool debug_output;
            loadParam(nodeHandle, "DEBUG_OUTPUT", debug_output, false);
            ROS_INFO_STREAM(debug_output << std::endl);
            if (debug_output) {
                ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
            }
            else {
                ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
            }
        }
    };
}

#endif //ROS_HELPER_H
