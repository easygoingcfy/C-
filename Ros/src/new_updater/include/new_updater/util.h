#include <ros/ros.h>

/**
 * @brief 从参数服务器获取参数值，如果获取失败，使用传入的默认值设置参数值
 * 
 * @tparam T 
 * @param param_name 
 * @param nh 
 * @param [in] param_value 
 * @param default_value 
 */
template<typename T>
void getRosParam(const std::string &param_name, ros::NodeHandle& nh, T& param_value, const T& default_value) {
    if (!nh.getParam(param_name, param_value)) {
        ROS_WARN_STREAM("Failed to get parameter '" << param_name << "', using default value: " << default_value);
        param_value = default_value;
    }
}