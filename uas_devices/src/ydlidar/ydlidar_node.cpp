#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "src/CYdLidar.h"
#include "std_srvs/srv/empty.hpp"

namespace uas_devices
{

class YDLidarNode : public rclcpp::Node
{
public:
YDLidarNode(const rclcpp::NodeOptions & options) : Node("ydlidar", options)
  {
    /// lidar port
    std::string str_optvalue = "/dev/ttyUSB0";
    declare_parameter("port", str_optvalue);
    get_parameter("port", str_optvalue);
    lidar.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

    /// ignore array
    str_optvalue = "";
    declare_parameter("ignore_array", str_optvalue);
    get_parameter("ignore_array", str_optvalue);
    lidar.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

    std::string frame_id = "base_lidar";
    declare_parameter("frame_id", frame_id);
    get_parameter("frame_id", frame_id);

    //////////////////////int property/////////////////
    /// lidar baudrate
    int optval = 230400;
    declare_parameter("baudrate", optval);
    get_parameter("baudrate", optval);
    lidar.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
    /// tof lidar
    optval = TYPE_TRIANGLE;
    declare_parameter("lidar_type", optval);
    get_parameter("lidar_type", optval);
    lidar.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    declare_parameter("device_type", optval);
    get_parameter("device_type", optval);
    lidar.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = 4;
    declare_parameter("sample_rate", optval);
    get_parameter("sample_rate", optval);
    lidar.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 4;
    declare_parameter("abnormal_check_count", optval);
    get_parameter("abnormal_check_count", optval);
    lidar.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    /// Intenstiy bit count
    optval = 8;
    declare_parameter("intensity_bit", optval);
    get_parameter("intensity_bit", optval);
    lidar.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = true;
    declare_parameter("fixed_resolution", b_optvalue);
    get_parameter("fixed_resolution", b_optvalue);
    lidar.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    b_optvalue = true;
    declare_parameter("reversion", b_optvalue);
    get_parameter("reversion", b_optvalue);
    lidar.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    b_optvalue = true;
    declare_parameter("inverted", b_optvalue);
    get_parameter("inverted", b_optvalue);
    lidar.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    declare_parameter("auto_reconnect", b_optvalue);
    get_parameter("auto_reconnect", b_optvalue);
    lidar.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    b_optvalue = false;
    declare_parameter("isSingleChannel", b_optvalue);
    get_parameter("isSingleChannel", b_optvalue);
    lidar.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    /// intensity
    b_optvalue = true;
    declare_parameter("intensity", b_optvalue);
    get_parameter("intensity", b_optvalue);
    lidar.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = false;
    declare_parameter("support_motor_dtr", b_optvalue);
    get_parameter("support_motor_dtr", b_optvalue);
    lidar.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    declare_parameter("angle_max", f_optvalue);
    get_parameter("angle_max", f_optvalue);
    lidar.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    declare_parameter("angle_min", f_optvalue);
    get_parameter("angle_min", f_optvalue);
    lidar.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 12.f;
    declare_parameter("range_max", f_optvalue);
    get_parameter("range_max", f_optvalue);
    lidar.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.03f;
    declare_parameter("range_min", f_optvalue);
    get_parameter("range_min", f_optvalue);
    lidar.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    f_optvalue = 10.f;
    declare_parameter("frequency", f_optvalue);
    get_parameter("frequency", f_optvalue);
    lidar.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    bool invalid_range_is_inf = false;
    declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
    get_parameter("invalid_range_is_inf", invalid_range_is_inf);

    bool ret = lidar.initialize();
    if (ret) {
      ret = lidar.turnOn();
    } else {
      RCLCPP_ERROR(get_logger(), "%s\n", lidar.DescribeError());
    }
  }

private:
  CYdLidar lidar;
};

}  // namespace uas_devices

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uas_devices::YDLidarNode)