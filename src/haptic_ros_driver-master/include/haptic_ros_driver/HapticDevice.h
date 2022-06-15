#ifndef HAPTIC_DEVICE_H__
#define HAPTIC_DEVICE_H__

#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <boost/thread/thread.hpp>
#include <mutex>
#include <vector>

//==============================================================================
/*!
    \brief
    This function checks if a given bit is enabled.

    \details
    This function checks if a given bit is enabled. The function takes by
    argument an unsigned integer \p a_value and the position of the bit
    \p a_bitPosition to be tested. \n

    If the selected bit is enabled, the function returns __true__, otherwise __false__.

    \param  a_value        Unsigned int value.
    \param  a_bitPosition  Bit position [0..31].

    \return __true__ if the selected bit is enabled, __false__ otherwise.
*/
//==============================================================================
inline bool cCheckBit(const unsigned int &a_value, const unsigned int &a_bitPosition)
{
    if ((a_value & (1 << a_bitPosition)) > 0)
    {
        return (true);
    }
    else
    {
        return (false);
    }
}

class HapticDevice
{
public:
    HapticDevice(ros::NodeHandle &node, bool set_force);
    virtual ~HapticDevice(); //虚拟函数 析构函数

    void PublishHapticData(); //发送主手数据
    void RegisterCallback();

    void GetHapticDataRun();

    void SetForce(double x, double y, double z);

    void SetForceLimit(double x, double y, double z);
    void VerifyForceLimit(double input_force[], std::vector<double> &output);
    void ForceCallback(const geometry_msgs::Vector3::ConstPtr &data);

    void Start();
    bool keep_alive_ = false;

protected:
    std::shared_ptr<boost::thread> dev_op_thread_;

private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    int device_count_;
    int dev_id_;
    bool set_force_;

    bool device_enabled_ = false;
    std::mutex val_lock_;
    bool force_released_;
    bool button0_state_ = false;
    bool button1_state_ = false;

    ros::Publisher position_pub_;     //初始化发布者，position_pub_，发布位置信息
    ros::Publisher button_state_pub_; //初始化发布者 button_state_pub_，发布按钮信息
    ros::Subscriber force_sub_;       //初始化订阅者，订阅力信息
    double position_[3];              //位置数组
    double orient_[3];                //姿态数组
    double gripperRad_;               //

    uint DeviceSwitches;

    std::string position_topic_;
    std::string buttons_topic_;
    std::string force_topic_;

    double force_x_limit_;      // x方向力限值
    double force_y_limit_;      // y方向力限值
    double force_z_limit_;      // z方向力限值
    std::vector<double> force_; //初始化力数组

    float getDataFreq_;
};

#endif
