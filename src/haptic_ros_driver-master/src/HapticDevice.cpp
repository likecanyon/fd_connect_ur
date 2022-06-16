#include "haptic_ros_driver/HapticDevice.h"

// haptic device API
#include "haptic_ros_driver/dhdc.h"
#include "std_msgs/Int8MultiArray.h"

HapticDevice::HapticDevice(ros::NodeHandle &node, bool set_force) : loop_rate_(1)
{
    nh_ = node;

    nh_.param<float>("getDataFreq", getDataFreq_, 1000);

    loop_rate_ = ros::Rate(getDataFreq_);

    dev_id_ = -2; // we set a value not in API defined range
    device_enabled_ = -1;

    set_force_ = set_force;

    for (int i = 0; i < 3; i++)
    {
        position_[i] = 0.0;
    }

    button0_state_ = false;
    keep_alive_ = false;
    force_released_ = true;

    force_.resize(3);
    force_[0] = 0.0;
    force_[1] = 0.0;
    force_[2] = 0.0;

    SetForceLimit(10.0, 10.0, 10.0);

    // connect to hardware device
    device_count_ = dhdGetDeviceCount();

    // we only accept one haptic device.
    if (device_count_ >= 1)
    {
        dev_id_ = dhdOpenID(0); // if open failed, we will get -1, and sucess with 0.
        if (dev_id_ < 0)
        {
            ROS_INFO("error: handler device: %s\n", dhdErrorGetLastStr());
            device_enabled_ = false;
            return;
        }
    }
    else
    {
        ROS_INFO("No handler device find! %s\n", dhdErrorGetLastStr());
        device_enabled_ = false;
        return;
    }

    // enable force
    dhdEnableForce(DHD_ON);

    // enable virtual switch
    dhdEmulateButton(DHD_ON);

    dhdSetForceAndGripperForce(0, 0, 0, 0.0);

    device_enabled_ = true;
}

HapticDevice::~HapticDevice()//析构函数的定义
{
    dev_id_ = -1;
    device_count_ = 0;
    keep_alive_ = false;
    if (dev_op_thread_)
    {
        dev_op_thread_->join();
    }
}

void HapticDevice::PublishHapticData()
{
    geometry_msgs::PoseStamped pose;                  //定义位姿时间戳 pose
    pose.header.frame_id = ros::this_node::getName(); // 获取这个节点的名字
    pose.header.stamp = ros::Time::now();             //得到ros::Time实例化的当前时间
    pose.pose.position.x = position_[0];              //将positon_[0]的值赋值给pose.x
    pose.pose.position.y = position_[1];              //将positon_[1]的值赋值给pose.y
    pose.pose.position.z = position_[2];              //将positon_[2]的值赋值给pose.z

    pose.pose.orientation.x = orient_[0]; //将orient_[0]的值赋值给pose.orientation.x
    pose.pose.orientation.y = orient_[1]; //将orient_[1]的值赋值给pose.orientation.y
    pose.pose.orientation.z = orient_[2]; //将orient_[2]的值赋值给pose.orientation.z

    pose.pose.orientation.w = gripperRad_; //四元数的w是gripperRad_

    std_msgs::Int8MultiArray button_stat;
    if (button0_state_)
    {
        button_stat.data.push_back(1); // push_back的作用：在队列后面加入（ ）的元素
        button_stat.data.push_back(1);
    }
    else
    {
        button_stat.data.push_back(0);
        button_stat.data.push_back(0);
    }

    position_pub_.publish(pose);
    button_state_pub_.publish(button_stat);
}

void HapticDevice::RegisterCallback() //发送消息到话题/haptic/button_state，/haptic/force，/haptic/position

{
    position_topic_ = "/haptic/position";
    buttons_topic_ = "/haptic/button_state";
    force_topic_ = "/haptic/force";

    position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(position_topic_.c_str(), 1);
    button_state_pub_ = nh_.advertise<std_msgs::Int8MultiArray>(buttons_topic_.c_str(), 1);
    force_sub_ = nh_.subscribe<geometry_msgs::Vector3>(force_topic_.c_str(), 1, &HapticDevice::ForceCallback, this);
}

void HapticDevice::ForceCallback(const geometry_msgs::Vector3::ConstPtr &data)
{
    // wrapper force
    SetForce(data->x, data->y, data->z);
    // std::cout << data->x <<" "<< data->y <<" "<< data->z <<" "<<std::endl;
    // std::cout <<force_[0]  <<" "<< force_[1] <<" "<< force_[2] <<" "<<std::endl;
    //
}

void HapticDevice::GetHapticDataRun()
{ // get and we will publish immediately

    double feed_force[3] = {0.0, 0.0, 0.0};
    double current_position[3] = {0.0, 0.0, 0.0};
    double current_orientRad[3] = {0.0, 0.0, 0.0};
    double current_gripperRand = 0.0;

    while (ros::ok() && (keep_alive_ == true))
    {

        if (device_count_ >= 1 && dev_id_ >= 0)
        {
            dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
            dhdGetOrientationRad(&current_orientRad[0], &current_orientRad[1], &current_orientRad[2]);
            dhdGetGripperAngleRad(&current_gripperRand);

            position_[0] = current_position[0];
            position_[1] = current_position[1];
            position_[2] = current_position[2];

            orient_[0] = current_orientRad[0];
            orient_[1] = current_orientRad[1];
            orient_[2] = current_orientRad[2];
            gripperRad_ = current_gripperRand;

            button0_state_ = dhdGetButton(0, dev_id_); // DHD_ON if the button is pressed, DHD_OFF otherwise,0 for the gripper button,[default=-1] device ID

            // read user switches and data
            // Return the 32-bit binary mask of the device buttons.
            // Parameters
            //     ID	[default=-1] device ID (see multiple devices section for details)
            // Returns
            //     A 32-bit long bitmask. Each bit is set to 1 if the button is pressed, 0 otherwise.
            // DeviceSwitches = dhdGetButtonMask();
            // ROS_INFO("position is: %d,%d",button0_state_,DeviceSwitches);
            // button0_state_ = cCheckBit(DeviceSwitches, 0);

            // ROS_INFO("position is: %f ,%f ,%f;  orient is: %f ,%f ,%f; gripper is %f;  button is : %d",
            //           position_[0], position_[1], position_[2], orient_[0], orient_[1], orient_[2], gripperRad_, button0_state_);
        }

        PublishHapticData();

        // apply force
        if (set_force_)
        {
            val_lock_.lock();
            feed_force[0] = force_[0];
            feed_force[1] = force_[1];
            feed_force[2] = force_[2];
            dhdSetForceAndGripperForce(feed_force[0], feed_force[1], feed_force[2], 0.0);
            val_lock_.unlock();
        }

        loop_rate_.sleep();
    }
}

void HapticDevice::SetForce(double x, double y, double z) //设定力，
{
    double input_force[3] = {0.0, 0.0, 0.0};
    if (set_force_)
    {
        val_lock_.lock(); // val_lock_的类型为std::mutex HapticDevice::val_lock_,mutex的意思是互斥量，这句话是对线程上锁
        input_force[0] = x;
        input_force[1] = y;
        input_force[2] = z;
        VerifyForceLimit(input_force, force_);
        force_released_ = false;
        val_lock_.unlock(); //解锁
    }
}

void HapticDevice::SetForceLimit(double x, double y, double z) //设定力限制
{
    force_x_limit_ = x;
    force_y_limit_ = y;
    force_z_limit_ = z;
}

void HapticDevice::VerifyForceLimit(double input_force[], std::vector<double> &output) //确认力限制
{
    if (output.size() != 3)
    {
        output.resize(3);
    }
    output[0] = input_force[0];
    output[1] = input_force[1];
    output[2] = input_force[2];
    if (input_force[0] < -force_x_limit_)
        output[0] = -force_x_limit_;
    if (input_force[1] < -force_y_limit_)
        output[1] = -force_y_limit_;
    if (input_force[2] < -force_z_limit_)
        output[2] = -force_z_limit_;

    if (input_force[0] > force_x_limit_)
        output[0] = force_x_limit_;
    if (input_force[1] > force_y_limit_)
        output[1] = force_y_limit_;
    if (input_force[2] > force_z_limit_)
        output[2] = force_z_limit_;
}

void HapticDevice::Start()
{
    if (!device_enabled_)
    {
        return;
    }

    RegisterCallback();
    dev_op_thread_ = std::make_shared<boost::thread>(boost::bind(&HapticDevice::GetHapticDataRun, this));
    keep_alive_ = true;
}
