#include <iostream>
#include <functional>
#include <string>
// #include <deque>

#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class RcBackhoeRetrofitModel : public ModelPlugin {
public:

    RcBackhoeRetrofitModel()
    {
    }

    ~RcBackhoeRetrofitModel()
    {
        delete pnh;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&RcBackhoeRetrofitModel::OnUpdate, this));

        if (!ros::isInitialized())
        {
            return;
        }

        pnh = new ros::NodeHandle("~");

        ROS_INFO("RcBackhoe retrofit controller loaded");

        swing_joint = model->GetJoint("swing_joint");
        boom_joint = model->GetJoint("boom_joint");
        arm_joint = model->GetJoint("arm_joint");
        bucket_joint = model->GetJoint("bucket_joint");

        if(!swing_joint) ROS_ERROR("Loading Error swing joint");
        if(!boom_joint) ROS_ERROR("Loading Error boom link");
        if(!arm_joint) ROS_ERROR("Loading Error arm joint");
        if(!bucket_joint) ROS_ERROR("Loading Error bucket joint");

        ROS_INFO("Joint registered");

        std::string sub_topic = _sdf->Get<std::string>("sub_topic", "/rc_backhoe_command").first;
        sub_joy = pnh->subscribe("/rc_backhoe/cmd/actuator", 1, &RcBackhoeRetrofitModel::cb_joy, this);

        ROS_INFO("Initialize!!");
        Reset();
    }

    void OnUpdate()
    {
        // // シミュレーション内部の時間を取得
        common::Time now = model->GetWorld()->SimTime();

        this->swing_joint->SetVelocity(0, swing_cmd);
        this->boom_joint->SetVelocity(0, boom_cmd);
        this->arm_joint->SetVelocity(0, arm_cmd);
        this->bucket_joint->SetVelocity(0, bucket_cmd);

    }

    void Reset(){
        // ROS_INFO("Gazebo initialize");
        // this->time_prev = model->GetWorld()->SimTime();
        // this->st = model->GetWorld()->SimTime().Double();
        // shift_status.status = wl_status_msgs::ShiftStatus::NUETRAL;
        // // shift_status.status = wl_status_msgs::ShiftStatus::FORWARD;
        // records_for_accel_pedal_delay.resize(accel_pedal_delay);
        // records_for_brake_pedal_delay.resize(brake_pedal_delay);
        // records_for_handle_delay.resize(handle_delay);
        // records_for_arm_lever_delay.resize(arm_lever_delay);
        // records_for_bucket_lever_delay.resize(bucket_lever_delay);
        // records_for_wheel_delay.resize(wheel_delay);
        // records_for_steering_delay.resize(steering_delay);
        // records_for_arm_delay.resize(arm_delay);
        // records_for_bucket_delay.resize(bucket_delay);
    }

    void cb_joy(const std_msgs::Int8MultiArray::ConstPtr& msg){
        // swing
        if (msg->data[0] == 1){ swing_cmd = -0.4; }
        else if(msg->data[0] == -1){ swing_cmd = 0.4; }
        else if(msg->data[0] == 0){ swing_cmd = 0.0; }
        else {ROS_ERROR("Not registered command : %d", msg->data[0]);}
        // boom
        if (msg->data[1] == 1){ boom_cmd = -0.4; }
        else if(msg->data[1] == -1){ boom_cmd = 0.4; }
        else if(msg->data[1] == 0){ boom_cmd = 0.0; }
        else {ROS_ERROR("Not registered command : %d", msg->data[1]);}
        // arm
        if (msg->data[2] == 1){ arm_cmd = -0.4; }
        else if(msg->data[2] == -1){ arm_cmd = 0.4; }
        else if(msg->data[2] == 0){ arm_cmd = 0.0; }
        else {ROS_ERROR("Not registered command : %d", msg->data[2]);}
        // bucket
        if (msg->data[3] == 1){ bucket_cmd = -0.4; }
        else if(msg->data[3] == -1){ bucket_cmd = 0.4; }
        else if(msg->data[3] == 0){ bucket_cmd = 0.0; }
        else {ROS_ERROR("Not registered command : %d", msg->data[3]);}
    }

private:
    ros::NodeHandle* pnh;

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    physics::JointPtr swing_joint, boom_joint, arm_joint, bucket_joint;

    common::Time time_prev;
    common::Time pub_time_prev;

    ros::Subscriber sub_joy;

    double swing_cmd = 0.0;
    double boom_cmd = 0.0;
    double arm_cmd = 0.0;
    double bucket_cmd = 0.0;

};

GZ_REGISTER_MODEL_PLUGIN(RcBackhoeRetrofitModel)
}
