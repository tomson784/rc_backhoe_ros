#include <ros/ros.h>
// #include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
// #include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <string>
#include <math.h>
#include <vector>

geometry_msgs::TransformStamped bucket_end_pose_from_base;

int main(int argc, char** argv){
    ros::init(argc, argv, "bucket_end_pose_from_base");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double hz;
	pnh.param<double>("hz", hz, 30.0);

    ros::Rate loop_rate(hz);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok()) {
        try{
            bucket_end_pose_from_base = tfBuffer.lookupTransform(
                "base_link", "target_link", ros::Time(0)
                );

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO("position x:%f, y:%f, z:%f", 
                bucket_end_pose_from_base.transform.translation.x, 
                bucket_end_pose_from_base.transform.translation.y, 
                bucket_end_pose_from_base.transform.translation.z);
        ROS_INFO("rotation x:%f, y:%f, z:%f, w:%f", 
                bucket_end_pose_from_base.transform.rotation.x, 
                bucket_end_pose_from_base.transform.rotation.y, 
                bucket_end_pose_from_base.transform.rotation.z,
                bucket_end_pose_from_base.transform.rotation.w);
        ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}
