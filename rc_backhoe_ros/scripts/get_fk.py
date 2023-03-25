#!/usr/bin/env python

# https://github.com/uts-magic-lab/moveit_python_tools/blob/master/src/moveit_python_tools/get_fk.py

import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
import tf

"""
Class to make FK calls using the /compute_fk service.
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GetFK(object):
    def __init__(self, fk_link, frame_id):
        """
        A class to do FK calls thru the MoveIt!'s /compute_ik service.
        :param str fk_link: link to compute the forward kinematics
        :param str frame_id: frame_id to compute the forward kinematics
        into account collisions
        """
        rospy.loginfo("Initalizing GetFK...")
        self.fk_link = fk_link
        self.frame_id = frame_id
        rospy.loginfo("Asking forward kinematics for link: " + self.fk_link)
        rospy.loginfo("PoseStamped answers will be on frame: " + self.frame_id)
        self.fk_srv = rospy.ServiceProxy('/compute_fk',
                                         GetPositionFK)
        rospy.loginfo("Waiting for /compute_fk service...")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Connected!")
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

    def js_cb(self, data):
        self.last_js = data

    def get_current_fk_pose(self):
        resp = self.get_current_fk()
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None

    def get_current_fk(self, l1, l2, l3, l4):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.logwarn("Waiting for a /joint_states message...")
            rospy.sleep(0.1)
        print(self.last_js.position)
        self.last_js.position = (l1, l2, l3, l4)
        return self.get_fk(self.last_js)

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        """
        Do an FK call to with.
        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        if fk_link is None:
            fk_link = self.fk_link

        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [self.fk_link]
        # joint_state.position[0] += 0.5
        # joint_state.position[1] += 0.5
        # joint_state.position[2] += 0.5
        # joint_state.position[3] += 0.5
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp

pub_marker = rospy.Publisher("/marker", MarkerArray, queue_size=10)

def publish_cube_marker(pos, rot, color=[1,0,0]):
    marker_data_array = MarkerArray()
    id = 0
    for i in range(len(pos)):
        marker_data = Marker()
        marker_data.header.frame_id = "base_link"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "my_name_space"
        marker_data.id = id
        marker_data.action = Marker.ADD
        marker_data.type = Marker.ARROW
        marker_data.pose.position.x = pos[i].x
        marker_data.pose.position.y = pos[i].y
        marker_data.pose.position.z = pos[i].z
        marker_data.pose.orientation.x = rot[i].x
        marker_data.pose.orientation.y = rot[i].y
        marker_data.pose.orientation.z = rot[i].z
        marker_data.pose.orientation.w = rot[i].w
        marker_data.color.r = color[0]
        marker_data.color.g = color[1]
        marker_data.color.b = color[2]
        marker_data.color.a = 0.5
        marker_data.scale.x = 0.3
        marker_data.scale.y = 0.3
        marker_data.scale.z = 0.3
        # marker_data.lifetime = rospy.Duration(100.0)
        marker_data.type = 0
        marker_data_array.markers.append(marker_data)
        id+=1
    pub_marker.publish(marker_data_array)

if __name__ == '__main__':
    import numpy as np
    rospy.init_node('test_fk')
    rospy.loginfo("Querying for FK")
    gfk = GetFK('target_link', 'base_link')
    # a = 0
    pos_list = []
    rot_list = []
    for i in np.arange(-0.5,0.51,0.2):
        for j in np.arange(0.0,1.51,0.3):
            for k in np.arange(0.0,1.51,0.3):
                for l in np.arange(0.0,1.51,0.3):
                    resp = gfk.get_current_fk(i, j, k, l)
                    print(i)
                    # print(resp)
                    # print(resp.pose_stamped)
                    # print(resp.pose_stamped[0])
                    # print(resp.pose_stamped[0].pose.position)
                    # print(resp.pose_stamped[0].pose.orientation)
                    pos_list.append(resp.pose_stamped[0].pose.position)
                    rot_list.append(resp.pose_stamped[0].pose.orientation)
    publish_cube_marker(pos_list, rot_list)
    rospy.sleep(1.0)
    # from moveit_python_tools.friendly_error_codes import moveit_error_dict
    # rospy.loginfo(moveit_error_dict[resp.error_code.val])
    # rospy.loginfo(resp)