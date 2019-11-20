import rospy
import ros_numpy as rnp
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_conjugate
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Point
from typing import Tuple

from util import PublisherValue


__all__ = ['ARTag', 'ARCube']



def qv_mult(q1, v1):
    # https: // answers.ros.org / question / 196149 / how - to - rotate - vector - by - quaternion - in -python /
    q2 = list(v1) + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[:3]


def pose_with_offset(pose, offset):  # type: (PoseStamped, Tuple[float, float, float]) -> PoseStamped
    offset_pose = PoseStamped()
    offset_pose.header.frame_id = pose.header.frame_id

    offset = qv_mult(rnp.numpify(pose.pose.orientation), offset)
    offset_pose.pose.position = rnp.msgify(Point, rnp.numpify(pose.pose.position) + offset)
    offset_pose.pose.orientation = pose.pose.orientation

    return offset_pose


class ARTag(object):
    def __init__(self, number, side_length, alvar_topic='/ar_pose_marker', visual_topic=None):  # type: (int, float, str) -> None
        self.number = number
        self.side_length = side_length
        self.pose = None
        self.subscriber = rospy.Subscriber(alvar_topic, AlvarMarkers, self._ar_alvar_callback)
        if visual_topic is not None:
            self.visual_publisher = PublisherValue(visual_topic, PoseStamped, 10, lambda: self.pose)

    def _ar_alvar_callback(self, msg):  # type: (AlvarMarkers) -> None
        for m in msg.markers:
            if m.id == self.number:
                self._set_pose(m.pose)

    def _set_pose(self, pose):  # type: (PoseStamped) -> None
        pose = pose_with_offset(pose, self._offset())
        pose.header.frame_id = 'odom'
        self.pose = pose

    def get_pose_with_offset(self, offset):  # type: (Tuple[float, float, float]) -> PoseStamped
        return pose_with_offset(self.pose, offset)

    def _offset(self): # type: ()-> Tuple[float, float, float]
        return (0., 0., 0.)


class ARCube(object):
    def __init__(self, number, tag_side_length, cube_side_length, alvar_topic='/ar_pose_marker', visual_topic=None):
        self.number = number
        self.tag_side_length = tag_side_length
        self.cube_side_length = cube_side_length
        self.pose = None
        self.subscriber = rospy.Subscriber(alvar_topic, AlvarMarkers, self._ar_alvar_callback)
        if visual_topic is not None:
            self.visual_publisher = PublisherValue(visual_topic, PoseStamped, 10, lambda: self.pose)


    def _ar_alvar_callback(self, msg):  # type: (AlvarMarkers) -> None
        poses = [m.pose for m in msg.markers if m.id == self.number]
        if len(poses) == 0:
            return

        poses = [pose_with_offset(pose, self._offset()) for pose in poses]
        center_point = np.mean([rnp.numpify(pose.pose.position) for pose in poses], axis=0)

        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position = rnp.msgify(Point, center_point)
        pose.pose.orientation.w = 1
        self._set_pose(pose)

    def _set_pose(self, pose):  # type: (PoseStamped) -> None
        self.pose = pose

    def _offset(self): # type: ()-> Tuple[float, float, float]
        return (0, 0, -self.cube_side_length/2)


if __name__ == '__main__':
    rospy.init_node('ar_tag_demo')

    cube = ARCube(2, tag_side_length=0.204, cube_side_length=0.32,
                  alvar_topic='/ar_pose_marker', visual_topic='/cube_viz')

    tag = ARTag(20, side_length=0.204,
                alvar_topic='/ar_pose_marker', visual_topic='/tag_viz')

    rospy.spin()
