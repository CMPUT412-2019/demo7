from math import atan2

import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from ros_numpy import numpify, msgify
from smach_ros import IntrospectionServer
from smach import State, StateMachine, Sequence
import tf
from tf import transformations
from tf import TransformListener


import numpy as np
from typing import Callable

from ar_tag import ARTag, ARCube, pose_with_offset
from util import SubscriberValue, angle_diff, qv_mult


class FinishedListener:
    def __init__(self):
        self.odom_listener = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.result = None

    def init(self):
        self.result = None

    def odom_callback(self, msg):  # type: (Odometry) -> None
        if msg.pose.pose.position.x < -0.5:
            self.result = 'finished'

    def __call__(self):
        return self.result

    @property
    def outcomes(self):
        return 'finished',


class BumperListener:
    def __init__(self):
        self.bumper_listener = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.result = None

    def init(self):
        self.result = None

    def bumper_callback(self, msg):  # type: (BumperEvent) -> None
        if msg.state == msg.PRESSED and msg.bumper == msg.LEFT:
            self.result = 'bumper_left'
        if msg.state == msg.PRESSED and msg.bumper == msg.RIGHT:
            self.result = 'bumper_right'

    def __call__(self):
        return self.result

    @property
    def outcomes(self):
        return 'bumper_left', 'bumper_right'


class DebouncedButton:
    def __init__(self):
        self._pressed = False
        self._debounced_pressed = False
        self.last_released_time = rospy.get_time()
        self.last_pressed_time = rospy.get_time()

    def press(self):
        self._pressed = True
        self.last_pressed_time = rospy.get_time()

    def release(self):
        self._pressed = False
        self.last_released_time = rospy.get_time()

    @property
    def pressed(self):
        time_since_press = rospy.get_time() - self.last_pressed_time
        time_since_release = rospy.get_time() - self.last_released_time
        if self._pressed == self._debounced_pressed:
            return self._pressed
        if self._pressed and time_since_press > 0.05:
            self._debounced_pressed = self._pressed
        if not self._pressed and time_since_release > 0.05:
            self._debounced_pressed = self._pressed
        return self._debounced_pressed


class NoBumperListener:
    def __init__(self):
        self.bumper_listener = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.result = None
        self.left = DebouncedButton()
        self.right = DebouncedButton()
        self.center = DebouncedButton()
        self.previously_pressed = False

    def init(self):
        self.left = DebouncedButton()
        self.right = DebouncedButton()
        self.center = DebouncedButton()
        self.previously_pressed = False
        self.result = None

    def bumper_callback(self, msg):  # type: (BumperEvent) -> None
        if msg.bumper == msg.LEFT:
            self.left.press() if msg.state == msg.PRESSED else self.left.release()
        if msg.bumper == msg.RIGHT:
            self.right.press() if msg.state == msg.PRESSED else self.right.release()
        if msg.bumper == msg.CENTER:
            self.center.press() if msg.state == msg.PRESSED else self.center.release()

    def __call__(self):
        print(self.left.pressed)
        if self.left.pressed or self.right.pressed or self.center.pressed:
            self.previously_pressed = True
        elif self.previously_pressed:
            if not (self.left.pressed or self.right.pressed or self.center.pressed):
                self.result = 'released'
        return self.result

    @property
    def outcomes(self):
        return 'released',


class CombinedListener:
    def __init__(self, listeners):
        self.listeners = listeners

    def init(self):
        for l in self.listeners:
            l.init()

    def __call__(self):
        result = None
        for l in self.listeners:
            result = l()
            if result is not None:
                break
        return result

    @property
    def outcomes(self):
        outcomes = tuple()
        for l in self.listeners:
            outcomes = outcomes + l.outcomes
        return outcomes


class TimerListener:
    def __init__(self, duration):
        self.duration = duration
        self.start_time = None

    def init(self):
        self.start_time = rospy.get_time()

    def __call__(self):
        if self.start_time + self.duration < rospy.get_time():
            return 'timeout'

    @property
    def outcomes(self):
        return 'timeout',


class FindMarkerListener:
    def __init__(self, marker_tracker):  # type: (MarkerTracker) -> None
        self.marker_tracker = marker_tracker

    def init(self):
        pass

    def __call__(self):
        if self.marker_tracker.get_visible_markers():
            return 'found'

    @property
    def outcomes(self):
        return 'found',


class MoveForwardAction:
    def __init__(self, v):
        self.v = v
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    def init(self):
        pass

    def __call__(self):
        t = Twist()
        t.linear.x = self.v
        self.twist_pub.publish(t)


class TurnAction:
    def __init__(self, w):
        self.w = w
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    def init(self):
        pass

    def __call__(self):
        t = Twist()
        t.angular.z = self.w
        self.twist_pub.publish(t)


class ActionUntil(State):
    def __init__(self, action, until, rate=10):
        super(ActionUntil, self).__init__(outcomes=until.outcomes)
        self.action = action
        self.until = until
        self.rate = rospy.Rate(rate)

    def execute(self, ud):
        self.action.init()
        self.until.init()

        while not rospy.is_shutdown():
            self.action()

            result = self.until()
            if result is not None:
                return result

            self.rate.sleep()


class FindTags(State):
    def __init__(self, tags, rotate_speed=0.5):
        super(FindTags, self).__init__(outcomes=['ok'])
        self.tags = tags
        self.rotate_speed = rotate_speed
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    def execute(self, ud):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self._found_all_tags():
            t = Twist()
            t.angular.z = self.rotate_speed
            self.twist_pub.publish(t)
            rate.sleep()
        return 'ok'

    def _found_all_tags(self):
        for tag in self.tags:
            if tag.pose is None:
                return False
        return True


class NavigateToGoalState(State):
    def __init__(self):
        super(NavigateToGoalState, self).__init__(outcomes=['ok', 'err'], input_keys=['target_pose'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        pose = ud.target_pose  # type: PoseStamped
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.orientation
        self.client.send_goal(goal)
        if self.client.wait_for_result():
            return 'ok'
        else:
            return 'err'


class PushToGoalState(State):
    def __init__(self, target, v):  # type: (Callable[[], PoseStamped], float) -> None
        super(PushToGoalState, self).__init__(outcomes=['ok'], input_keys=['target_pose'])
        self.v = v
        self.target = target
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()

    def execute(self, ud):
        while True:
            target_pose = self.target()

            try:
                this_pose = PoseStamped()
                this_pose.header.frame_id = 'odom'
                this_pose.pose = self.odometry.value.pose.pose
                this_pose = self.tf_listener.transformPose('map', this_pose)
            except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
                continue

            cube_offset = 0.17 + 0.32  # TODO: remove magic numbers
            this_position = numpify(this_pose.pose.position)[0:2]
            cube_position = this_position + qv_mult(numpify(this_pose.pose.orientation), [1, 0, 0])[0:2] * cube_offset
            target_position = numpify(target_pose.pose.position)[0:2]
            print(target_position - this_position, target_position - cube_position)
            if (np.dot(target_position - this_position, target_position - cube_position)) <= 0:
                return 'ok'

            target_angle = np.arctan2(target_pose.pose.position.y - this_pose.pose.position.y, target_pose.pose.position.x - this_pose.pose.position.x)
            this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))

            t = Twist()
            t.linear.x = self.v
            t.angular.z = - angle_diff(this_angle, target_angle)
            self.twist_pub.publish(t)


def normal_to_quaternion(normal):  # type: (np.ndarray) -> np.ndarray
    angle = np.arctan2(normal[1], normal[0])
    return transformations.quaternion_about_axis(angle, [0., 0., 1.])


def half_way_target(marker, cube):  # type: (ARTag, ARCube) -> Callable[[], PoseStamped]
    def inner():
        normal = marker.surface_normal
        r_cm = numpify(cube.pose.pose.position) - numpify(marker.pose.pose.position)
        r_cm_parallel = normal * np.dot(normal, r_cm)
        goal_position = r_cm_parallel + numpify(marker.pose.pose.position)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position = msgify(Point, goal_position)
        goal.pose.orientation = msgify(Quaternion, normal_to_quaternion(-r_cm_parallel))
        return goal

    return inner


def parking_square_target(marker, offset):  # type: (ARTag, float) -> Callable[[], PoseStamped]
    def inner():
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position = marker.get_pose_with_offset([0., 0., offset]).pose.position
        goal.pose.orientation = msgify(Quaternion, normal_to_quaternion(-marker.surface_normal))
        return goal

    return inner


class ChooseNewNavGoalState(State):
    def __init__(self, target, cube, back_distance):  # type: (Callable[[], PoseStamped], ARCube, float) -> None
        super(ChooseNewNavGoalState, self).__init__(outcomes=['ok'], input_keys=['marker_pose'], output_keys=['target_pose'])
        self.target = target
        self.cube = cube
        self.back_distance = back_distance

    def execute(self, ud):
        cube_pose = self.cube.pose  # type: PoseStamped
        cube_position = np.array([cube_pose.pose.position.x, cube_pose.pose.position.y])

        tag_pose = self.target()
        tag_position = np.array([tag_pose.pose.position.x, tag_pose.pose.position.y])

        r_mo = transformations.unit_vector(cube_position - tag_position)
        goal_position = cube_position + self.back_distance * r_mo

        e_mo = transformations.unit_vector(np.append(r_mo, [0]))
        orientation_facing_marker = np.eye(4)
        orientation_facing_marker[0:3, 0:3] = np.column_stack((-e_mo, np.cross([0, 0, 1], -e_mo), [0, 0, 1]))
        orientation_facing_marker = transformations.quaternion_from_matrix(orientation_facing_marker)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = cube_pose.header.frame_id
        goal_pose.pose.position = Point(goal_position[0], goal_position[1], 0)
        goal_pose.pose.orientation = Quaternion(
            orientation_facing_marker[0],
            orientation_facing_marker[1],
            orientation_facing_marker[2],
            orientation_facing_marker[3],
        )
        ud.target_pose = goal_pose
        return 'ok'


class StraightenCubeState(State):
    def __init__(self, cube, tag, v, w):  # type: (ARCube, ARTag, float, float) -> None
        super(StraightenCubeState, self).__init__(outcomes=['ok', 'err'])
        self.cube = cube
        self.tag = tag
        self.v = v
        self.w = w
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.odometry = SubscriberValue('/odom', Odometry)
        self.tf_listener = TransformListener()
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, ud):
        rate = rospy.Rate(10)

        print('Straighten: moving to start')
        target_pose = pose_with_offset(self.cube.pose, (0, 0, 0.5))
        angle_to_cube = np.arctan2(self.cube.pose.pose.position.y - target_pose.pose.position.y,
                                   self.cube.pose.pose.position.x - target_pose.pose.position.x)
        target_pose.pose.orientation = msgify(Quaternion, transformations.quaternion_about_axis(angle_to_cube, [0, 0, 1]))

        if not self._move_to(target_pose):
            return 'err'

        print('Straighten: moving forward')
        start_time = rospy.get_time()
        duration = (0.5 - self.cube.cube_side_length/2) / self.v
        while rospy.get_time() - start_time < duration:
            t = Twist()
            t.linear.x = self.v
            self.twist_pub.publish(t)
            rate.sleep()

        print('Straighten: turning')
        while not self._is_cube_straight():
            t = Twist()
            t.linear.x = self.v
            t.angular.z = self.w
            self.twist_pub.publish(t)
            rate.sleep()

        return 'ok'

    def _move_to(self, pose):  # type: (PoseStamped) -> bool
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = pose.header.frame_id
        goal.target_pose.pose.position = pose.pose.position
        goal.target_pose.pose.orientation = pose.pose.orientation
        self.client.send_goal(goal)
        return self.client.wait_for_result()

    def _is_cube_straight(self):
        try:
            this_pose = PoseStamped()
            this_pose.header.frame_id = 'odom'
            this_pose.pose = self.odometry.value.pose.pose
            this_pose = self.tf_listener.transformPose('map', this_pose)
        except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException), e:
            return False

        this_angle, _, _ = transformations.rotation_from_matrix(transformations.quaternion_matrix(numpify(this_pose.pose.orientation)))
        normal = numpify(pose_with_offset(self.tag.pose, (0, 0, 1)).pose.position) - numpify(self.tag.pose.pose.position)
        tag_angle = np.arctan2(normal[1], normal[0])
        diff = angle_diff(this_angle, tag_angle)

        return abs(diff % (np.pi/2)) < 0.1 or abs(diff % (np.pi/2)) > np.pi/2 - 0.1


def main():
    v = 0.2
    w = 1

    turn_time = 1.2
    move_time = 1.5
    wall_offset = 0.3

    cube = ARCube(2, alvar_topic='/alvar_main/ar_pose_marker', visual_topic='/viz/cube')
    marker = ARTag(4, alvar_topic='/alvar_main/ar_pose_marker', visual_topic='/viz/marker')

    sm = StateMachine(outcomes=['ok', 'err', 'finished'])
    with sm:
        StateMachine.add('find_cube_and_marker', FindTags([cube, marker]), transitions={'ok': 'straighten'})
        StateMachine.add('straighten', StraightenCubeState(cube, marker, v, 0.2), transitions={'ok': 'back_up_y'})

        StateMachine.add('back_up_y', ActionUntil(MoveForwardAction(-v), TimerListener(5)), transitions={'timeout': 'choose_start_y'})
        StateMachine.add('choose_start_y', ChooseNewNavGoalState(half_way_target(marker, cube), cube, 0.5),
                         transitions={'ok': 'goto_start_y'})
        StateMachine.add('goto_start_y', NavigateToGoalState(), transitions={'ok': 'push_box_y', 'err': None})
        StateMachine.add('push_box_y', PushToGoalState(half_way_target(marker, cube), v=v), transitions={'ok': 'back_up_x'})

        StateMachine.add('back_up_x', ActionUntil(MoveForwardAction(-v), TimerListener(5)), transitions={'timeout': 'choose_start_x'})
        StateMachine.add('choose_start_x', ChooseNewNavGoalState(parking_square_target(marker, wall_offset), cube, 0.5),
                         transitions={'ok': 'goto_start_x'})
        StateMachine.add('goto_start_x', NavigateToGoalState(), transitions={'ok': 'push_box_x', 'err': None})
        StateMachine.add('push_box_x', PushToGoalState(parking_square_target(marker, wall_offset), v=v))

    sis = IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()

    sm.execute()


if __name__ == '__main__':
    rospy.init_node('demo6')
    main()
