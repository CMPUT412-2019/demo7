import rospy
from typing import Any, Callable, Optional
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import Led
import playsound
import numpy as np
import time
from os import path
from threading import Thread
from tf.transformations import quaternion_conjugate, quaternion_multiply

def qv_mult(q1, v1):
    # https: // answers.ros.org / question / 196149 / how - to - rotate - vector - by - quaternion - in -python /
    q2 = list(v1) + [0.0]
    return quaternion_multiply(quaternion_multiply(q1, q2), quaternion_conjugate(q1))[:3]


class PublisherValue:
    def __init__(self, name, data_class, rate, value_fn, **kwargs):
        self.rate = rospy.Rate(rate)
        self.value_fn = value_fn
        self._publisher = rospy.Publisher(name, data_class, **kwargs)
        self._thread = Thread(target=self._spin)
        self._thread.start()

    def _spin(self):
        while not rospy.is_shutdown():
            value = self.value_fn()
            if value is not None:
                self._publisher.publish(value)
            self.rate.sleep()


class SubscriberValue:
    def __init__(self, name, data_class, wait=True, queue_size=1, transform=None):  # type: (str, Any, bool, int, Optional[Callable[[Any], Any]]) -> None
        self._subscriber = rospy.Subscriber(name, data_class, callback=self._callback, queue_size=queue_size)
        self._topic = name
        self._wait = wait
        self._transform = transform
        self._value = None
        self._n = 0

    def _callback(self, message):
        self._n += 1
        if self._transform is None:
            self._value = message
        else:
            self._value = self._transform(message)

    def wait(self):
        while self._value is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}...'.format(self._topic))
            rospy.sleep(0.1)
        return self._value

    @property
    def value(self):
        if self._wait:
            self.wait()
        return self._value

    def wait_for_n_messages(self, num_messages):  # type: (int) -> Any
        target = self._n + num_messages
        while self._n < target and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}... {}/{}'.format(self._topic, target-self._n, self._n))
            rospy.sleep(0.1)
        return self.value


class ProximityDetector:
    def __init__(self, proximity=1):
        self.proximity = proximity
        self.laser_scan = SubscriberValue('scan', LaserScan)

    def __call__(self, _):
        return np.nanmin(self.laser_scan.value.ranges) < self.proximity


def led(msg):  # type: (str) -> None
    msg = msg.upper()

    led_pub = {1: rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1, latch=True),
               2: rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1, latch=True)}
    led_col = {'R': Led.RED, 'G': Led.GREEN, 'O': Led.ORANGE, 'B': Led.BLACK}

    i = 0
    while i < len(msg):
        if msg[i] in led_col.keys() and i + 1 < len(msg):
            led_pub[int(msg[i+1])].publish(led_col[msg[i]])
            i = i + 1
        elif msg[i] == 'W':
            time.sleep(2)
        else:
            raise ValueError('Invalid message character: {}'.format(msg[i]))
        i = i + 1


def notify_count(count):
    led({1: 'b1b2', 2: 'g1b2', 3: 'g1g2'}.get(count, 'b1b2'))
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/{}.mp3'.format(count)), block=True)
    led('b1b2')


def notify_match():
    led('o1o2')
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/match.mp3'), block=True)
    led('b1b2')


def notify_artag():
    led('g1g2')
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/artag.mp3'), block=True)
    led('b1b2')


def notify_unmarked():
    led('r1r2')
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/unmarked.mp3'), block=True)
    led('b1b2')


def notify_number(n):  # type:  (int) -> None
    playsound.playsound(path.join(path.dirname(__file__), '../../../sound/number_{}.mp3'.format(int(n))), block=True)

def angle_diff(a, b):
    diff = a - b
    if diff < -np.pi: diff += 2 * np.pi
    if diff > np.pi: diff -= 2 * np.pi
    return diff
