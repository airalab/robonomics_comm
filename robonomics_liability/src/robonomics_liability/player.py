# -*- coding: utf-8 -*-

import rospy, rosbag
from threading import Thread


def file_is_rosbag(filename):
    try:
        bag = rosbag.Bag(filename, 'r')
        bag.get_type_and_topic_info()
        return True
    except Exception as e:
        rospy.logwarn("Failed to get rosbag topics info from file %s with exception: \"%s\"", filename, e)
        return False


def simple_publisher(msgs):
    pubs = {}
    for topic, msg, _ in msgs:
        if not topic in pubs:
            rospy.logdebug('New publisher %s of %s', topic, msg.__class__)
            pubs[topic] = rospy.Publisher(topic, msg.__class__, queue_size=10)
            rospy.sleep(1)

        rospy.logdebug('Publish message %s to %s', msg, topic)
        pubs[topic].publish(msg)

    rospy.sleep(5)
    rospy.logdebug('Publish done')


class Player:
    def __init__(self, filename):
        try:
            bag = rosbag.Bag(filename, 'r')
            self.publisher = Thread(target=simple_publisher, daemon=True, args=(bag.read_messages(),))
            rospy.logdebug('Player created for %s', filename)
        except Exception as e:
            rospy.logerr('Player exception: %s', e)

    def start(self):
        self.publisher.start()
        rospy.logdebug('Player started')
