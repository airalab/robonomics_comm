# -*- coding: utf-8 -*-

import rospy, rosbag
from threading import Thread


def get_rosbag_from_file(filename):
    try:
        bag = rosbag.Bag(filename, 'r')
        return bag
    except Exception as e:
        rospy.logwarn("Failed to get rosbag topics info from file %s with exception: \"%s\"", filename, e)
        return None


class Player:
    def __init__(self, bag, namespace=''):
        self.namespace = namespace
        self.pubs = {}

        try:
            self.publisher = Thread(target=self.simple_publisher, daemon=True, args=(bag.read_messages(),))
            rospy.logdebug('Player created for %s', bag.filename)
        except Exception as e:
            rospy.logerr('Player exception: %s', e)

    def simple_publisher(self, msgs):

        for topic, msg, _ in msgs:
            if not topic in self.pubs:
                rospy.logdebug('New publisher %s of %s', topic, msg.__class__)
                self.pubs[topic] = rospy.Publisher(self.namespace + topic, msg.__class__, queue_size=10)
                rospy.sleep(1)

            rospy.logdebug('Publish message %s to %s', msg, topic)
            self.pubs[topic].publish(msg)

        rospy.sleep(5)
        rospy.logdebug('Publish done')

    def start(self):
        self.publisher.start()
        rospy.logdebug('Player started')

    def stop(self):
        for topic in self.pubs:
            try:
                self.pubs[topic].unregister()
            except Exception as e:
                rospy.logwarn("Failed to unregister %s publisher", self.namespace + topic)
