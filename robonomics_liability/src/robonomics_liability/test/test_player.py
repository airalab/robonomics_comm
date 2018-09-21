#!/usr/bin/env python

import rospy, rostest, rosbag
import sys, os, time
import ipfsapi, unittest

from robonomics_liability.player import Player
from tempfile import TemporaryDirectory
from urllib.parse import urlparse

PKG = 'robonomics_lighthouse'
NAME = 'test_player'
TEST_ROSBAG = 'Qmb3H3tHZ1QutcrLq7WEtQWbEWjA11aPqVmeatMSrmFXvE'


class TestPlayer(unittest.TestCase):

    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestPlayer, self).__init__(*args)
        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))
        self.subscribers_msg_counters = {}

    def get_rosbag_Bag(self, rosbag_ipfs_objective):
        self.ipfs.get(rosbag_ipfs_objective)
        return rosbag.Bag(rosbag_ipfs_objective, 'r')

    def decrement_subscriber_msg_counter(self, topic):
        dcmnt = self.subscribers_msg_counters[topic] - 1
        rospy.loginfo("Decrement topic %s counter from value %s to %s", topic, self.subscribers_msg_counters[topic], dcmnt)
        self.subscribers_msg_counters[topic] = dcmnt

    def test_player(self):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            bag = self.get_rosbag_Bag(TEST_ROSBAG)
            subscribers = {}

            bag_topics = bag.get_type_and_topic_info()
            for topic, topic_info in bag_topics[1].items():
                self.subscribers_msg_counters[topic] = topic_info[1]
                rospy.loginfo("rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)

            def createSubscriberForTopic(topic, msg):
                rospy.loginfo('Create subscriber %s class %s', topic, msg.__class__)
                subscribers[topic] = rospy.Subscriber(topic,
                                                      msg.__class__,
                                                      lambda m: self.decrement_subscriber_msg_counter(topic))

            for topic, msg, _ in bag.read_messages():
                if topic not in subscribers:
                    createSubscriberForTopic(topic, msg)

            time.sleep(3)
            rospy.loginfo("Start player")
            player = Player(TEST_ROSBAG)
            player.start()
            time.sleep(3)

            for topic in self.subscribers_msg_counters:
                self.assertEqual(0, self.subscribers_msg_counters[topic],
                                 'Expected {0} more messages in topic {1}'.format(self.subscribers_msg_counters[topic], topic))
            rospy.loginfo("All messages from bag published")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestPlayer, sys.argv)