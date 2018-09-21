#!/usr/bin/env python

import rospy, rostest, rosbag
import sys, os, time
import unittest

from robonomics_liability.recorder import Recorder
from tempfile import TemporaryDirectory
from std_msgs.msg import String, Float32

PKG = 'robonomics_lighthouse'
NAME = 'test_recorder'


class TestRecorder(unittest.TestCase):
    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestRecorder, self).__init__(*args)

    def test_recorder(self):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)

            result_file = os.path.join(tmpdir, 'test_recorder.bag')
            recorder = Recorder(result_file,
                                all=True,
                                master_check_interval=0.1)
            recorder.start()

            pub1 = rospy.Publisher('/test_publisher1', String,  queue_size=10)
            time.sleep(1)
            pub1.publish("pub1-msg0")

            pub2 = rospy.Publisher('/test_publisher2', String,  queue_size=10)
            time.sleep(1)
            pub2.publish("pub2-msg0")
            time.sleep(1)
            pub2.publish("pub2-msg1")

            pub3 = rospy.Publisher('/test_publisher3', Float32, queue_size=10)
            time.sleep(1)
            pub3.publish(3.00)
            time.sleep(1)
            pub3.publish(3.01)

            time.sleep(5)
            recorder.stop()

            recorded_topics = {}
            recorded_bag = rosbag.Bag('test_recorder.bag', 'r')
            bag_topics = recorded_bag.get_type_and_topic_info()
            for topic, topic_info in bag_topics[1].items():
                rospy.loginfo("recorder test rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)
                if topic not in recorded_topics:
                    recorded_topics[topic] = topic_info[1]

            self.assertTrue('/test_publisher1' in recorded_topics, "/test_publisher1 is not recorded!")
            self.assertEqual(1, recorded_topics['/test_publisher1'], "unexpected number of topic messages")

            self.assertTrue('/test_publisher2' in recorded_topics, "/test_publisher2 is not recorded!")
            self.assertEqual(2, recorded_topics['/test_publisher2'], "unexpected number of topic messages")

            self.assertTrue('/test_publisher3' in recorded_topics, "/test_publisher3 is not recorded!")
            self.assertEqual(2, recorded_topics['/test_publisher3'], "unexpected number of topic messages")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRecorder, sys.argv)