#!/usr/bin/env python

import rospy, rostest, rosbag
import sys, os, time
import unittest

from robonomics_liability.recorder import Recorder
from tempfile import TemporaryDirectory
from std_msgs.msg import String, Float32

PKG = 'robonomics_liability'
NAME = 'test_recorder'


class TestRecorder(unittest.TestCase):
    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestRecorder, self).__init__(*args)

    def test_recorder(self):
        with TemporaryDirectory() as tmpdir:
            os.chdir(tmpdir)
            # records all messages
            recorder_all_bag = os.path.join(tmpdir, 'test_recorder_all.bag')
            recorder_all = Recorder(recorder_all_bag,
                                    master_check_interval=0.1)
            recorder_all.start()

            # records all messages in namespace = 'namesapce'
            recorder_namespace_bag = os.path.join(tmpdir, 'test_recorder_namespace.bag')
            recorder_namespace = Recorder(recorder_namespace_bag,
                                          namespace='/namespace',
                                          master_check_interval=0.1)
            recorder_namespace.start()

            # records all messages in namespace = 'namesapce'
            recorder_namespace_by_regex_bag = os.path.join(tmpdir, 'test_recorder_namespace_by_regex.bag')
            recorder_namespace_by_regex = Recorder(recorder_namespace_by_regex_bag,
                                                   namespace='/namespace',
                                                   topics=['/test'],
                                                   regex=True,
                                                   master_check_interval=0.1)
            recorder_namespace_by_regex.start()

            #   records all messages in namespace with specified topics
            recorder_namespace_by_topic_bag = os.path.join(tmpdir, 'test_recorder_namespace_by_topic.bag')
            recorder_namespace_by_topic = Recorder(recorder_namespace_by_topic_bag,
                                                   namespace='/namespace',
                                                   topics=['/test/publisher2'],
                                                   master_check_interval=0.1)
            recorder_namespace_by_topic.start()

            pub1 = rospy.Publisher('/test/publisher1', String, queue_size=10)
            time.sleep(1)
            pub1.publish("pub1-msg0")

            pub2 = rospy.Publisher('/namespace/test/publisher2', String, queue_size=10)
            time.sleep(1)
            pub2.publish("pub2-msg0")
            time.sleep(1)
            pub2.publish("pub2-msg1")

            pub3 = rospy.Publisher('/namespace/test/publisher3', Float32, queue_size=10)
            time.sleep(1)
            pub3.publish(3.00)
            time.sleep(1)
            pub3.publish(3.01)

            time.sleep(5)
            recorder_all.stop()
            recorder_namespace.stop()
            recorder_namespace_by_regex.stop()
            recorder_namespace_by_topic.stop()

            ### check if recorded_all bag contains all published messages
            recorded_all_topics = {}
            recorded_all = rosbag.Bag(recorder_all_bag, 'r')
            bag_topics = recorded_all.get_type_and_topic_info()
            for topic, topic_info in bag_topics[1].items():
                # rospy.loginfo("recorder test rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)
                if topic not in recorded_all_topics:
                    recorded_all_topics[topic] = topic_info[1]

            for t in recorded_all_topics:
                rospy.loginfo(" %s contains %s messages in topic %s", recorder_all_bag, t, recorded_all_topics[t])

            self.assertTrue('/test/publisher1' in recorded_all_topics, "/test/publisher1 is not recorded!")
            self.assertEqual(1, recorded_all_topics['/test/publisher1'], "unexpected number of topic messages")

            self.assertTrue('/namespace/test/publisher2' in recorded_all_topics,
                            "/namespace/test/publisher2 is not recorded!")
            self.assertEqual(2, recorded_all_topics['/namespace/test/publisher2'],
                             "unexpected number of topic messages")

            self.assertTrue('/namespace/test/publisher3' in recorded_all_topics,
                            "/namespace/test/publisher3 is not recorded!")
            self.assertEqual(2, recorded_all_topics['/namespace/test/publisher3'],
                             "unexpected number of topic messages")

            ### check if recorded_in namespace bag contains only messages from namespace 'namespace'
            recorded_namespace_topics = {}
            recorded_namespace = rosbag.Bag(recorder_namespace_bag, 'r')
            bag_topics = recorded_namespace.get_type_and_topic_info()
            for topic, topic_info in bag_topics[1].items():
                if topic not in recorded_namespace_topics:
                    recorded_namespace_topics[topic] = topic_info[1]

            for t in recorded_namespace_topics:
                rospy.loginfo(" %s contains %s messages in topic %s", recorder_namespace_bag, t,
                              recorded_namespace_topics[t])

            self.assertTrue('/namespace/test/publisher2' in recorded_namespace_topics,
                            "/namespace/test/publisher2 is not recorded!")
            self.assertEqual(2, recorded_namespace_topics['/namespace/test/publisher2'],
                             "unexpected number of topic messages")

            self.assertTrue('/namespace/test/publisher3' in recorded_namespace_topics,
                            "/namespace/test/publisher3 is not recorded!")
            self.assertEqual(2, recorded_namespace_topics['/namespace/test/publisher3'],
                             "unexpected number of topic messages")

            self.assertEqual(2, recorded_namespace_topics.__len__(),
                             "recorded by namespace topics contains something wrong")

            ### check if recorded_namespace_by_regex bag contains all published messages
            recorded_namespace_by_regex_topics = {}
            recorded_namespace_by_regex = rosbag.Bag(recorder_namespace_by_regex_bag, 'r')
            bag_topics = recorded_namespace_by_regex.get_type_and_topic_info()
            for topic, topic_info in bag_topics[1].items():
                # rospy.loginfo("recorder test rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)
                if topic not in recorded_namespace_by_regex_topics:
                    recorded_namespace_by_regex_topics[topic] = topic_info[1]

            for t in recorded_namespace_by_regex_topics:
                rospy.loginfo(" %s contains %s messages in topic %s", recorder_namespace_by_regex_bag, t,
                              recorded_namespace_by_regex_topics[t])

            self.assertTrue('/namespace/test/publisher2' in recorded_namespace_by_regex_topics,
                            "/namespace/test/publisher2 is not recorded!")
            self.assertEqual(2, recorded_namespace_by_regex_topics['/namespace/test/publisher2'],
                             "unexpected number of topic messages")

            self.assertTrue('/namespace/test/publisher3' in recorded_namespace_by_regex_topics,
                            "/namespace/test/publisher3 is not recorded!")
            self.assertEqual(2, recorded_namespace_by_regex_topics['/namespace/test/publisher3'],
                             "unexpected number of topic messages")

            self.assertEqual(2, recorded_namespace_by_regex_topics.__len__(),
                             "recorded in namespace by regex topics contains something wrong")

            ### check if recorded_namespace_by_topic bag contains namespace_by_topic published messages
            recorded_namespace_by_topic_topics = {}
            recorded_namespace_by_topic = rosbag.Bag(recorder_namespace_by_topic_bag, 'r')
            bag_topics = recorded_namespace_by_topic.get_type_and_topic_info()
            for topic, topic_info in bag_topics[1].items():
                # rospy.loginfo("recorder test rosbag contains %s messages of type %s in topic %s", topic_info[1], topic_info[0], topic)
                if topic not in recorded_namespace_by_topic_topics:
                    recorded_namespace_by_topic_topics[topic] = topic_info[1]

            for t in recorded_namespace_by_topic_topics:
                rospy.loginfo(" %s contains %s messages in topic %s", recorder_namespace_by_topic_bag, t,
                              recorded_namespace_by_topic_topics[t])

            self.assertTrue('/namespace/test/publisher2' in recorded_namespace_by_topic_topics,
                            "/namespace/test/publisher2 is not recorded!")
            self.assertEqual(2, recorded_namespace_by_topic_topics['/namespace/test/publisher2'],
                             "unexpected number of topic messages")

            self.assertEqual(1, recorded_namespace_by_topic_topics.__len__(),
                             "recorded in namespace by regex topics contains something wrong")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRecorder, sys.argv)