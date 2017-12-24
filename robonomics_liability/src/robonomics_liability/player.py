# -*- coding: utf-8 -*-

import rospy, rosbag
from threading import Thread

class Player:
    def __init__(self, filename):
        bag = rosbag.Bag(filename, 'r')

        def worker():
            rospy.logdebug('Player: start %s', filename)
            pubs = {}
            for topic, msg, _ in bag.read_messages():
                if not topic in pubcache:
                    pubs[topic] = rospy.Publisher(topic, msg.__class__, queue_size=10)
                pubs[topic].publish(msg)
            rospy.logdebug('Player: finish %s', filename)
        Thread(target=worker, daemon=True).start()
