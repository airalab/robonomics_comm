# -*- coding: utf-8 -*-
#
# Simple quoted transaction manager. 
#

from threading import Thread
from queue import Queue
from web3 import Web3
import rospy

class QuotaManager:
    def __init__(self, quota, transact, marker):
        self.totalQuota = quota
        self.transact = transact
        self.marker = marker
        self.queue = Queue()
        rospy.loginfo('QuotaManager created (quota: %d)', self.totalQuota())

    def put(self, tx):
        rospy.loginfo('Transaction added to queue')
        self.queue.put(tx)

    def waitingForMarker(self):
        rospy.loginfo('Waiting for marker...')
        self.queue.join()
        while not self.marker():
            rospy.sleep(15)
        rospy.loginfo('Marker aquired, run!')

    def start(self):
        def master():
            self.waitingForMarker()
            quota = self.totalQuota()
            while not rospy.is_shutdown():
                if self.queue.empty():
                    rospy.sleep(5)
                    continue

                def slave():
                    self.transact(self.queue)
                Thread(target=slave, daemon=True).start()

                quota -= 1
                if quota == 0:
                    self.waitingForMarker()
                    quota = self.totalQuota()

        Thread(target=master, daemon=True).start()
