# -*- coding: utf-8 -*-
#
# Simple quoted transaction manager. 
#

from threading import Thread
from queue import Queue
from web3 import Web3
import rospy

class QuotaManager:
    def __init__(self, transact, marker):
        self.transact = transact
        self.marker = marker
        self.queue = Queue()

    def put(self, tx):
        rospy.loginfo('Transaction added to queue')
        self.queue.put(tx)

    def start(self):
        def worker():
            while not rospy.is_shutdown():
                while not self.marker():
                    rospy.sleep(15)
                self.transact(self.queue.get())

        Thread(target=worker, daemon=True).start()
