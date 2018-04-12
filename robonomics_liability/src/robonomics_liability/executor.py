# -*- coding: utf-8 -*-
#
# Robonomics liability execution node.
#

from robonomics_liability.msg import Liability
from std_srvs.srv import Empty, EmptyResponse
from tempfile import TemporaryDirectory 
from web3 import Web3, HTTPProvider
from urllib.parse import urlparse
from threading import Thread
from .recorder import Recorder
from .player import Player
from queue import Queue
import rospy, ipfsapi, os

class Executor:
    liability_queue = Queue()
    liability_finish = False

    def __init__(self):
        '''
            Robonomics liability node initialisation.
        '''
        rospy.init_node('robonomics_liability_executor')

        web3_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(web3_provider))

        self.account = rospy.get_param('~eth_account_address')
        self.account = self.web3.eth.accounts[0] if len(self.account) == 0 else self.account

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        def incoming_liability(msg):
            if msg.promisor != self.account:
                rospy.logwarn('Liability %s is not for me, SKIP.', msg.address)
            else:
                rospy.loginfo('Append %s to liability queue.', msg.address)
                self.liability_queue.put(msg)
        rospy.Subscriber('incoming', Liability, incoming_liability)

        def finish_liability(msg):
            self.liability_finish = True
            return EmptyResponse()
        rospy.Service('finish', Empty, finish_liability)

        self.complete = rospy.Publisher('complete', Liability, queue_size=10)
        self.current  = rospy.Publisher('current', Liability, queue_size=10)

    def _liability_worker(self):
        while not rospy.is_shutdown():
            msg = self.liability_queue.get()
            rospy.loginfo('Start work on liability %s', msg.address)
            self.current.publish(msg)
            self.liability_finish = False

            with TemporaryDirectory() as tmpdir:
                rospy.logdebug('Temporary directory created: %s', tmpdir)
                os.chdir(tmpdir)

                rospy.logdebug('Getting objective %s...', msg.objective)
                self.ipfs.get(msg.objective)
                rospy.logdebug('Objective is written to %s', tmpdir + '/' + msg.objective)

                result_file = os.path.join(tmpdir, 'result.bag')
                rospy.logdebug('Start recording to %s...', result_file)

                recorder = Recorder(result_file)
                recorder.start()
                rospy.logdebug('Rosbag recorder started')

                player = Player(msg.objective)
                player.start()
                rospy.logdebug('Rosbag player started')

                while not self.liability_finish:
                    rospy.sleep(1)

                msg.result = self.ipfs.add(result_file)['Hash']
                self.complete.publish(msg)
                rospy.loginfo('Liability %s finished with %s', msg.address, msg.result)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        Thread(target=self._liability_worker, daemon=True).start()
        rospy.spin()
