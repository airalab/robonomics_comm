# -*- coding: utf-8 -*-
#
# Robonomics liability execution node.
#

from robonomics_liability.msg import Liability
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from tempfile import TemporaryDirectory 
from web3 import Web3, HTTPProvider
from urllib.parse import urlparse
from .recorder import Recorder
from subprocess import Popen
from signal import SIGINT 
import rospy, ipfsapi, os

class Executor:
    player = {}
    recorder = {}

    def __init__(self):
        '''
            Robonomics liability node initialisation.
        '''
        rospy.init_node('robonomics_liability_executor')

        web3_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(web3_provider))

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        self.run_liability_immediately = rospy.get_param('~run_liability_immediately')
        self.account = self.web3.eth.accounts[0] 

        def incoming_liability(msg):
            if self.run_liability_immediately:
                self.run_liability(msg)
            else:
                rospy.signal_shutdown(rospy.get_name() + ' liability queue is not implemented yet!')

        rospy.Subscriber('incoming', Liability, incoming_liability)

        def finish_record(msg):
            try:
                self.player[msg.data].terminate()
                self.recorder[msg.data].stop()
            except:
                rospy.logerr('Unable to finish liability %s', msg.data)
        rospy.Subscriber('finish', String, finish_record)

        self.complete = rospy.Publisher('complete', Liability, queue_size=10)
        self.current  = rospy.Publisher('current', Liability, queue_size=10)

    def run_liability(self, msg):
        if msg.promisor != self.account:
            rospy.loginfo('Liability %s promisor is %s not me, skip it.', msg.address, msg.promisor)
            return
        else:
            rospy.loginfo('Liability %s promisor is %s is me, running...', msg.address, msg.promisor)
            self.current.publish(msg)

        with TemporaryDirectory() as tmpdir:
            rospy.loginfo('Temporary directory created: %s', tmpdir)
            os.chdir(tmpdir)

            rospy.loginfo('Getting objective %s...', msg.objective)
            self.ipfs.get(msg.objective)
            rospy.loginfo('Objective is written to %s', tmpdir + '/' + msg.objective)

            self.player[msg.address] = Popen('rosbag play -k ' + msg.objective, shell=True, cwd=tmpdir, preexec_fn=os.setsid)
            rospy.logdebug('Rosbag player started')

            result_file = os.path.join(tmpdir, 'result.bag')
            rospy.loginfo('Start recording to %s...', result_file)

            self.recorder[msg.address] = Recorder(result_file)
            self.recorder[msg.address].start()
            rospy.logdebug('Rosbag recorder started')

            self.player[msg.address].wait()

            msg.result = self.ipfs.add(result_file)['Hash']
            rospy.loginfo('Liability %s finished with %s', msg.address, msg.result)

            self.complete.publish(msg)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
