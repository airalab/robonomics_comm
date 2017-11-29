# -*- coding: utf-8 -*-
#
# Robonomics liability execution node.
#

from robonomics_liability.msg import Liability
from robonomics_liability.srv import SetLiabilityResult
from std_srvs.srv import Empty, EmptyResponse
from tempfile import TemporaryDirectory 
from web3 import Web3, HTTPProvider
from urllib.parse import urlparse
from .recorder import Recorder
from subprocess import Popen
from signal import SIGINT 
import rospy, ipfsapi, os

class Executor:
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

        self.set_result = rospy.ServiceProxy('set_result', SetLiabilityResult)

    def run_liability(self, msg):
        if msg.promisor != self.account:
            rospy.loginfo('Liability %s promisor is %s not me, skip it.', msg.address, msg.promisor)
            return
        else:
            rospy.loginfo('Liability %s promisor is %s is me, running...', msg.address, msg.promisor)

        with TemporaryDirectory() as tmpdir:
            rospy.loginfo('Temporary directory created: %s', tmpdir)
            os.chdir(tmpdir)

            rospy.loginfo('Getting objective %s...', msg.objective)
            self.ipfs.get(msg.objective)
            rospy.loginfo('Objective is written to %s', tmpdir + '/' + msg.objective)

            player = Popen('rosbag play -k ' + msg.objective, shell=True, cwd=tmpdir, preexec_fn=os.setsid)
            rospy.logdebug('Rosbag player started')

            result_file = os.path.join(tmpdir, 'result.bag')
            rospy.loginfo('Start recording to %s...', result_file)

            recorder = Recorder(result_file) 
            recorder.start()
            rospy.logdebug('Rosbag recorder started')

            def finish_record(r):
                player.terminate()
                recorder.stop()
                return EmptyResponse()
            rospy.Service(msg.objective + '/finish', Empty, finish_record)

            player.wait()

            result_hash = self.ipfs.add(result_file)['Hash']
            rospy.loginfo('Liability %s finished with %s', msg.address, result_hash)

            try:
                response = self.set_result(msg.address, result_hash)
                rospy.loginfo('Liability %s result txhash is %s', msg.address, response) 
            except rospy.ServiceException as e:
                rospy.logerr('Exception at result submission %s', e)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
