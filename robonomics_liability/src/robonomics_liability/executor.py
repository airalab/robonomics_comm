# -*- coding: utf-8 -*-
#
# Robonomics liability execution node.
#

from robonomics_liability.msg import Liability
from robonomics_liability.srv import SetLiabilityResult
from tempfile import NamedTemporaryFile 
from urllib.parse import urlparse
from std_srvs.srv import Empty
import rospy, ipfsapi, os

class Executor:
    def __init__(self):
        '''
            Robonomics liability node initialisation.
        '''
        rospy.init_node('robonomics_liability_executor')

        ipfs_provider = urlparse(rospy.get_param('ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        self.run_liability_immediately = rospy.get_param('~run_liability_immediately')
        self.account = rospy.get_param('~account')

        def incoming_liability(msg):
            if self.run_liability_immediately:
                self.run_liability(msg)
            else:
                rospy.signal_shutdown(rospy.get_name() + ' liability queue is not implemented yet!')

        rospy.Subscriber('incoming', Liability, incoming_liability)

        self.set_result = rospy.ServiceProxy('set_result', SetLiabilityResult)

    def run_liability(self, msg):
        if msg.promisor != self.account:
            rospy.loginfo('Liability promisor is not me, skip it.')
            return

        with TemporaryDirectory() as tmpdir:
            rospy.loginfo('Temporary directory created %s', tmpdir)
            os.chdir(tmpdir)
            ipfs.get(msg.objective)

            player = Popen('rosbag play -k ' + msg.objective, shell=True)
            rospy.loginfo('Rosbag player started')
            recorder = Popen('rosbag record -j -a -o result.bag', shell=True)
            rospy.loginfo('Rosbag recorder started')


            def finish_record():
                player.terminate()
                recorder.terminate()
                rospy.loginfo('%s liability finished.', msg.objective)
            rospy.Service(msg.objective + '/finish', Empty, finish_record)

            player.wait()
            recorder.wait()

            result = ipfs.add('result.bag')
            rospy.loginfo('%s liability result hash is %s', msg.objective, result)

            try:
                self.set_result(msg.address, result)
            except rospy.ServiceException as e:
                rospy.logerr('Exception at result submission %s', e)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
