# -*- coding: utf-8 -*-
#
# Robonomics liability execution node.
#

from robonomics_liability.msg import Liability
from robonomics_liability.srv import FinishLiability, FinishLiabilityResponse
from robonomics_lighthouse.msg import Result
from tempfile import TemporaryDirectory
from urllib.parse import urlparse
from threading import Thread
from .recorder import Recorder
from .player import Player, get_rosbag_from_file
from queue import Queue
import rospy, ipfsapi, os
from ethereum_common import eth_keyfile_helper

class Executor:
    liability_queue = Queue()
    liability_finish = False

    def __init__(self):
        '''
            Robonomics liability node initialisation.
        '''
        rospy.init_node('robonomics_liability_executor')

        self.recording_topics = list(filter(None, [x.strip() for x in rospy.get_param('~recording_topics').split(",")]))
        self.master_check_interval = rospy.get_param('~master_check_interval')

        __keyfile = rospy.get_param('~keyfile')
        __keyfile_password_file = rospy.get_param('~keyfile_password_file')
        __keyfile_helper = eth_keyfile_helper.KeyfileHelper(__keyfile, keyfile_password_file=__keyfile_password_file)
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        def incoming_liability(msg):
            if msg.promisor != self.__account.address:
                rospy.logwarn('Liability %s is not for me, SKIP.', msg.address)
            else:
                rospy.loginfo('Append %s to liability queue.', msg.address)
                self.liability_queue.put(msg)
        rospy.Subscriber('incoming', Liability, incoming_liability)

        def finish_liability(msg):
            self.liability_success = msg.success
            self.liability_finish = True
            return FinishLiabilityResponse()
        rospy.Service('finish', FinishLiability, finish_liability)

        self.complete = rospy.Publisher('complete', Liability, queue_size=10)
        self.current  = rospy.Publisher('current', Liability, queue_size=10)
        self.result_topic = rospy.Publisher('result', Result, queue_size=10)

    def _liability_worker(self):
        while not rospy.is_shutdown():
            msg = self.liability_queue.get()
            rospy.loginfo('Start work on liability %s', msg.address)
            self.current.publish(msg)
            self.liability_finish = False
            self.liability_success = False

            with TemporaryDirectory() as tmpdir:
                rospy.logdebug('Temporary directory created: %s', tmpdir)
                os.chdir(tmpdir)

                rospy.logdebug('Getting objective %s...', msg.objective)
                self.ipfs.get(msg.objective)
                rospy.logdebug('Objective is written to %s', tmpdir + '/' + msg.objective)

                result_file = os.path.join(tmpdir, 'result.bag')
                rospy.logdebug('Start recording to %s...', result_file)
                rospy.logdebug("Recording all topics: %s", (not self.recording_topics))

                recorder = Recorder(result_file,
                                    all=(not self.recording_topics),
                                    topics=self.recording_topics,
                                    master_check_interval=self.master_check_interval)
                recorder.start()
                rospy.logdebug('Rosbag recorder started')

                objective_rosbag = get_rosbag_from_file(msg.objective)
                if objective_rosbag is not None:
                    player = Player(objective_rosbag)
                    player.start()
                    rospy.logdebug('Rosbag player started')
                else:
                    rospy.logwarn('Skip playing objective using rosbag player in liability %s', msg.address)

                while not self.liability_finish:
                    rospy.sleep(1)

                recorder.stop()
                ipfs_response = self.ipfs.add(result_file)
                try:
                    msg.result = ipfs_response['Hash']
                except TypeError:
                    rospy.logwarn('IPFS add proceeding error: %s', ipfs_response[1]['Message'])
                    msg.result = ipfs_response[0]['Hash']
                result_msg = Result()
                result_msg.liability = msg.address
                result_msg.result = msg.result
                result_msg.success = self.liability_success

                self.complete.publish(msg)
                self.result_topic.publish(result_msg)
                rospy.loginfo('Liability %s finished with %s', msg.address, msg.result)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        Thread(target=self._liability_worker, daemon=True).start()
        rospy.spin()
