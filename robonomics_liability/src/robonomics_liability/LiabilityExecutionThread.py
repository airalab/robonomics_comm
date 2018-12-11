import threading

from robonomics_msgs.msg import Result
from ipfs_common.msg import Multihash
from .player import Player, get_rosbag_from_file
from .recorder import Recorder
from tempfile import TemporaryDirectory
import rospy, os


class LiabilityExecutionThread(object):
    def __init__(self, ipfs_client, master_check_interval, recording_topics, liability):

        self.ipfs_client = ipfs_client
        self.master_check_interval = master_check_interval
        self.recording_topics = recording_topics

        self.liability = liability

        self.thread = threading.Thread(target=self.__execute_liability, args=())
        self.thread.daemon = True

        self.__recorder = None
        self.__liability_result_file = None
        self.__liability_execution_namespace = "eth_{0}".format(self.liability.address)

        self.tmp_directory = TemporaryDirectory()
        rospy.logdebug('Temporary directory created: %s', self.tmp_directory.name)

    def getLiabilityMsg(self):
        return self.liability

    def start(self):
        self.thread.start()

    def finish(self, success):
        self.__recorder.stop()

        ipfs_response = self.ipfs_client.add(self.__liability_result_file)
        try:
            self.liability.result = ipfs_response['Hash']
        except TypeError:
            rospy.logwarn('IPFS add proceeding error: %s', ipfs_response[1]['Message'])
            self.liability.result = ipfs_response[0]['Hash']

        result_msg = Result()

        result_mh = Multihash()
        result_mh.multihash = self.liability.result

        result_msg.liability = self.liability.address
        result_msg.result = result_mh
        result_msg.success = success
        return result_msg

    def __createRecorder(self, result_file):
        recorder = Recorder(result_file,
                            topics=self.recording_topics,
                            namespace="/liability/{0}".format(self.__liability_execution_namespace),
                            master_check_interval=self.master_check_interval)
        return recorder

    def __execute_liability(self):
        os.chdir(self.tmp_directory.name)
        
        rospy.logdebug('Getting objective %s...', self.liability.objective.multihash)
        self.ipfs_client.get(self.liability.objective.multihash)
        rospy.logdebug('Objective is written to %s', self.tmp_directory.name + '/' + self.liability.objective.multihash)

        self.__liability_result_file = os.path.join(self.tmp_directory.name, 'result.bag')
        rospy.logdebug('Start recording to %s...', self.__liability_result_file)
        rospy.logdebug("Recording all topics: %s", (not self.recording_topics))

        self.__recorder = self.__createRecorder(self.__liability_result_file)
        self.__recorder.start()
        rospy.logdebug('Rosbag recorder started')

        objective_rosbag = get_rosbag_from_file(self.liability.objective.multihash)
        if objective_rosbag is not None:
            player = Player(objective_rosbag, self.__liability_execution_namespace)
            player.start()
            rospy.logdebug('Rosbag player started')
        else:
            rospy.logwarn('Skip playing objective using rosbag player in liability %s', self.liability.address)
