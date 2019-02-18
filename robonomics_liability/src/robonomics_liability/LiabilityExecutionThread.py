import threading

from robonomics_msgs.msg import Result
from .player import Player, get_rosbag_from_file
from .recorder import Recorder
import rospy
import os
from ipfs_common import ipfs_fileutils
from ipfs_common.srv import IpfsUploadFileRequest, IpfsDownloadFileRequest


class LiabilityExecutionThread(object):
    def __init__(self, work_directory, ipfs_client, master_check_interval, recording_topics, liability):

        self.work_directory = work_directory
        self.ipfs_client = ipfs_client
        self.master_check_interval = master_check_interval
        self.recording_topics = recording_topics

        self.liability = liability
        self.thread = None

        self.__liability_execution_namespace = "eth_{0}".format(self.liability.address.address)
        self.__liability_result_file = os.path.join(self.work_directory, 'result.bag')
        self.__liability_objective_dst_file = os.path.join(self.work_directory, self.liability.objective.multihash)

        self.__player = self.__initializePlayer()
        self.__recorder = self.__initializeRecorder()

    def getLiabilityMsg(self):
        return self.liability

    def start(self, timestamp=None):
        rospy.loginfo("Liability execution thread start called: %s with timestamp %s", self.liability.address.address, timestamp)
        self.thread = threading.Thread(target=self.__execute_liability, args=(timestamp,))
        self.thread.daemon = True
        self.thread.start()

    def finish(self, success):
        self.__recorder.stop()
        self.__player.stop()

        ipfs_add_file_request = IpfsUploadFileRequest()
        ipfs_add_file_request.file.filepath = self.__liability_result_file
        ipfs_add_file_response = ipfs_fileutils.ipfs_upload_file(self.ipfs_client, ipfs_add_file_request)

        self.liability.result = ipfs_add_file_response.ipfs_address

        result_msg = Result()
        result_msg.liability = self.liability.address
        result_msg.result = self.liability.result
        result_msg.success = success
        return result_msg

    def interrupt(self, delete_result=True):
        if self.thread.isAlive():
            self.__recorder.stop()
            self.__player.stop()
            if delete_result is True and os.path.exists(self.__liability_result_file):
                os.remove(self.__liability_result_file)

    def __initializeRecorder(self):
        __recorder = Recorder(self.__liability_result_file,
                              topics=self.recording_topics,
                              namespace="/liability/{0}".format(self.__liability_execution_namespace),
                              master_check_interval=self.master_check_interval)
        return __recorder

    def __initializePlayer(self):
        rospy.logdebug('Getting objective %s...', self.liability.objective.multihash)

        ipfs_get_file_request = IpfsDownloadFileRequest()
        ipfs_get_file_request.ipfs_address = self.liability.objective
        ipfs_get_file_request.file.filepath = self.__liability_objective_dst_file
        ipfs_get_file_response = ipfs_fileutils.ipfs_download_file(self.ipfs_client, ipfs_get_file_request)

        if not ipfs_get_file_response.success:
            rospy.logerr("Failed to download %s objective with IPFS error msg: %s",
                         self.liability.objective, ipfs_get_file_response.error_msg)
            return None
        rospy.logdebug('Objective is written to %s', self.__liability_objective_dst_file)

        objective_rosbag = get_rosbag_from_file(self.__liability_objective_dst_file)
        if objective_rosbag is not None:
            __player = Player(objective_rosbag, self.liability.address, self.__liability_execution_namespace)
            return __player
        else:
            rospy.logwarn('rosbag player in liability %s is not created', self.liability.address)
            return None

    def __execute_liability(self, resume_from_timestamp):
        rospy.logdebug('Start recording to %s in liability %s...',
                       self.__liability_result_file, self.liability.address.address)
        rospy.logdebug("Recording all topics: %s", (not self.recording_topics))

        self.__recorder.start()
        rospy.logdebug('Rosbag recorder started in liability %s', self.liability.address.address)

        if self.__player is not None:
            self.__player.start(resume_from_timestamp)
            rospy.logdebug('Rosbag player started in liability %s', self.liability.address.address)
        else:
            rospy.logwarn('Skip playing objective using rosbag player in liability %s', self.liability.address)
