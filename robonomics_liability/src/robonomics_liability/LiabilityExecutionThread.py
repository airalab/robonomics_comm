import threading

from .player import Player, get_rosbag_from_file
from .recorder import Recorder
import rospy
import os
from ipfs_common import ipfs_fileutils
from ipfs_common.srv import IpfsUploadFileRequest, IpfsDownloadFileRequest


class LiabilityExecutionThread(object):
    def __init__(self, work_directory, ipfs_client, master_check_interval, recording_topics,
                 liability=None, liability_execution_id=None, objective=None, namespace=None):

        self.work_directory = work_directory
        self.ipfs_client = ipfs_client
        self.master_check_interval = master_check_interval
        self.recording_topics = recording_topics

        self.liability = liability
        self.thread = None

        if self.liability is None:
            self.liability_execution_id = liability_execution_id
            self.execution_objective = objective
            self.__liability_execution_namespace = namespace
        else:
            self.liability_execution_id = self.liability.address.address
            self.execution_objective = self.liability.objective
            self.__liability_execution_namespace = "/liability/eth_{0}".format(self.liability_execution_id)

        self.__liability_result_file = os.path.join(self.work_directory, 'result.bag')
        self.__liability_objective_dst_file = os.path.join(self.work_directory, self.execution_objective.multihash)

        self.__player = self.__initializePlayer()
        self.__recorder = self.__initializeRecorder()

    def getLiabilityMsg(self):
        return self.liability

    def start(self, timestamp=None):
        rospy.loginfo("Liability execution thread start called: %s with timestamp %s", self.liability_execution_id, timestamp)
        self.thread = threading.Thread(target=self.__execute_liability, args=(timestamp,))
        self.thread.daemon = True
        self.thread.start()

    def finish(self, success):
        self.__recorder.stop()
        self.__player.stop()

        ipfs_add_file_request = IpfsUploadFileRequest()
        ipfs_add_file_request.file.filepath = self.__liability_result_file
        ipfs_add_file_response = ipfs_fileutils.ipfs_upload_file(self.ipfs_client, ipfs_add_file_request)

        return ipfs_add_file_response.ipfs_address

    def interrupt(self, delete_result=True):
        if self.thread.isAlive():
            self.__recorder.stop()
            self.__player.stop()
            if delete_result is True and os.path.exists(self.__liability_result_file):
                os.remove(self.__liability_result_file)

    def __initializeRecorder(self):
        __recorder = Recorder(self.__liability_result_file,
                              topics=self.recording_topics,
                              namespace=self.__liability_execution_namespace,
                              master_check_interval=self.master_check_interval)
        return __recorder

    def __initializePlayer(self):
        rospy.logdebug('Getting objective %s...', self.execution_objective.multihash)

        ipfs_get_file_request = IpfsDownloadFileRequest()
        ipfs_get_file_request.ipfs_address = self.execution_objective
        ipfs_get_file_request.file.filepath = self.__liability_objective_dst_file
        ipfs_get_file_response = ipfs_fileutils.ipfs_download_file(self.ipfs_client, ipfs_get_file_request)

        if not ipfs_get_file_response.success:
            rospy.logerr("Failed to download %s objective with IPFS error msg: %s",
                         self.execution_objective, ipfs_get_file_response.error_msg)
            return None
        rospy.logdebug('Objective is written to %s', self.__liability_objective_dst_file)

        objective_rosbag = get_rosbag_from_file(self.__liability_objective_dst_file)
        if objective_rosbag is not None:
            __player = Player(objective_rosbag, self.liability_execution_id, self.__liability_execution_namespace)
            return __player
        else:
            rospy.logwarn('rosbag player in liability %s is not created', self.liability_execution_id)
            return None

    def __execute_liability(self, resume_from_timestamp):
        rospy.logdebug('Start recording to %s in liability %s...',
                       self.__liability_result_file, self.liability_execution_id)
        rospy.logdebug("Recording all topics: %s", (not self.recording_topics))

        self.__recorder.start()
        rospy.logdebug('Rosbag recorder started in liability %s', self.liability_execution_id)

        if self.__player is not None:
            self.__player.start(resume_from_timestamp)
            rospy.logdebug('Rosbag player started in liability %s', self.liability_execution_id)
        else:
            rospy.logwarn('Skip playing objective using rosbag player in liability %s', self.liability_execution_id)
