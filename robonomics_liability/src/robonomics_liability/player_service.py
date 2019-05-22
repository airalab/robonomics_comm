# -*- coding: utf-8 -*-
#
# Rosbag player service node.
#

from urllib.parse import urlparse
from .LiabilityExecutionThread import LiabilityExecutionThread
import rospy
from rospy.names import ParameterInvalid
from robonomics_liability.srv import CreateRosbagPlayer, CreateRosbagPlayerRequest, CreateRosbagPlayerResponse
from robonomics_liability.srv import StartRosbagPlayer, StartRosbagPlayerRequest, StartRosbagPlayerResponse
from robonomics_liability.srv import StopRosbagPlayer, StopRosbagPlayerRequest, StopRosbagPlayerResponse
from ipfs_common.msg import Multihash
import ipfsapi
import os
import string
import random


class PlayerService:
    def __init__(self):
        rospy.init_node('player_service', anonymous=True)

        ipfs_provider = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs_client = ipfsapi.connect(ipfs_provider[0], int(ipfs_provider[1]))

        self.executions_work_directory = os.path.join(os.getcwd(), 'liabilities_executions')
        self.master_check_interval = rospy.get_param('~master_check_interval')

        self.recording_topics = list(filter(None, [x.strip() for x in rospy.get_param('~recording_topics').split(",")]))

        self.player_threads = {}
        ns_validator = rospy.names.valid_name("namespace", True)

        def player_id_generator(multihash, namespace, size=8, chars=string.ascii_lowercase + string.digits):
            id = ''.join(random.choice(chars) for _ in range(size))
            return "{}{}_{}".format(multihash, namespace.replace('/', '_'), id)

        def __new_player(msg):
            ipfs_address = msg.ipfs_address

            response = CreateRosbagPlayerResponse()

            try:
                namespace = ns_validator(msg.namespace, '/')
            except ParameterInvalid as e:
                response.success = False
                response.error_msg = str(e)
                return response

            rospy.logwarn("createNewRosbagPlayer ipfs_address: " + ipfs_address.multihash)
            rospy.logwarn("createNewRosbagPlayer namespace: " + namespace)

            player_id = player_id_generator(msg.ipfs_address.multihash, namespace)
            try:
                player_thread = self.__createPlayerThread(player_id=player_id,
                                                          objective=ipfs_address,
                                                          namespace=namespace)
                self.player_threads[player_id] = player_thread

                response.success = True
                response.player_id = player_id
                response.error_msg = ''
            except Exception as e:
                response.success = False
                response.error_msg = str(e)
                response.player_id = ''

            return response
        rospy.Service('/rosbagplayer/new', CreateRosbagPlayer,
                      __new_player)

        def __start_player(msg):
            try:
                player_thread = self.player_threads[msg.player_id]
            except KeyError as e:
                rospy.logerr("Could not find player_id %s for starting", msg.player_id)
                return StartRosbagPlayerResponse(False, "Could not find player_id {0} for starting".format(msg.player_id))
            try:
                player_thread.start()
                rospy.loginfo('Player %s started', msg.player_id)
            except Exception as e:
                rospy.logerr("Can't start player %s with %s", msg.player_id, e)
                return StartRosbagPlayerResponse(False,
                                                 "Can't start player {0} with exception: {1}".format(msg.player_id, e))

            return StartRosbagPlayerResponse(True, "Player {0} started".format(msg.player_id))

        rospy.Service('/rosbagplayer/start', StartRosbagPlayer, __start_player)

        def __stop_player(msg):
            try:
                player_thread = self.player_threads.pop(msg.player_id)
                result_ipfs_address = player_thread.finish(msg.success)
                rospy.loginfo('Player %s stopped with %s', msg.player_id, result_ipfs_address)
                return StopRosbagPlayerResponse(True, result_ipfs_address)
            except KeyError as e:
                rospy.logerr("Could not find player_id %s for stopping", msg.player_id)
                return StopRosbagPlayerResponse(False, Multihash(''))
        rospy.Service('/rosbagplayer/stop', StopRosbagPlayer, __stop_player)

    def __createPlayerThread(self, player_id, objective, namespace):
        liability_work_directory = os.path.join(self.executions_work_directory, player_id)
        os.makedirs(liability_work_directory, exist_ok=True)
        rospy.loginfo('Use directory %s for liability %s executor thread',
                      liability_work_directory, player_id)
        thread = LiabilityExecutionThread(liability_work_directory,
                                          self.ipfs_client,
                                          self.master_check_interval,
                                          self.recording_topics,
                                          liability_execution_id=player_id,
                                          objective=objective,
                                          namespace=namespace)
        return thread

    def spin(self):
        rospy.spin()
