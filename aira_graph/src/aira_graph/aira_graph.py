# -*- coding: utf-8 -*-
#
# AIRA graph node
#

from eth_account.messages import defunct_hash_message
from time import sleep, time
import ipfsapi
from urllib.parse import urlparse
from web3.auto import w3
from json import dumps
from threading import Thread
from ethereum_common.eth_keyfile_helper import KeyfileHelper
import rospy


class AIRAGraph:
    def __init__(self):
        rospy.init_node('aira_graph')
        ipfs_api_parts = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        __keyfile_helper = KeyfileHelper(rospy.get_param('~keyfile'),
                                         keyfile_password_file=rospy.get_param('~keyfile_password_file'))
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.ipfs_client = ipfsapi.Client(host=ipfs_api_parts[0], port=ipfs_api_parts[1])

        self.graph_topic = rospy.get_param('~graph_topic')
        self.lighthouse_topic = rospy.get_param('~lighthouse_topic')

    def __sign(self, json):
        msg = dumps(json)
        message_hash = defunct_hash_message(text=msg)
        signature = self.__account.signHash(message_hash)['signature']
        return "{}\n---\n{}".format(msg, w3.toHex(signature))

    def spin(self):
        stat = {'id': self.ipfs_client.id()['ID'], 'addresses': self.ipfs_client.id()['Addresses'], 'lighthouse': self.lighthouse_topic}

        def graph_thread():
            with self.ipfs_client.pubsub_sub(self.lighthouse_topic, discover=True) as sub:
                with self.ipfs_client.pubsub_sub(self.graph_topic, discover=True) as graph_sub:
                    while True:
                        stat['peers'] = self.ipfs_client.pubsub_peers(self.lighthouse_topic)['Strings']
                        stat['timestamp'] = int(time())
                        stat_hash = '/ipfs/{}'.format(self.ipfs_client.add_str(self.__sign(stat)))
                        name = self.ipfs_client.name_publish(stat_hash)['Name']
                        rospy.loginfo('Published to /ipns/{}'.format(name))
                        self.ipfs_client.pubsub_pub(self.graph_topic, '{}\r\n'.format(name))
                        sleep(5)

        Thread(target=graph_thread, daemon=True).start()
        rospy.spin()
