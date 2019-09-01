# -*- coding: utf-8 -*-
#
# AIRA graph node
#

from eth_account.messages import defunct_hash_message
from ipfs_common.pubsub import subscribe, publish
from time import sleep, time
import ipfshttpclient
from urllib.parse import urlparse
from web3.auto import w3
from json import dumps, loads
from threading import Thread
from ethereum_common.eth_keyfile_helper import KeyfileHelper
from std_msgs.msg import String
import rospy


class AIRAGraph:
    def __init__(self):
        rospy.init_node('aira_graph')
        ipfs_api_parts = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        __keyfile_helper = KeyfileHelper(rospy.get_param('~keyfile'),
                                         keyfile_password_file=rospy.get_param('~keyfile_password_file'))
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.ipfs_client = ipfshttpclient.connect("/dns/{0}/tcp/{1}/http".format(ipfs_api_parts[0], ipfs_api_parts[1]))

        self.graph_topic = rospy.get_param('~graph_topic')
        self.lighthouse_topic = rospy.get_param('~lighthouse_topic')
        self.greeting = rospy.Publisher('greeting', String, queue_size=10)

    def __sign(self, json):
        msg = dumps(json)
        message_hash = defunct_hash_message(text=msg)
        signature = self.__account.signHash(message_hash)['signature']
        return "{}\n---\n{}".format(msg, w3.toHex(signature))

    def spin(self):
        stat = {'id': self.ipfs_client.id()['ID'], 'addresses': self.ipfs_client.id()['Addresses'], 'lighthouse': self.lighthouse_topic}

        def graph_thread():
            while not rospy.is_shutdown():
                sleep(5)
                try:
                    stat['peers'] = self.ipfs_client.pubsub.peers(self.lighthouse_topic)['Strings']
                    stat['timestamp'] = int(time())
                    stat_hash = '/ipfs/{}'.format(self.ipfs_client.add_str(self.__sign(stat)))
                    name = self.ipfs_client.name.publish(stat_hash)['Name']
                    rospy.loginfo('Published to /ipns/{}'.format(name))
                    publish(self.ipfs_client, self.graph_topic, name)
                except Exception as e:
                    rospy.logerr(e)
                    continue

        def greeting_thread():
            m = String()
            for msg in subscribe(self.ipfs_client, self.graph_topic):
                try:
                    path = self.ipfs_client.resolve('/ipns/{}'.format(msg))['Path']
                    payload, signature = self.ipfs_client.cat(path).decode('utf8').split('---')
                    json = loads(payload)
                    m.data = 'aira {}..{} online'.format(json['id'][:4], json['id'][-4:])
                    self.greeting.publish(m)
                except Exception as e:
                    rospy.logerr(e)
        Thread(target=graph_thread, daemon=True).start()
        Thread(target=greeting_thread, daemon=True).start()
        rospy.spin()
