# -*- coding: utf-8 -*-
#
# IPFS driven global publisher/subscriber transport.
#
# Notice: ipfs daemon with `--enable-pubsub-experiment` should be started.

from base64 import b64decode
from json import dumps, loads
import rospy

def publish(ipfs_client, topic, msg):
    '''
        Publish message to given topic.
    '''
    return ipfs_client.pubsub_pub(topic, dumps(msg))

def subscribe(ipfs_client, topic):
    '''
        Subscribe to given topic and return generator of received messages.
    '''
    with ipfs_client.pubsub_sub(topic) as sub:
        for msg in sub:
            try:
                yield loads(b64decode(msg['data']).decode('utf-8'))
            except Exception as e:
                rospy.logerr(e)
                rospy.sleep(1)
