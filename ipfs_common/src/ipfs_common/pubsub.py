# -*- coding: utf-8 -*-
#
# IPFS driven global publisher/subscriber transport.
#
# Notice: ipfs daemon with `--enable-pubsub-experiment` should be started.

from base64 import b64decode
from json import dumps, loads
import rospy


def publish(ipfs_pubsub_client, topic, msg):
    '''
        Publish message to given topic.
    '''
    return ipfs_pubsub_client.pubsub.publish(topic, dumps(msg))


def subscribe(ipfs_pubsub_client, topic):
    '''
        Subscribe to given topic and return generator of received messages.
    '''
    with ipfs_pubsub_client.pubsub.subscribe(topic) as sub:
        for msg in sub:
            try:
                yield loads(b64decode(msg['data']).decode('utf-8'))
            except Exception as e:
                rospy.logerr("IPFS Client sub error: %s", e)
                rospy.sleep(1)
