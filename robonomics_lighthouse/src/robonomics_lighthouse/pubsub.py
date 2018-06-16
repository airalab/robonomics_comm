# -*- coding: utf-8 -*-
#
# IPFS driven global publisher/subscriber transport.
#
# Notice: ipfs daemon with `--enable-pubsub-experiment` should be started.

from base64 import b64encode, b64decode
from pexpect import spawn, EOF
from json import dumps, loads
import rospy

def publish(api, topic, msg):
    '''
        Publish message to given topic.
    '''
    msgdata = b64encode(dumps(msg).encode('utf-8')).decode('utf-8')
    return spawn('ipfs --api={0} pubsub pub {1} "{2}\r\n"'.format(api, topic, msgdata)).expect(EOF)

def subscribe(api, topic):
    '''
        Subscribe to given topic and return generator of received messages.
    '''
    child = spawn('ipfs --api={0} pubsub sub --discover {1}'.format(api, topic))
    while not child.eof():
        child.expect('\r\n', timeout=None)
        try:
            yield loads(b64decode(child.before).decode('utf-8'))
        except Exception as e:
            rospy.logerr(e)
            rospy.sleep(1)
