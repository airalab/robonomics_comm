# -*- coding: utf-8 -*-
#
# IPFS driven global publisher/subscriber transport.
#
# Notice: ipfs daemon with `--enable-pubsub-experiment` should be started.

from base64 import b64encode, b64decode
from pexpect import spawn, EOF
from json import dumps, loads

def publish(topic, msg):
    '''
        Publish message to given topic.
    '''
    return spawn('ipfs pubsub pub {0} "{1}\r\n"'.format(topic, b64encode(dumps(msg)))).expect(EOF)

def subscribe(topic):
    '''
        Subscribe to given topic and return generator of received messages.
    '''
    child = spawn('ipfs pubsub sub {0}'.format(topic))
    while not child.eof():
        child.expect('\r\n', timeout=None)
        yield loads(b64decode(child.before))
