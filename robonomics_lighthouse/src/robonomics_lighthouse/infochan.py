# -*- coding: utf-8 -*-
#
# Robonomics information channels support node.
#

from robonomics_msgs.msg import Demand, Offer, Result
from binascii import hexlify
from .pubsub import publish, subscribe
from urllib.parse import urlparse
from threading import Thread
from .messageValidator import convertMessage
import rospy
import ipfsapi

def bid2dict(b):
    return { 'model'         : b.model,
             'objective'     : b.objective,
             'token'         : b.token,
             'cost'          : b.cost,
             'validator'     : b.validator,
             'lighthouseFee' : b.lighthouseFee,
             'deadline'      : b.deadline,
             'nonce'         : hexlify(b.nonce).decode('utf-8'),
             'signature'     : hexlify(b.signature).decode('utf-8') }


def ask2dict(a):
    return { 'model'        : a.model,
             'objective'    : a.objective,
             'token'        : a.token,
             'cost'         : a.cost,
             'validator'    : a.validator,
             'validatorFee' : a.validatorFee,
             'deadline'     : a.deadline,
             'nonce'        : hexlify(a.nonce).decode('utf-8'),
             'signature'    : hexlify(a.signature).decode('utf-8') }


def res2dict(r):
    return { 'liability' : r.liability,
             'result'    : r.result,
             'success'   : r.success,
             'signature' : hexlify(r.signature).decode('utf-8') }


class InfoChan:
    def __init__(self):
        '''
            Robonomics information channel initialisation.
        '''
        rospy.init_node('robonomics_infochan')
        self.lighthouse = rospy.get_param('~lighthouse_contract')
        ipfs_api_parts = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs_client = ipfsapi.Client(host=ipfs_api_parts[0], port=ipfs_api_parts[1])

        self.incoming_offer  = rospy.Publisher('incoming/offer',  Offer, queue_size=10)
        self.incoming_demand = rospy.Publisher('incoming/demand', Demand, queue_size=10)
        self.incoming_result = rospy.Publisher('incoming/result', Result, queue_size=10)

        rospy.Subscriber('eth/sending/offer',  Offer,  lambda m: publish(self.ipfs_client, self.lighthouse, bid2dict(m)))
        rospy.Subscriber('eth/sending/demand', Demand, lambda m: publish(self.ipfs_client, self.lighthouse, ask2dict(m)))
        rospy.Subscriber('eth/sending/result', Result, lambda m: publish(self.ipfs_client, self.lighthouse, res2dict(m)))

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        def channel_thread():
            for m in subscribe(self.ipfs_client, self.lighthouse):
                converted = convertMessage(m)
                if not (converted is None):
                    if isinstance(converted, Demand):
                        # rospy.logwarn('DEBUG: Publish valid Ask message %s', converted)
                        self.incoming_demand.publish(converted)
                    elif isinstance(converted, Offer):
                        # rospy.logwarn('DEBUG: Publish valid Bid message %s', converted)
                        self.incoming_offer.publish(converted)
                    elif isinstance(converted, Result):
                        # rospy.logwarn('DEBUG: Publish valid Result message %s', converted)
                        self.incoming_result.publish(converted)

        Thread(target=channel_thread, daemon=True).start()
        rospy.spin()
