# -*- coding: utf-8 -*-
#
# Robonomics market support node.
#

from robonomics_market.msg import Ask, Bid
from binascii import hexlify, unhexlify
from threading import Thread
from . import pubsub
import rospy

def bid2dict(b):
    return { 'model'    : b.model,
             'cost'     : b.cost,
             'count'    : b.count,
             'fee'      : b.fee,
             'salt'     : hexlify(b.salt).decode('utf-8'),
             'signature': hexlify(b.signature).decode('utf-8') }

def ask2dict(a):
    return { 'model'    : a.model,
             'objective': a.objective,
             'cost'     : a.cost,
             'count'    : a.count,
             'fee'      : a.fee,
             'salt'     : hexlify(a.salt).decode('utf-8'),
             'signature': hexlify(a.signature).decode('utf-8') }

class Market:
    def __init__(self):
        '''
            Robonomics market initialisation.
        '''
        rospy.init_node('robonomics_market')
        self.market_topic = rospy.get_param('~market_topic')
        self.incoming_bid = rospy.Publisher('incoming/bid', Bid, queue_size=10)
        self.incoming_ask = rospy.Publisher('incoming/ask', Ask, queue_size=10)
        rospy.Subscriber('sending/bid', Bid, lambda m: pubsub.publish(self.market_topic, bid2dict(m)))
        rospy.Subscriber('sending/ask', Ask, lambda m: pubsub.publish(self.market_topic, ask2dict(m)))

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        def incoming_thread():
            for m in pubsub.subscribe(self.market_topic):
                if 'objective' in m:
                    msg = Ask()
                    msg.model     = m['model']
                    msg.objective = m['objective']
                    msg.cost      = m['cost']
                    msg.count     = m['count']
                    msg.fee       = m['fee']
                    msg.salt      = unhexlify(m['salt'].encode('utf-8'))
                    msg.signature = unhexlify(m['signature'].encode('utf-8'))
                    self.incoming_ask.publish(msg)
                else:
                    msg = Bid()
                    msg.model     = m['model']
                    msg.cost      = m['cost']
                    msg.count     = m['count']
                    msg.fee       = m['fee']
                    msg.salt      = unhexlify(m['salt'].encode('utf-8'))
                    msg.signature = unhexlify(m['signature'].encode('utf-8'))
                    self.incoming_bid.publish(msg)
        Thread(target=incoming_thread).start()
        rospy.spin()
