# -*- coding: utf-8 -*-
#
# Robonomics market support node.
#

from robonomics_market.msg import Ask, Bid
from threading import Thread
import rospy, pubsub

def bid2dict(b):
    return { 'model'    : b.model,
             'cost'     : b.cost,
             'fee'      : b.fee,
             'salt'     : b.salt,
             'signature': b.signature }

def ask2dict(a):
    return { 'model'    : a.model,
             'objective': a.objective,
             'cost'     : a.cost,
             'fee'      : a.fee,
             'salt'     : a.salt,
             'signature': a.signature }

class Market:
    def __init__(self):
        '''
            Robonomics market initialisation.
        '''
        rospy.init_node('robonomics_market')
        self.market_topic = rospy.get_param('market_topic', 'market')
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
                    msg.fee       = m['fee']
                    msg.salt      = m['salt']
                    msg.signature = m['signature']
                    self.incoming_ask.publish(msg)
                else:
                    msg = Bid()
                    msg.model     = m['model']
                    msg.cost      = m['cost']
                    msg.fee       = m['fee']
                    msg.salt      = m['salt']
                    msg.signature = m['signature']
                    self.incoming_bid.publish(msg)
        Thread(target=incoming_thread).start()
        rospy.spin()
