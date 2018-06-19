# -*- coding: utf-8 -*-
#
# Robonomics market Bid/Ask matcher node. 
#

from robonomics_lighthouse.msg import Bid, Ask, Deal
import rospy

class Matcher:
    bids = {}
    asks = {}

    def __init__(self):
        '''
            Market market matcher initialisation.
        '''
        rospy.init_node('robonomics_matcher')

        rospy.Subscriber('infochan/incoming/bid', Bid, lambda x: self.match(bid=x))
        rospy.Subscriber('infochan/incoming/ask', Ask, lambda x: self.match(ask=x))
        self.deal = rospy.Publisher('deal', Deal, queue_size=10)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def match(self, bid=None, ask=None):
        '''
            Make bids and asks intersection.
        '''
        assert(not bid or not ask)

        if not ask:
            h = hash((bid.model, bid.objective, bid.token, bid.cost))
            if h in self.bids:
                self.bids[h].append(bid)
            else:
                self.bids[h] = [bid]
        else:
            h = hash((ask.model, ask.objective, ask.token, ask.cost))
            if h in self.asks:
                self.asks[h].append(ask)
            else:
                self.asks[h] = [ask]

        prlot = lambda lot: '| Mod: {} Obj: {} Cst: {} |'.format(lot.model, lot.objective, lot.cost)

        try:
            if not ask:
                h = hash((bid.model, bid.objective, bid.token, bid.cost))
                ask = self.asks[h].pop()
                bid = self.bids[h].pop()
            else:
                h = hash((ask.model, ask.objective, ask.token, ask.cost))
                bid = self.bids[h].pop()
                ask = self.asks[h].pop()

            rospy.loginfo('Matched %s', prlot(ask))

            msg = Deal()
            msg.ask = ask
            msg.bid = bid
            self.deal.publish(msg)

        except (KeyError, IndexError):
            rospy.loginfo('No match %s', prlot(ask or bid))
