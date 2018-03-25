# -*- coding: utf-8 -*-
#
# Robonomics market Bid/Ask generator.
#

from robonomics_lighthouse.msg import Ask, Bid
from robonomics_lighthouse.srv import *
import rospy

class Generator:
    def __init__(self):
        '''
            Robonomics market Bid/Ask generator initialisation.
        '''
        rospy.init_node('robonomics_generator')
        self.signing_bid = rospy.Publisher('signing/bid', Bid, queue_size=128)
        self.signing_ask = rospy.Publisher('signing/ask', Ask, queue_size=128)

        def gen_asks(req):
            try:
                self.gen_linear_asks(req.a, req.k, req.market, req.objective, req.lighthouseFee, req.price_range)
                return AsksGeneratorResponse('ok')
            except Exception as e:
                return AsksGeneratorResponse('fail: {0}'.format(e))
        rospy.Service('gen_asks', AsksGenerator, gen_asks)

        def gen_bids(req):
            try:
                self.gen_linear_bids(req.a, req.k, req.market, req.lighthouseFee, req.validatorFee, req.price_range)
                return BidsGeneratorResponse('ok')
            except Exception as e:
                return BidsGeneratorResponse('fail: {0}'.format(e))
        rospy.Service('gen_bids', BidsGenerator, gen_bids)

    def spin(self):
        rospy.spin()

    def gen_linear_asks(self, a, k, market, objective, lighthouseFee, price_range):
        '''
            Market asks linear generator, ask function is `Q = a - k * P`
        '''
        msg = Ask()
        msg.model     = market
        msg.objective = objective
        msg.lighthouseFee = lighthouseFee
        for P in range(1, price_range):
            count = a - k * P
            if count > 0 and count % 1 == 0:
                msg.cost  = P
                msg.count = int(count)
                self.signing_ask.publish(msg)

    def gen_linear_bids(self, a, k, market, lighthouseFee, validatorFee, price_range):
        '''
            Market bids linear generator, bid function is `Q = a + k * P`
        '''
        msg = Bid()
        msg.model = market
        msg.lighthouseFee = lighthouseFee
        msg.validatorFee  = validatorFee
        for P in range(1, price_range):
            count = a + k * P
            if count > 0 and count % 1 == 0:
                msg.cost  = P
                msg.count = int(count)
                self.signing_bid.publish(msg)
