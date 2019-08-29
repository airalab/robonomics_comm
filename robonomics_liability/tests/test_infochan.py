#!/usr/bin/env python

import rostest, sys, unittest, rospy, time
from robonomics_msgs.msg import Demand, Offer, Result, AddedOrderFeedback, AddedPendingTransactionFeedback
from robonomics_msgs.robonomicsMessageUtils import offer2dict, demand2dict, res2dict
import testMessages

PKG = 'robonomics_liability'
NAME = 'test_infochan'


class TestInfochan(unittest.TestCase):

    def __init__(self, *args):
        rospy.init_node(NAME)
        super(TestInfochan, self).__init__(*args)
        self._infochan_published_ask_success = False
        self._infochan_published_bid_success = False
        self._infochan_published_res_success = False
        self._infochan_published_added_order_feedback_success = False
        self._infochan_published_added_pending_transaction_feedback_success = False

        rospy.Subscriber('/liability/infochan/incoming/demand',
                         Demand, self.infochan_published_ask_handler)
        rospy.Subscriber('/liability/infochan/incoming/offer',
                         Offer,  self.infochan_published_bid_handler)
        rospy.Subscriber('/liability/infochan/incoming/result',
                         Result, self.infochan_published_result_handler)
        rospy.Subscriber('/liability/infochan/incoming/feedback/added_order',
                         AddedOrderFeedback, self.infochan_published_added_order_feedback_handler)
        rospy.Subscriber('/liability/infochan/incoming/feedback/added_pending_transaction',
                         AddedPendingTransactionFeedback, self.infochan_published_added_pending_transaction_feedback_handler)

        self.infochan_Ask_subscriber_topic = rospy.Publisher('/liability/infochan/eth/sending/demand', Demand, queue_size=10)
        self.infochan_Bid_subscriber_topic = rospy.Publisher('/liability/infochan/eth/sending/offer',  Offer,  queue_size=10)
        self.infochan_Res_subscriber_topic = rospy.Publisher('/liability/infochan/eth/sending/result', Result, queue_size=10)
        self.infochan_Added_Order_Feedback_subscriber_topic = rospy.Publisher('/liability/infochan/eth/sending/feedback/added_order',
                                                                              AddedOrderFeedback, queue_size=10)
        self.infochan_Added_Pending_Transaction_Feedback_subscriber_topic = rospy.Publisher('/liability/infochan/eth/sending/feedback/added_pending_transaction',
                                                                              AddedPendingTransactionFeedback, queue_size=10)


    def test_demand2dict(self):
        self.assertEqual(testMessages.validAskDict, demand2dict(testMessages.getValidAsk()))

    def test_offer2dict(self):
        self.assertEqual(testMessages.validBidDict, offer2dict(testMessages.getValidBid()))

    def test_res2dict(self):
        self.assertEqual(testMessages.validResDict, res2dict(testMessages.getValidRes()))

    def infochan_published_ask_handler(self, ask):
        self.assertEqual(testMessages.getValidAsk(), ask)
        self._infochan_published_ask_success = True

    def test_ask_is_published_on_topic(self):
        time.sleep(3) #infochan node subscribers may be not registered in master
        self.infochan_Ask_subscriber_topic.publish(testMessages.getValidAsk())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._infochan_published_ask_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._infochan_published_ask_success)

    def infochan_published_bid_handler(self, bid):
        self.assertEqual(testMessages.getValidBid(), bid)
        self._infochan_published_bid_success = True

    def test_bid_is_published_on_topic(self):
        time.sleep(3) #infochan node subscribers may be not registered in master
        self.infochan_Bid_subscriber_topic.publish(testMessages.getValidBid())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._infochan_published_bid_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._infochan_published_bid_success)

    def infochan_published_result_handler(self, result):
        self.assertEqual(testMessages.getValidRes(), result)
        self._infochan_published_res_success = True

    def test_result_is_published_on_topic(self):
        time.sleep(3) #infochan node subscribers may be not registered in master
        self.infochan_Res_subscriber_topic.publish(testMessages.getValidRes())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._infochan_published_res_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._infochan_published_res_success)

    def infochan_published_added_order_feedback_handler(self, feedback):
        self.assertEqual(testMessages.getValidAddedOrderFeedback(), feedback)
        self._infochan_published_added_order_feedback_success = True

    def test_added_order_feedback_is_published_on_topic(self):
        time.sleep(3) #infochan node subscribers may be not registered in master
        self.infochan_Added_Order_Feedback_subscriber_topic.publish(testMessages.getValidAddedOrderFeedback())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._infochan_published_added_order_feedback_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._infochan_published_added_order_feedback_success)

    def infochan_published_added_pending_transaction_feedback_handler(self, feedback):
        self.assertEqual(testMessages.getValidAddedPendingTransactionFeedback(), feedback)
        self._infochan_published_added_pending_transaction_feedback_success = True

    def test_added_pending_transaction_feedback_is_published_on_topic(self):
        time.sleep(3) #infochan node subscribers may be not registered in master
        self.infochan_Added_Pending_Transaction_Feedback_subscriber_topic.publish(testMessages.getValidAddedPendingTransactionFeedback())
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self._infochan_published_added_pending_transaction_feedback_success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self._infochan_published_added_pending_transaction_feedback_success)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestInfochan, sys.argv)
