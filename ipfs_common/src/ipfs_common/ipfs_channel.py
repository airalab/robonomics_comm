# -*- coding: utf-8 -*-
#
# Robonomics information channels support node.
#

from robonomics_msgs.msg import Demand, Offer, Result, AddedOrderFeedback, AddedPendingTransactionFeedback
from robonomics_msgs.messageValidator import convertMessage
from robonomics_msgs.robonomicsMessageUtils import offer2dict, demand2dict, res2dict, addedOrderFeedback2dict, addedPendingTransactionFeedback2dict
from .pubsub import publish, subscribe
from .ipfs_common import build_client
from threading import Thread
import rospy


class IPFSChannel:
    def __init__(self):
        '''
            Robonomics information channel initialisation.
        '''
        rospy.init_node('ipfs_channel')
        self.lighthouse = rospy.get_param('~lighthouse_contract')
        self.ipfs_pubsub_provider = rospy.get_param('~ipfs_pubsub_provider')

        self.ipfs_pubsub_client = build_client(self.ipfs_pubsub_provider)
        self.ipfs_topic_subscriber = subscribe(self.ipfs_pubsub_client, self.lighthouse)

        self.incoming_offer  = rospy.Publisher('incoming/offer',  Offer, queue_size=10)
        self.incoming_demand = rospy.Publisher('incoming/demand', Demand, queue_size=10)
        self.incoming_result = rospy.Publisher('incoming/result', Result, queue_size=10)
        self.incoming_added_order_feedback = rospy.Publisher('incoming/feedback/added_order', AddedOrderFeedback, queue_size=10)
        self.incoming_added_pending_transaction_feedback = rospy.Publisher('incoming/feedback/added_pending_transaction', AddedPendingTransactionFeedback, queue_size=10)

        rospy.Subscriber('eth/sending/offer', Offer,
                         lambda m: publish(self.ipfs_pubsub_client, self.lighthouse, offer2dict(m)))
        rospy.Subscriber('eth/sending/demand', Demand,
                         lambda m: publish(self.ipfs_pubsub_client, self.lighthouse, demand2dict(m)))
        rospy.Subscriber('eth/sending/result', Result,
                         lambda m: publish(self.ipfs_pubsub_client, self.lighthouse, res2dict(m)))
        rospy.Subscriber('eth/sending/feedback/added_order', AddedOrderFeedback,
                         lambda m: publish(self.ipfs_pubsub_client, self.lighthouse, addedOrderFeedback2dict(m)))
        rospy.Subscriber('eth/sending/feedback/added_pending_transaction', AddedPendingTransactionFeedback,
                         lambda m: publish(self.ipfs_pubsub_client, self.lighthouse, addedPendingTransactionFeedback2dict(m)))

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        def channel_thread():
            while True:
                try:
                    rospy.loginfo("[ipfs_channel] subscribe to %s topic %s", self.ipfs_pubsub_provider, self.lighthouse)
                    for m in self.ipfs_topic_subscriber:
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
                            elif isinstance(converted, AddedOrderFeedback):
                                # rospy.logwarn('DEBUG: Publish valid AddedOrderFeedback message %s', converted)
                                self.incoming_added_order_feedback.publish(converted)
                            elif isinstance(converted, AddedPendingTransactionFeedback):
                                # rospy.logwarn('DEBUG: Publish valid AddedPendingTransactionFeedback message %s', converted)
                                self.incoming_added_pending_transaction_feedback.publish(converted)
                except Exception as e:
                    rospy.logerr("[ipfs_channel] channel_thread exception: %s", e)
                    self.ipfs_pubsub_client = build_client(self.ipfs_pubsub_provider)
                    self.ipfs_topic_subscriber = subscribe(self.ipfs_pubsub_client, self.lighthouse)

        Thread(target=channel_thread, daemon=True).start()
        rospy.spin()
