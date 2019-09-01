# -*- coding: utf-8 -*-
#
# Robonomics information channels support node.
#

from robonomics_msgs.msg import Demand, Offer, Result, AddedOrderFeedback, AddedPendingTransactionFeedback
from robonomics_msgs.messageValidator import convertMessage
from robonomics_msgs.robonomicsMessageUtils import offer2dict, demand2dict, res2dict, addedOrderFeedback2dict, addedPendingTransactionFeedback2dict
from .pubsub import publish, subscribe
from urllib.parse import urlparse
from threading import Thread
import rospy
import ipfshttpclient


class IPFSChannel:
    def __init__(self):
        '''
            Robonomics information channel initialisation.
        '''
        rospy.init_node('ipfs_channel')
        self.lighthouse = rospy.get_param('~lighthouse_contract')
        ipfs_api_parts = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs_client = ipfshttpclient.connect("/dns/{0}/tcp/{1}/http".format(ipfs_api_parts[0], ipfs_api_parts[1]))

        self.incoming_offer  = rospy.Publisher('incoming/offer',  Offer, queue_size=10)
        self.incoming_demand = rospy.Publisher('incoming/demand', Demand, queue_size=10)
        self.incoming_result = rospy.Publisher('incoming/result', Result, queue_size=10)
        self.incoming_added_order_feedback = rospy.Publisher('incoming/feedback/added_order', AddedOrderFeedback, queue_size=10)
        self.incoming_added_pending_transaction_feedback = rospy.Publisher('incoming/feedback/added_pending_transaction', AddedPendingTransactionFeedback, queue_size=10)

        rospy.Subscriber('eth/sending/offer',  Offer,
                         lambda m: publish(self.ipfs_client, self.lighthouse, offer2dict(m)))
        rospy.Subscriber('eth/sending/demand', Demand,
                         lambda m: publish(self.ipfs_client, self.lighthouse, demand2dict(m)))
        rospy.Subscriber('eth/sending/result', Result,
                         lambda m: publish(self.ipfs_client, self.lighthouse, res2dict(m)))
        rospy.Subscriber('eth/sending/feedback/added_order', AddedOrderFeedback,
                         lambda m: publish(self.ipfs_client, self.lighthouse, addedOrderFeedback2dict(m)))
        rospy.Subscriber('eth/sending/feedback/added_pending_transaction', AddedPendingTransactionFeedback,
                         lambda m: publish(self.ipfs_client, self.lighthouse, addedPendingTransactionFeedback2dict(m)))

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
                    elif isinstance(converted, AddedOrderFeedback):
                        # rospy.logwarn('DEBUG: Publish valid AddedOrderFeedback message %s', converted)
                        self.incoming_added_order_feedback.publish(converted)
                    elif isinstance(converted, AddedPendingTransactionFeedback):
                        # rospy.logwarn('DEBUG: Publish valid AddedPendingTransactionFeedback message %s', converted)
                        self.incoming_added_pending_transaction_feedback.publish(converted)

        Thread(target=channel_thread, daemon=True).start()
        rospy.spin()
