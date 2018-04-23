# -*- coding: utf-8 -*-
#
# Robonomics information channels support node.
#

from robonomics_lighthouse.msg import Ask, Bid, Result
from binascii import hexlify, unhexlify
from .pubsub import publish, subscribe
from urllib.parse import urlparse
from threading import Thread
import rospy

def bid2dict(b):
    return { 'model'    : b.model,
             'token'    : b.token,
             'cost'     : b.cost,
             'count'    : b.count,
             'lighthouseFee' : b.lighthouseFee,
             'salt'     : hexlify(b.salt).decode('utf-8'),
             'signature': hexlify(b.signature).decode('utf-8'),
             'deadline' : b.deadline }

def ask2dict(a):
    return { 'model'    : a.model,
             'objective': a.objective,
             'token'    : a.token,
             'cost'     : a.cost,
             'count'    : a.count,
             'validator'    : a.validator,
             'validatorFee' : a.validatorFee,
             'salt'     : hexlify(a.salt).decode('utf-8'),
             'signature': hexlify(a.signature).decode('utf-8'),
             'deadline' : a.deadline }

def res2dict(r):
    return { 'liability' : r.liability, 
             'result'    : r.result,
             'signature' : hexlify(r.signature).decode('utf-8') }

class InfoChan:
    def __init__(self):
        '''
            Robonomics information channel initialisation.
        '''
        rospy.init_node('robonomics_infochan')
        self.lighthouse = rospy.get_param('~lighthouse_contract')
        ipfs_api = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs_api = '/ip4/{0}/tcp/{1}'.format(ipfs_api[0], ipfs_api[1])

        self.incoming_bid = rospy.Publisher('incoming/bid', Bid, queue_size=10)
        self.incoming_ask = rospy.Publisher('incoming/ask', Ask, queue_size=10)
        self.incoming_res = rospy.Publisher('incoming/result', Result, queue_size=10)

        self.market_chan = '{0}_market'.format(self.lighthouse)
        self.result_chan = '{0}_result'.format(self.lighthouse)
        rospy.Subscriber('sending/bid', Bid, lambda m: publish(self.ipfs_api, self.market_chan, bid2dict(m)))
        rospy.Subscriber('sending/ask', Ask, lambda m: publish(self.ipfs_api, self.market_chan, ask2dict(m)))
        rospy.Subscriber('sending/result', Result, lambda m: publish(self.ipfs_api, self.result_chan, res2dict(m)))

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        def market_thread():
            for m in subscribe(self.ipfs_api, self.market_chan):
                if 'objective' in m:
                    msg = Ask()
                    msg.model     = m['model']
                    msg.objective = m['objective']
                    msg.token     = m['token']
                    msg.cost      = m['cost']
                    msg.count     = m['count']
                    msg.validator    = m['validator']
                    msg.validatorFee = m['validatorFee']
                    msg.salt      = unhexlify(m['salt'].encode('utf-8'))
                    msg.signature = unhexlify(m['signature'].encode('utf-8'))
                    msg.deadline  = m['deadline']
                    self.incoming_ask.publish(msg)
                else:
                    msg = Bid()
                    msg.model     = m['model']
                    msg.token     = m['token']
                    msg.cost      = m['cost']
                    msg.count     = m['count']
                    msg.lighthouseFee = m['lighthouseFee']
                    msg.salt      = unhexlify(m['salt'].encode('utf-8'))
                    msg.signature = unhexlify(m['signature'].encode('utf-8'))
                    msg.deadline  = m['deadline']
                    self.incoming_bid.publish(msg)

        def result_thread():
            for m in subscribe(self.ipfs_api, self.result_chan):
                msg = Result()
                msg.liability = m['liability']
                msg.result    = m['result']
                msg.signature = unhexlify(m['signature'].encode('utf-8'))
                self.incoming_res.publish(msg)

        Thread(target=market_thread, daemon=True).start()
        Thread(target=result_thread, daemon=True).start()
        rospy.spin()
