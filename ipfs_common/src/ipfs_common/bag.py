# -*- coding: utf-8 -*-

from ipfs_common.srv import IpfsUploadFile, IpfsDownloadFile
from ipfs_common.msg import Multihash, Filepath
from tempfile import NamedTemporaryFile
from rosbag import Bag
import rospy

def ipfs_download(multihash):
    rospy.wait_for_service('/ipfs/get_file')
    download = rospy.ServiceProxy('/ipfs/get_file', IpfsDownloadFile) 
    with NamedTemporaryFile(delete=False) as tmpfile:
        res = download(multihash, Filepath(tmpfile.name))
        if not res.success:
            raise Exception(res.error_msg)
        messages = {}
        for topic, msg, timestamp in Bag(tmpfile.name, 'r').read_messages():
            messages[topic] = msg
        return messages

def ipfs_upload(messages):
    rospy.wait_for_service('/ipfs/add_file')
    upload = rospy.ServiceProxy('/ipfs/add_file', IpfsUploadFile) 
    with NamedTemporaryFile(delete=False) as tmpfile:
        recorder = Bag(tmpfile.name, 'w')
        for topic in messages:
            recorder.write(topic, messages[topic])
        recorder.close()
        res = upload(Filepath(tmpfile.name))
        if not res.success:
            raise Exception(res.error_msg)
        return res.ipfs_address

class Bag:
    def __init__(self, messages=None, multihash=None):
        '''
            Parameters
            ----------
            messages : dict
                Serialize messages as objective BAG and upload to IPFS (default is None).

            multihash: Multihash
                Download and parse objective BAG from IPFS (default is None).
        '''
        if messages is None and multihash is None:
            raise NotImplementedError('messages or multihash should be set')

        if messages is None:
            self.multihash = multihash 
            self.messages = ipfs_download(multihash)
        else:
            self.messages = messages
            self.multihash = ipfs_upload(messages)
