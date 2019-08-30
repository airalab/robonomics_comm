from .ipfs_channel import IPFSChannel
from .ipfs_common import IPFSCommon


def ipfs_channel_node():
    IPFSChannel().spin()


def ipfs_common_node():
    IPFSCommon().spin()
