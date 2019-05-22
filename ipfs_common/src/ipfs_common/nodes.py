from . import ipfs_channel
from . import ipfs_common


def ipfs_channel_node():
    ipfs_channel.IPFSChannel().spin()


def ipfs_common_node():
    ipfs_common.IPFSCommon().spin()
