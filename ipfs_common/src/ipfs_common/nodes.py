from . import ipfs_channel


def ipfs_channel_node():
    ipfs_channel.IPFSChannel().spin()
