# -*- coding: utf-8 -*-
#
# IPFS common integration node.
#

from .ipfs_fileutils import _ipfs_upload_file, _ipfs_download_file
from urllib.parse import urlparse
import rospy
import ipfshttpclient
from urllib3.util.timeout import Timeout
from ipfs_common.srv import IpfsUploadFile, IpfsDownloadFile


def build_client(provider_endpoint):
    rospy.loginfo("Build IPFS client: %s", provider_endpoint)
    ipfs_url = urlparse(provider_endpoint)
    ipfs_netloc = ipfs_url.netloc.split(':')
    no_read_timeout = Timeout(read=None)
    return ipfshttpclient.connect("/dns/{0}/tcp/{1}/{2}".format(ipfs_netloc[0], ipfs_netloc[1], ipfs_url.scheme),
                                  session=True, timeout=no_read_timeout)


class IPFSCommon:
    def __init__(self):
        '''
            IPFS common integration
        '''
        rospy.init_node('ipfs_commmon')

        providers = rospy.get_param('~ipfs_file_providers')
        self.ipfs_file_clients = list(map(build_client, providers))

        def ipfs_add_file(add_file_request):
            return _ipfs_upload_file(self.ipfs_file_clients, add_file_request)
        rospy.Service('/ipfs/add_file', IpfsUploadFile, ipfs_add_file)

        def ipfs_get_file(download_file_request):
            return _ipfs_download_file(self.ipfs_file_clients, download_file_request)
        rospy.Service('/ipfs/get_file', IpfsDownloadFile, ipfs_get_file)

    def spin(self):
        rospy.spin()
