# -*- coding: utf-8 -*-
#
# IPFS common integration node.
#

from .ipfs_fileutils import ipfs_upload_file, ipfs_download_file
from urllib.parse import urlparse
import rospy
import ipfshttpclient
from ipfs_common.srv import IpfsUploadFile, IpfsDownloadFile


class IPFSCommon:
    def __init__(self):
        '''
            IPFS common integration
        '''
        rospy.init_node('ipfs_commmon')

        ipfs_api_parts = urlparse(rospy.get_param('~ipfs_http_provider')).netloc.split(':')
        self.ipfs_client = ipfshttpclient.connect("/dns/{0}/tcp/{1}/http".format(ipfs_api_parts[0], ipfs_api_parts[1]))

        def ipfs_add_file(add_file_request):
            return ipfs_upload_file(self.ipfs_client, add_file_request)
        rospy.Service('/ipfs/add_file', IpfsUploadFile, ipfs_add_file)

        def ipfs_get_file(download_file_request):
            return ipfs_download_file(self.ipfs_client, download_file_request)
        rospy.Service('/ipfs/get_file', IpfsDownloadFile, ipfs_get_file)

    def spin(self):
        rospy.spin()
