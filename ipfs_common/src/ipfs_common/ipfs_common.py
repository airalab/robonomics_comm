# -*- coding: utf-8 -*-
#
# IPFS common integration node.
#

from .ipfs_fileutils import _ipfs_upload_file, _ipfs_download_file
from urllib.parse import urlparse
from threading import Timer
import rospy
import ipfshttpclient
from urllib3.util.timeout import Timeout
from ipfs_common.srv import IpfsUploadFile, IpfsDownloadFile, IpfsUploadFileRequest


def build_client(provider_endpoint):
    rospy.loginfo("Build IPFS client: %s", provider_endpoint)
    return ipfshttpclient.connect(provider_endpoint, session=True)


class IPFSCommon:
    def __init__(self):
        '''
            IPFS common integration
        '''
        rospy.init_node('ipfs_commmon')

        file_providers = rospy.get_param('~ipfs_file_providers')
        self.ipfs_file_clients = list(map(build_client, file_providers))

        http_provider = rospy.get_param('~ipfs_http_provider')
        self.ipfs_http_client = build_client(http_provider)
        self.ipfs_http_clients = [self.ipfs_http_client]

        self.swarm_connect_addresses = rospy.get_param('~ipfs_swarm_connect_to')
        self.swarm_connect_interval = rospy.get_param('~swarm_connect_interval', 60)

        self.ipfs_add_file_publisher = rospy.Publisher('~ipfs/add_to_file_providers', IpfsUploadFileRequest, queue_size=10)

        def ipfs_add_file(add_file_request):
            self.ipfs_add_file_publisher.publish(add_file_request)
            return _ipfs_upload_file(self.ipfs_http_clients, add_file_request)
        rospy.Service('/ipfs/add_file', IpfsUploadFile, ipfs_add_file)

        def ipfs_get_file(download_file_request):
            return _ipfs_download_file(self.ipfs_http_client, download_file_request)
        rospy.Service('/ipfs/get_file', IpfsDownloadFile, ipfs_get_file)

        def __ipfs_add_to_file_providers(add_file_request):
            _ipfs_upload_file(self.ipfs_file_clients, add_file_request)
        if len(self.ipfs_file_clients) != 0:
            rospy.Subscriber('~ipfs/add_to_file_providers', IpfsUploadFileRequest, __ipfs_add_to_file_providers)

    def spin(self):
        def __ipfs_swarm_connect_thread():
            try:
                for s in self.swarm_connect_addresses:
                    self.ipfs_http_client.swarm.connect(s)
            except Exception as e:
                rospy.logerr("[ipfs_common] ipfs swarm connect error: %s", str(e))
            Timer(self.swarm_connect_interval, __ipfs_swarm_connect_thread).start()

        if len(self.swarm_connect_addresses) != 0:
            rospy.loginfo("[ipfs_common] launch swarm connect to %s", self.swarm_connect_addresses)
            __ipfs_swarm_connect_thread()

        rospy.spin()
