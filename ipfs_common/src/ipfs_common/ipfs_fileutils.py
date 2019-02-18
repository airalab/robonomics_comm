import rospy
import os
from tempfile import gettempdir
from shutil import move
from ipfs_common.srv import IpfsUploadFileResponse, IpfsDownloadFileResponse


def ipfs_upload_file(ipfs_client, add_file_request):
    add_file_response = IpfsUploadFileResponse()

    filepath = add_file_request.file.filepath
    if os.path.isfile(filepath):
        ipfs_response = ipfs_client.add(filepath)
        try:
            add_file_response.ipfs_address.multihash = ipfs_response['Hash']
            add_file_response.success = True
        except TypeError:
            rospy.logwarn('IPFS add proceeding error: %s', ipfs_response[1]['Message'])
            add_file_response.ipfs_address.multihash = ipfs_response[0]['Hash']
            add_file_response.error_msg = ipfs_response[1]['Message']
            add_file_response.success = False
    else:
        add_file_response.error_msg = "File not found"
        add_file_response.success = False

    return add_file_response


def ipfs_download_file(ipfs_client, download_file_request):
    download_file_response = IpfsDownloadFileResponse()

    file_dst = download_file_request.file.filepath
    dst_dir, dst_file = os.path.split(file_dst)

    if not os.path.isdir(dst_dir):
        try:
            os.mkdir(dst_dir)
        except Exception as e:
            rospy.logerr("Directory %s does not exists and cannot be created: %s", e)
            download_file_response.success = False
            download_file_response.error_msg = "Failed to create directory " + dst_dir
            return download_file_response

    if os.path.isdir(file_dst):
        rospy.logwarn(
            "Collision between existed directory and IPFS downloading file destination \"%s\". Please fix it manually.",
            file_dst)
        download_file_response.success = False
        download_file_response.error_msg = "Collision between existed directory and IPFS downloading file destination " + file_dst + ". Please fix it manually "
        return download_file_response

    try:
        tempdir = gettempdir()
        os.chdir(tempdir)
        ipfs_client.get(download_file_request.ipfs_address.multihash)
        move(tempdir + os.path.sep + download_file_request.ipfs_address.multihash,
             file_dst)
        download_file_response.success = True
    except Exception as e:
        rospy.logerr("Failed to download %s to %s with exception: %s",
                     download_file_request.ipfs_address.multihash, file_dst, e)
        download_file_response.success = False
        download_file_response.error_msg = "Failed to download file: " + str(e)
    return download_file_response

