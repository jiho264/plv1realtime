import time
import rospy
import os
import numpy as np
import dev.kitti_util as kitti_util
import ctypes
import struct
import cv2
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
#########################################################
## COEX MAIN INIT 
import torch
from ruamel.yaml import YAML
from stereo import Stereo
torch.backends.cudnn.benchmark = True
torch.set_grad_enabled(False)
config = 'cfg_coex.yaml'
version = 0  # CoEx
half_precision = True
def load_configs(path):
    cfg = YAML().load(open(path, 'r'))
    backbone_cfg = YAML().load(
        open(cfg['model']['stereo']['backbone']['cfg_path'], 'r'))
    cfg['model']['stereo']['backbone'].update(backbone_cfg)
    return cfg
#########################################################

def png_to_bin(left_img, right_img, calib):  
    ##png to npy################################################
    cv2.imwrite('dev/input.png',left_img)
    ## COEX Start ####################################
    start = torch.cuda.Event(enable_timing=True)
    end = torch.cuda.Event(enable_timing=True)
    start.record()
    
    tl = left_img.transpose((2,0,1))
    tr = right_img.transpose((2,0,1))
    imgL = torch.unsqueeze(torch.Tensor(tl), 0).cuda()
    imgR = torch.unsqueeze(torch.Tensor(tr), 0).cuda()

    end.record()
    torch.cuda.synchronize()
    runtime = start.elapsed_time(end)
    print('Data Preparation: {:.3f}'.format(runtime))

    start = torch.cuda.Event(enable_timing=True)
    end = torch.cuda.Event(enable_timing=True)
    start.record()
    with torch.no_grad():
        with torch.cuda.amp.autocast(enabled=half_precision):
            img = torch.cat([imgL, imgR], 0)
            disp = pose_ssstereo(img, training=False)
    end.record()
    torch.cuda.synchronize()
    runtime = start.elapsed_time(end)
    # print('Stereo runtime: {:.3f}'.format(runtime))

    global fps_list
    fps = 1000/runtime
    fps_list = np.append(fps_list, fps)
    if len(fps_list) > 5:
        fps_list = fps_list[-5:]
    avg_fps = np.mean(fps_list)
    print('Stereo runtime: {:.3f}'.format(1000/avg_fps))

    disp_np = (2*disp[0]).data.cpu().numpy().astype(np.uint8)
    
    disparity_map = disp_np
    ## COEX End ####################################
    cv2.imwrite('dev/disparity_end.png', disparity_map) 
    ##npy to bin################################################
    mycalib = kitti_util.Calibration(calib)
    disp_map = (disparity_map*256).astype(np.uint16)/256.
    def project_disp_to_points(calib00, disp, max_high):
        disp[disp < 0] = 0
        baseline = 0.54
        mask = disp > 0
        depth = calib00.f_u * baseline / (disp + 1. - mask)
        rows, cols = depth.shape
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        points = np.stack([c, r, depth])
        points = points.reshape((3, -1))
        points = points.T
        points = points[mask.reshape(-1)]
        cloud = calib00.project_image_to_velo(points)
        valid = (cloud[:, 0] >= 0) & (cloud[:, 2] < max_high)
        return cloud[valid]
    # max_high ==1
    lidar = project_disp_to_points(mycalib, disp_map, 1)
    lidar = np.concatenate([lidar, np.ones((lidar.shape[0], 1))], 1)
    lidar = lidar.astype(np.float32)
    return lidar

def sparsify(mylidar):
    def pto_ang_map(velo_points, H=64, W=512, slice=1):
        """
        :param H: the row num of depth map, could be 64(default), 32, 16
        :param W: the col num of depth map
        :param slice: output every slice lines
        """
        dtheta = np.radians(0.4 * 64.0 / H)
        dphi = np.radians(90.0 / W)

        x, y, z, i = velo_points[:, 0], velo_points[:, 1], velo_points[:, 2], velo_points[:, 3]

        d = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        r = np.sqrt(x ** 2 + y ** 2)
        d[d == 0] = 0.000001
        r[r == 0] = 0.000001
        phi = np.radians(45.) - np.arcsin(y / r)
        phi_ = (phi / dphi).astype(int)
        phi_[phi_ < 0] = 0
        phi_[phi_ >= W] = W - 1

        theta = np.radians(2.) - np.arcsin(z / d)
        theta_ = (theta / dtheta).astype(int)
        theta_[theta_ < 0] = 0
        theta_[theta_ >= H] = H - 1

        depth_map = - np.ones((H, W, 4))
        depth_map[theta_, phi_, 0] = x
        depth_map[theta_, phi_, 1] = y
        depth_map[theta_, phi_, 2] = z
        depth_map[theta_, phi_, 3] = i
        depth_map = depth_map[0::slice, :, :]
        depth_map = depth_map.reshape((-1, 4))
        depth_map = depth_map[depth_map[:, 0] != -1.0]
        return depth_map

    pc_velo = mylidar.reshape((-1, 4))
    valid_inds = (pc_velo[:, 0] < 120) & \
                (pc_velo[:, 0] >= 0) & \
                (pc_velo[:, 1] < 50) & \
                (pc_velo[:, 1] >= -50) & \
                (pc_velo[:, 2] < 1.5) & \
                (pc_velo[:, 2] >= -2.5)
    pc_velo = pc_velo[valid_inds]
    # depth, width, height
    sparse_points = pto_ang_map(pc_velo, H=64, W=512, slice=1)

    return sparse_points.astype(np.float32)

def bin_to_pcl(bin):
    def paint_points(points, color=[192,0,0]):
        color = np.array([color])
        new_pts = np.zeros([points.shape[0],6])
        new_pts[:,:3] = points[:,:3]
        new_pts[:, 3:] = new_pts[:, 3:] + color
        return new_pts

    points = bin.reshape((-1, 4))
    points = paint_points(points)
    return points
    
def pcl_to_ros(pcl_array):
    ros_msg = PointCloud2()

    ros_msg.header.stamp = rospy.Time.now()
    ros_msg.header.frame_id = "world"

    ros_msg.height = 1
    ros_msg.width = len(pcl_array)
    ros_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)
        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    ros_msg.data = b''.join(buffer)

    return ros_msg

def getimage2(ros_data):
    global image2
    image2 = np.frombuffer(ros_data.data, np.uint8)
    image2 = image2.reshape(ros_data.height, ros_data.width, 3)

def getimage3(ros_data):
    global image3
    image3 = np.frombuffer(ros_data.data, np.uint8)
    image3 = image3.reshape(ros_data.height, ros_data.width, 3)

def getcalib(input_ros_msg):
    global calib
    calib.P = input_ros_msg.P
    calib.R = input_ros_msg.R
    calib.velo = np.array([7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
                           1.480249e-02,  7.280733e-04, -9.998902e-01, -7.631618e-02,
                           9.998621e-01,  7.523790e-03,  1.480755e-02, -2.717806e-01])

if __name__ == "__main__":
    os.environ['CUDA_VISIBLE_DEVICES'] = '0' 

    rospy.init_node('plv1', anonymous=True)
    image2 = rospy.Subscriber('/kitti/camera_color_left/image_raw', Image, getimage2)
    image3 = rospy.Subscriber('/kitti/camera_color_right/image_raw', Image, getimage3)
    calib = rospy.Subscriber('/kitti/camera_color_left/camera_info', CameraInfo, getcalib)
    pub = rospy.Publisher("/pseudo_lidar", PointCloud2, queue_size = 10)

    rate = rospy.Rate(10) # 10hz
    ################################################
    ## COEX MAIN INIT
    cfg = load_configs(
        './configs/stereo/{}'.format(config))

    ckpt = '{}/{}/version_{}/checkpoints/last.ckpt'.format(
        'logs/stereo', cfg['model']['name'], version)
    cfg['stereo_ckpt'] = ckpt
    pose_ssstereo = Stereo.load_from_checkpoint(cfg['stereo_ckpt'],
                                                strict=False,
                                                cfg=cfg).cuda()
    fps_list = np.array([])
    pose_ssstereo.eval()
    
    ################################################
    while not rospy.is_shutdown():
        start = time.time()     
        # stereo image(.png) >> depth(.npy) >> pointcloud array(.bin)
        pointcloudxyzrgb = png_to_bin(image2, image3, calib)
        # High resolution pointcloud >> Low resolution pointcloud
        cloud = sparsify(pointcloudxyzrgb)       
        # pointcloudXYZRGB >> msgs/pointcloud2
        cloud = bin_to_pcl(cloud)
        # output
        cloud_new = pcl_to_ros(cloud)
        pub.publish(cloud_new)
        print('ok! %.5f' % (time.time() - start))