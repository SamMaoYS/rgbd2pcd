import os
import argparse
import numpy as np
import open3d as o3d
from PIL import Image

def file_exist(file_path, ext=''):
    if not os.path.exists(file_path) or not os.path.isfile(file_path):
        return False
    elif ext in os.path.splitext(file_path)[1] or not ext:
        return True
    return False

def align_color2depth(o3d_color, o3d_depth):
    color_data = np.asarray(o3d_color)
    depth_data = np.asarray(o3d_depth)
    scale = [np.shape(depth_data)[0]/np.shape(color_data)[0], \
        np.shape(depth_data)[1]/np.shape(color_data)[1]]
    if scale != [1.0, 1.0]:
        color = Image.fromarray(color_data)
        depth = Image.fromarray(depth_data)
        color = color.resize(depth.size)
        return o3d.geometry.Image(np.asarray(color)), scale, np.shape(depth_data)
    return o3d_color, scale, np.shape(depth_data)

def get_intrinsic(intrinsic_str, width, height, scale=[1.0, 1.0]):
    if not intrinsic_str:
        return o3d.camera.PinholeCameraIntrinsic( \
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    intrinsic = np.fromstring(intrinsic_str[1:-1], dtype=np.float, sep=',')
    intrinsic = intrinsic.reshape(3, 3).transpose()
    scale = np.append(scale, 1.0)
    intrinsic = np.matmul(np.diag(scale), intrinsic)
    intrinsic = intrinsic.flatten('F').tolist()
    return o3d.camera.PinholeCameraIntrinsic(width, height, \
        intrinsic[0], intrinsic[4], intrinsic[6], intrinsic[7])

def configure(args):
    if not file_exist(args.color):
        print(f'ERROR: Cannot open file {args.color}')
        return False
    if not file_exist(args.depth):
        print(f'ERROR: Cannot open file {args.depth}')
        return False
    if not os.path.splitext(args.color)[1] in ['.jpg', '.jpeg', '.png', '.tiff', '.tif', '.bmp']:
        print(f'ERROR: File extension {os.path.splitext(args.color)[1]} is not supported')
        return False
    if not os.path.splitext(args.depth)[1] in ['.jpg', '.jpeg', '.png', '.tiff', '.tif', '.bmp']:
        print(f'ERROR: File extension {os.path.splitext(args.depth)[1]} is not supported')
        return False
    if args.output and not file_exist(args.output):
        print(f'ERROR: Cannot write file {args.output}')
        return False
    return True
    

def main(args):
    color = o3d.io.read_image(args.color)
    depth = o3d.io.read_image(args.depth)
    color, scale, shape = align_color2depth(color, depth)
    assert color, "ERROR: Empty color image"
    assert depth, "ERROR: Empty depth image"
    assert len(scale)==2, "ERROR: Wrong scale during align color to depth"
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth( \
        color, depth, convert_rgb_to_intensity=False)
    intrinsic = get_intrinsic(args.intrinsic, shape[0], shape[1], scale)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image( \
        rgbd, intrinsic)

    if args.output:
        o3d.io.write_point_cloud(args.output, pcd)

    # Flip pcd, otherwise will be upside down
    pcd.transform(np.diag([1, -1, -1, 1]))
    if args.view:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='View', width=640, height=480, visible=True)
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert RGBD images to a point cloud!')
    parser.add_argument('-c', '--color', dest='color', action='store', required=True, \
        help='Input RGB color image')
    parser.add_argument('-d', '--depth', dest='depth', action='store', required=True, \
        help='Input depth map')
    parser.add_argument('-i', '--intrinsic', dest='intrinsic', action='store', type=str, required=False, \
        help='Camera intrinsic parameters')
    parser.add_argument('-nv', '--no_view', dest='view', default=True, action='store_false', required=False, \
        help='Disable point cloud visualization')
    parser.add_argument('-o', '--output', dest='output', action='store', required=False, \
        help='Output point cloud file path')

    args = parser.parse_args()
    if not configure(args):
        exit(0)
    
    main(args)