# Convert RGBD images to a point cloud

### Parameters
`-c/--color` _required_, str, Input RGB color image  
`-d/--depth` _required_, str, Input depth map  
`-i/--intrinsic` _required_, str, Camera intrinsic parameters (column-major order, 9x1 shape)  
`-nv/--no_view`, _optional_, Disable point cloud visualization  
`-o/--output` _optional_, str, Output point cloud file path

### Setup environment (Open3D==0.12.0)
```bash
pip install -r requirements.txt
```

### Run

```bash
python rgbd2pcd.py -c /path/to/file/rgb_image -d /path/to/file/depth_image -i "[fx, 0, 0, 0, fy, 0, cx, cy, 1]"
```
Where `fx`, `fy` represent the focal length, `cx`, `cy` represent the principal point.