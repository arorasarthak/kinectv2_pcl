# kinectv2_pcl
Repo for Multi Kinect v2 Setup

## Requirements
- libfreekinect2
- PCL 1.8 or 1.9
- CUDA => https://support.system76.com/articles/cuda/
- cudnn
    $ sudo apt install system76-cudnn-latest
    $ sudo apt install system76-cudnn-10.1


## TODO List
- [x] Interface single Kinect v2 with libfreekinect2.
- [x] Ingest data into Point Clouds using PCL.
- [x] Interface multiple Kinect v2 devices with multithreading.
- [ ] Make a classes for Gabber and Viewer
- [ ] Use a concurrent data structure for capturing multiple Point Cloud Streams.
- [ ] Apply Registration to the point clouds (Optional)
