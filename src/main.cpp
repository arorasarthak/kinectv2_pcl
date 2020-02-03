#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>


std::mutex myMutex;
bool updating {false};
bool isViewing {false};



void stream(std::string ser, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {

    // [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    // [context]

    // Serial for one kinect
    std::string serial = ser; //"006824161547"

    bool viewer_enabled = true;
    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;

    // Device 0 for CUDA uses the GPU
    pipeline = new libfreenect2::CudaPacketPipeline(deviceId);

    // Detects and Lists Devices (DO NOT REMOVE!!)
    freenect2.enumerateDevices();
    dev = freenect2.openDevice(serial, pipeline);

    // Setup Frame Listeners Here
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    // Print Kinect Info
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    // Create registration and frame objects
    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams(),
                                                                              dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    // Setup Variables for iterating over the depth image
    int ctr = 0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    int img_height = 424;
    int img_width = 512;


    // Pointcloud
    pcl::PointXYZ point;
    auto pc{pointcloud};
    //auto v {viewer};

    //pc->makeShared();
    while (isViewing) {
        {
            std::lock_guard<std::mutex> guard(myMutex);
            if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 seconds
            {
                std::cout << "timeout!" << std::endl;
                //return -1;
            }
            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
            if (enable_rgb && enable_depth) {
                //registration->apply(rgb, depth, &undistorted, &registered);
                registration->undistortDepth(depth, &undistorted);

                for (int col{0}; col <= img_height; ++col) {
                    for (int row{0}; row <= img_width; ++row) {
                        registration->getPointXYZ(&undistorted, row, col, x, y, z);
                        if (!(std::isnan(x) && std::isnan(y) && std::isnan(z))) {
                            point.x = x;
                            point.y = y;
                            point.z = z;
                            pc->points.push_back(point);
                            //pc->makeShared();
                        }
                    }
                }
            }
            listener.release(frames);
            ++ctr;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Close and Disconnect the Device
    dev->stop();
    dev->close();
    delete registration;
}

void view(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    // Viewer object goes here
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();
    //auto pc {pointcloud};
    //auto v {viewer};
    isViewing = true;
    while(!viewer->wasStopped())
    {
        {
            const std::lock_guard<std::mutex> lock(myMutex);
            viewer->addPointCloud(pointcloud, "kinect_stream");
            viewer->spinOnce();
            viewer->removePointCloud("kinect_stream");
            pointcloud->clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    isViewing = false;
}


int main()
{
    // Device Serial goes here
    std::string device_serial {"006824161547"};

    // Initialize Point Cloud and Point
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {nullptr};
    pc->height = 424;
    pc->width = 512;
    pc->is_dense = true;

    // Start threads here
    std::thread stream_worker(stream, device_serial, pc);
    std::thread viz_worker(view, pc);

    stream_worker.join();
    viz_worker.join();

    return 0;
}
