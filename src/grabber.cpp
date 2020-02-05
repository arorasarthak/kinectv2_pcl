#include "grabber.h"

#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <thread>

Grabber::Grabber(std::string serial, PointCloudBuffer *pcBuffer) :
    serial(serial), pcBuffer(pcBuffer)
{
    started = false;
    initDevice();
}

Grabber::~Grabber()
{
    std::cout << "~Grabber()" << std::endl;
    dev->stop();
    dev->close();

    delete registration;
    delete listener;
}

void Grabber::initDevice()
{
    // Device 0 for CUDA uses the GPU
    pipeline = new libfreenect2::CudaPacketPipeline(deviceId);

    // Detects and Lists Devices (DO NOT REMOVE!!)
    auto devices = freenect2.enumerateDevices();
    std::cerr << devices << '\n';
    for (int d {0}; d <= devices; ++d){
        std::cerr << freenect2.getDeviceSerialNumber(d) << '\n';
    }

    dev = freenect2.openDevice(serial, pipeline);

    // Setup Frame Listeners Here
    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    listener = new libfreenect2::SyncMultiFrameListener(types);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    dev->start();

    // Print Kinect Info
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    // Create registration and frame objects
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

void Grabber::setStarted(bool value)
{
    started = value;
}

void Grabber::run()
{
    // Setup Variables for iterating over the depth image
    int ctr = 0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    int img_height = 424;
    int img_width = 512;

    // Pointcloud
    pcl::PointXYZ point;

    libfreenect2::FrameMap frames;
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    while (started) {

        if (!listener->waitForNewFrame(frames, 10 * 1000)) { // 10 sec timeout
            std::cout << "timeout!" << std::endl;
            break;
        }

        std::cout << "Frame received " << ctr << " " << std::this_thread::get_id() << std::endl;

        //libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
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
                        pcBuffer->updateBuffer(point);
                    }
                }
            }


        }
        listener->release(frames);
        ++ctr;

        pcBuffer->notifyViewer();

//        if (ctr == 50) {
//            break;
//        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }
}

std::string Grabber::getSerial() const
{
    return serial;
}

void Grabber::setSerial(const std::string &value)
{
    serial = value;
}
