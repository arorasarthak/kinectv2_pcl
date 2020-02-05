#include "viewer.h"

Viewer::Viewer(PointCloudBuffer *pcBuffer) : pcBuffer(pcBuffer)
{

}

void Viewer::run()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    while(!viewer->wasStopped())
    {
        const auto pointcloud = pcBuffer->readBuffer();

        std::cout << pointcloud->size() << std::endl;

        viewer->addPointCloud(pointcloud, "kinect_stream");
        viewer->spinOnce();
        viewer->removePointCloud("kinect_stream");
        pointcloud->clear();
        pcBuffer->notifyGrabber();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
