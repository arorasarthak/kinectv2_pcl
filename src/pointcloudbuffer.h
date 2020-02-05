#ifndef POINTCLOUDBUFFER_H
#define POINTCLOUDBUFFER_H

#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

class PointCloudBuffer
{
public:
    PointCloudBuffer();

    void updateBuffer(pcl::PointXYZ &point);
    pcl::PointCloud<pcl::PointXYZ>::Ptr readBuffer();

    void notifyViewer();
    void notifyGrabber();
    bool isDataViewed();


    void setViewer(pcl::visualization::PCLVisualizer::Ptr v);

private:
    std::mutex m_mutex;
    std::mutex m_mutex_viewed;

    std::condition_variable m_condVar;
    bool m_dataViewed;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc;
};

#endif // POINTCLOUDBUFFER_H
