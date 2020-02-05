#include "pointcloudbuffer.h"
#include <iostream>

using namespace std::placeholders;

PointCloudBuffer::PointCloudBuffer()
{
    pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pc->height = 424;
    pc->width = 512;
    pc->is_dense = true;

    m_dataViewed = true;
}

void PointCloudBuffer::updateBuffer(pcl::PointXYZ & point)
{
    // Lock The Data structure
    std::unique_lock<std::mutex> guard(m_mutex);
    m_condVar.wait(guard, [&]{return isDataViewed();});
    pc->points.push_back(point);
}

void PointCloudBuffer::notifyViewer()
{
    std::cout << "notifyViewer()" << std::this_thread::get_id() << std::endl;
    // Set the flag to true, means data is loaded
    m_dataViewed = false;
    // Notify the condition variable
    m_condVar.notify_one();
}

void PointCloudBuffer::notifyGrabber()
{
    m_dataViewed = true;
    m_condVar.notify_one();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudBuffer::readBuffer()
{
    std::unique_lock<std::mutex> mlock(m_mutex);

    m_condVar.wait(mlock, [&]{return !isDataViewed();});

    std::cout << "readBuffer() " << std::this_thread::get_id() << std::endl;

    return pc;
}

bool PointCloudBuffer::isDataViewed()
{
    std::lock_guard<std::mutex> guard(m_mutex_viewed);
    return m_dataViewed;
}
