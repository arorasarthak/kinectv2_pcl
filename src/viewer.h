#ifndef VIEWER_H
#define VIEWER_H

#include "pointcloudbuffer.h"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

class Viewer
{
public:
    Viewer(PointCloudBuffer *pcBuffer);
    void run();

private:
    PointCloudBuffer *pcBuffer;
};

#endif // VIEWER_H
