#ifndef GRABBER_H
#define GRABBER_H

#include "pointcloudbuffer.h"
#include <string>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/frame_listener_impl.h>
#include <iostream>

class Grabber
{
public:
    Grabber(std::string serial, PointCloudBuffer *pcBuffer);
    ~Grabber();

    std::string getSerial() const;

    void setSerial(const std::string &value);
    void setStarted(bool value);
    void run();

private:
    void initDevice();

private:
    std::string serial;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::Registration *registration;
    libfreenect2::SyncMultiFrameListener *listener;

    int deviceId = -1;
    bool enable_rgb = true;
    bool enable_depth = true;

    bool started;

    PointCloudBuffer *pcBuffer;
};


#endif // GRABBER_H
