#include "grabber.h"
#include "viewer.h"
#include "pointcloudbuffer.h"

int main(int argc, char* argv[])
{
    std::string device_serial {"006407462747"};

    std::cout << "Thread is " << std::this_thread::get_id() << std::endl;

    PointCloudBuffer pc; // used by both grabber and viewer

    Grabber device0(device_serial, &pc);
    device0.setStarted(true);
    std::thread stream_worker(&Grabber::run, std::ref(device0));

    Viewer view(&pc);
    std::thread viz_worker(&Viewer::run, std::ref(view));

    stream_worker.join();
    viz_worker.join();

    return 0;
}
