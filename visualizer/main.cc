
#include <vision_module/visualizer/visualizer.h>
#include <stdio.h>
#include <time.h>
#include <thread>

class thread_guard{
    std::thread& _t;

public:
    explicit thread_guard(std::thread& t) : _t(t) {}
    ~thread_guard(){
        if(_t.joinable())_t.join();
        printf("THREAD_GUARD : DESTRUCTOR\n");
    }

    thread_guard(thread_guard const&) = delete;
    thread_guard& operator=(thread_guard const&) = delete;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "visualizer");
    QApplication app(argc, argv);
    CVisualizer* window = new CVisualizer;

//    std::thread rosThread([]{ros::spin();});
//    thread_guard tg(rosThread);

    window->show();
    return app.exec();
}

