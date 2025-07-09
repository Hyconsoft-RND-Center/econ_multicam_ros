#include <thread>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include "econ_ros/adaptive_application.h"

std::condition_variable cv;
std::mutex mtx;
bool is_running = false;

void Start_application()
{
    std::cout << "Starting ROS2 e-multicam application..." << std::endl;
    
    // Simple initialization without AUTOSAR EM dependencies
    try {
        std::cout << "e-multicam ROS2 application initialized successfully" << std::endl;
        
        // Notify main thread that the state is running
        {
            std::unique_lock<std::mutex> lock(mtx);
            is_running = true;
            cv.notify_one();
        }

        // Keep the application running
        while (true) {
            std::cout << "e-multicam ROS2 application running..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error in ROS2 application: " << e.what() << std::endl;
    }

    std::cout << "e-multicam ROS2 application stopped" << std::endl;
}

extern "C" void start_AA()
{
    std::thread(Start_application).detach();
}

extern "C" void wait_for_AA_running()
{
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, []{ return is_running; });
}