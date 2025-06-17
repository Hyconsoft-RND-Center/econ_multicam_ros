#include <thread>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <mutex>

// Popcornsar header files
#include "adaptive_application.h"
#include "ara/core/initialization.h"
#include "ara/exec/execution_client.h"
#include "ara/log/logger.h"

#include "adaptive_application.h"

std::condition_variable cv;
std::mutex mtx;
bool is_running = false;

void Start_application()
{
    ara::core::Initialize();
    ara::log::Logger &mLogger = ara::log::CreateLogger("ECAM", "ECAM", ara::log::LogLevel::kInfo);
    ara::exec::ExecutionClient executionClient;

    while (true)
    {
        auto exec = executionClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);
        if (exec.HasValue())
        {
            mLogger.LogInfo() << "Ecosystem Provider AA Changed to Running State";
            
            // Notify main thread that the state is running
            {
                std::unique_lock<std::mutex> lock(mtx);
                is_running = true;
                cv.notify_one();
            }

            // Enter running state
            while (exec.HasValue())
            {
                mLogger.LogInfo() << "Ecosystem Provider AA Provider Running.........";
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // Optionally, you can re-check the execution state here if needed
                exec = executionClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);
            }
        }
        else
        {
            mLogger.LogInfo() << exec.Error().Message();
            // Optionally, add a delay before retrying
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // Cleanup code if necessary after the loop ends
    mLogger.LogInfo() << "Ecosystem Provider AA Stopped Running.";
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