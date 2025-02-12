// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include "SystemInfosManager.hpp"

SystemInfosManager::~SystemInfosManager() {
    stopCpuMonitoring();
    data_.clear();
}

void SystemInfosManager::startCpuMonitoring() {
    data_.clear();
    running_          = true;
    monitoringThread_ = std::thread(&SystemInfosManager::monitoringLoop, this);
}

void SystemInfosManager::stopCpuMonitoring() {
    running_ = false;

    if(monitoringThread_.joinable()) {
        monitoringThread_.join();
    }
}

std::string SystemInfosManager::getCurrentTimeHMS() {
    // Get current time as time_point
    auto now = std::chrono::system_clock::now();

    // Convert to time_t for formatting
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert to tm structure for local time
    std::tm now_tm;
#ifdef _WIN32
    localtime_s(&now_tm, &now_time_t);  // Thread-safe on Windows
#else
    localtime_r(&now_time_t, &now_tm);  // Thread-safe on POSIX
#endif

    // Format the time into a stringstream (HH:MM:SS)
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%H:%M:%S");

    return oss.str();
}

#if(defined(_WIN32) || defined(_WIN64))
uint64_t SystemInfosManager::convertTimeFormat(const FILETIME *ftime) {
    LARGE_INTEGER li;

    li.LowPart  = ftime->dwLowDateTime;
    li.HighPart = ftime->dwHighDateTime;
    return li.QuadPart;
}

float SystemInfosManager::getCpuUsage() {
    // Static variables to store previous system and process times
    static int64_t last_time        = 0;
    static int64_t last_system_time = 0;

    auto pid = GetCurrentProcessId();

    FILETIME now;
    FILETIME creation_time;
    FILETIME exit_time;
    FILETIME kernel_time;
    FILETIME user_time;
    int64_t  system_time;
    int64_t  time;
    int64_t  system_time_delta;
    int64_t  time_delta;

    // get cpu num
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    int cpu_num = info.dwNumberOfProcessors;

    float cpu_ratio = 0.0;

    // get process hanlde by pid
    HANDLE process = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    // use GetCurrentProcess() can get current process and no need to close handle

    // get now time
    GetSystemTimeAsFileTime(&now);

    if(!GetProcessTimes(process, &creation_time, &exit_time, &kernel_time, &user_time)) {
        // We don't assert here because in some cases (such as in the Task Manager)
        // we may call this function on a process that has just exited but we have
        // not yet received the notification.
        printf("GetCpuUsageRatio GetProcessTimes failed\n");
        return 0.0;
    }

    // should handle the multiple cpu num
    system_time = (convertTimeFormat(&kernel_time) + convertTimeFormat(&user_time)) / cpu_num;
    time        = convertTimeFormat(&now);

    if((last_system_time == 0) || (last_time == 0)) {
        // First call, just set the last values.
        last_system_time = system_time;
        last_time        = time;
        return 0.0;
    }

    system_time_delta = system_time - last_system_time;
    time_delta        = time - last_time;

    CloseHandle(process);

    if(time_delta == 0) {
        printf("GetCpuUsageRatio time_delta is 0, error\n");
        return 0.0;
    }

    // We add time_delta / 2 so the result is rounded.
    cpu_ratio = (float)((static_cast<float>(system_time_delta) * 100.0 + static_cast<float>(time_delta) / 2.0) / static_cast<float>(time_delta));  // the % unit
    last_system_time = system_time;
    last_time        = time;

    return cpu_ratio;
}

float SystemInfosManager::getMemoryUsage() {
    PROCESS_MEMORY_COUNTERS memCounters;
    // Retrieve memory information for the current process
    if(GetProcessMemoryInfo(GetCurrentProcess(), &memCounters, sizeof(memCounters))) {
        // WorkingSetSize is the current size of the working set in bytes
        float memoryUsageMB = static_cast<float>(memCounters.WorkingSetSize) / (1024.0f * 1024.0f);

        return memoryUsageMB;
    }
    else {
        std::cerr << "Failed to get process memory info." << std::endl;

        return -1.0f;
    }
}

#elif defined(__linux__)
const std::string processName = "ob_benchmark";

float SystemInfosManager::getCpuUsage() {
    std::ostringstream command;
    command << "top -bn1 | grep '" << processName << "' | awk '{print $9}'";

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.str().c_str(), "r"), pclose);
    if(!pipe) {
        std::cerr << "Failed to run command" << std::endl;
        return -1.0f;
    }

    char buffer[128];
    if(fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
        return std::strtof(buffer, nullptr);
    }

    std::cerr << "Failed to read CPU usage" << std::endl;
    return -1.0f;
}

float SystemInfosManager::getMemoryUsage() {
    std::ostringstream command;
    command << "top -b -n 1 | grep '" << processName << "' | awk '{print $6}'";

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.str().c_str(), "r"), pclose);
    if(!pipe) {
        std::cerr << "Failed to run command" << std::endl;
        return -1.0f;
    }

    char        buffer[128];
    std::string result;
    while(fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
        result += buffer;
    }

    std::stringstream ss(result);
    long              memoryUsageKb;
    ss >> memoryUsageKb;

    float memoryUsageMb = static_cast<float>(memoryUsageKb) / 1024.0f;
    return memoryUsageMb;
}
#endif

const std::vector<SystemInfo> &SystemInfosManager::getData() const {
    return data_;
}

void SystemInfosManager::monitoringLoop() {
    while(running_) {
        float       cpuUsage    = getCpuUsage();
        float       memUsage    = getMemoryUsage();
        std::string currentTime = getCurrentTimeHMS();

        std::cout << "CPU usage: " << cpuUsage << "% | Memory usage: " << memUsage << "MB" << std::endl;

        data_.emplace_back(currentTime, cpuUsage, memUsage);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}