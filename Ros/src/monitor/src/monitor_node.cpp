#include <dirent.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class NodeMonitor {
   public:
    NodeMonitor(const std::string& node_name, double cpu_threshold, double mem_threshold)
        : node_name_(node_name), nh_() {
        pid_ = getPidByName(node_name);
        if (pid_ == -1) {
            ROS_ERROR("Failed to find node with name: %s", node_name.c_str());
        }

        nh_.param("monitor/node1/cpu_threshold", cpu_threshold_, cpu_threshold);
        ROS_INFO("CPU threshold: %.2f%%", cpu_threshold_);

        nh_.param("monitor/node1/mem_threshold", mem_threshold_, mem_threshold);
        ROS_INFO("Memory threshold: %.2f%%", mem_threshold_);

    }

    void monitor() {
        if (pid_ == -1) return;

        double cpu_usage = getCpuUsage(pid_);
        double mem_usage = getMemUsage(pid_);

        ROS_INFO("Node: %s, PID: %d, CPU usage: %.2f%%, Memory usage: %.2f%%", node_name_.c_str(), pid_, cpu_usage,
                 mem_usage);

        if (cpu_usage > cpu_threshold_) {
            ROS_WARN("CPU usage of node %s exceeded threshold: %.2f%% > %.2f%%", node_name_.c_str(), cpu_usage,
                     cpu_threshold_);
            handleHighCpuUsage();
        }

        if (mem_usage > mem_threshold_) {
            ROS_WARN("Memory usage of node %s exceeded threshold: %.2f%% > %.2f%%", node_name_.c_str(), mem_usage,
                     mem_threshold_);
            handleHighMemUsage();
        }
    }

   private:
    pid_t getPidByName(const std::string& node_name) {
        ROS_INFO("Finding PID of node: %s", node_name.c_str());
        DIR* dir;
        if (!(dir = opendir("/proc"))) {
            ROS_ERROR("can't open /proc");
            return -1;
        }

        struct dirent* ent;

        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_type != DT_DIR) continue;

            if (!std::all_of(ent->d_name, ent->d_name + std::strlen(ent->d_name), ::isdigit)) continue;
            std::string cmdline_file = std::string("/proc/") + ent->d_name + "/cmdline";
            ROS_DEBUG("cmdline file: %s", cmdline_file.c_str());
            std::ifstream cmdline(cmdline_file.c_str());

            std::string line;
            std::getline(cmdline, line);
            std::replace(line.begin(), line.end(), '\0', ' ');
            ROS_DEBUG("PID: %s, CMD: %s", ent->d_name, line.c_str());

            if (line.find(node_name) != std::string::npos) {
                closedir(dir);
                return std::stoi(ent->d_name);
            }
        }
        closedir(dir);
        return -1;
    }

    double getCpuUsage(pid_t pid) {
        std::string stat_path = "/proc/" + std::to_string(pid) + "/stat";
        std::ifstream stat_file(stat_path);
        if (!stat_file.is_open()) {
            ROS_ERROR("Failed to open stat file: %s", stat_path.c_str());
            return -1;
        }

        std::string line;
        std::getline(stat_file, line);
        stat_file.close();

        std::istringstream iss(line);
        std::vector<std::string> parts;
        std::string part;
        while (std::getline(iss, part, ' ')) {
            parts.push_back(part);
        }

        long utime = std::stol(parts[13]);
        long stime = std::stol(parts[14]);
        long cutime = std::stol(parts[15]);
        long cstime = std::stol(parts[16]);

        long total_time = utime + stime + cutime + cstime;
        long uptime = getUptime();
        long hertz = sysconf(_SC_CLK_TCK);

        double cpu_usage = 100.0 * (total_time / hertz) / uptime;
        return cpu_usage;
    }

    double getMemUsage(pid_t pid) {
        std::string status_path = "/proc/" + std::to_string(pid) + "/status";
        std::ifstream status_file(status_path);
        if (!status_file.is_open()) {
            ROS_ERROR("Failed to open status file: %s", status_path.c_str());
            return -1;
        }

        std::string line;
        while (std::getline(status_file, line)) {
            if (line.find("VmRSS:") == 0) {
                std::istringstream iss(line);
                std::vector<std::string> parts;
                std::string part;
                while (iss >> part) {
                    parts.push_back(part);
                }

                long mem_kb = std::stol(parts[1]);
                long total_mem_kb = getTotalMem();
                double mem_usage = 100.0 * mem_kb / total_mem_kb;
                return mem_usage;
            }
        }

        status_file.close();
        return -1;
    }

    long getUptime() {
        std::ifstream uptime_file("/proc/uptime");
        double uptime;
        uptime_file >> uptime;
        uptime_file.close();
        return static_cast<long>(uptime);
    }

    long getTotalMem() {
        std::ifstream meminfo_file("/proc/meminfo");
        std::string line;
        long total_mem = 0;
        while (std::getline(meminfo_file, line)) {
            if (line.find("MemTotal:") == 0) {
                std::istringstream iss(line);
                std::string label;
                iss >> label >> total_mem;
                break;
            }
        }
        meminfo_file.close();
        return total_mem;
    }

    void handleHighCpuUsage() {
        // 定义CPU使用率过高时的处理逻辑，例如重启节点、发送告警等
        ROS_WARN("Handling high CPU usage for node %s...", node_name_.c_str());
        // 可以调用系统命令或者直接退出节点，以下示例是重启节点
        std::system(("rosnode kill " + node_name_).c_str());
        std::system("roslaunch your_package your_launch_file.launch");
    }

    void handleHighMemUsage() {
        // 定义内存使用率过高时的处理逻辑，例如重启节点、发送告警等
        ROS_WARN("Handling high memory usage for node %s...", node_name_.c_str());
        // 可以调用系统命令或者直接退出节点，以下示例是重启节点
        std::system(("rosnode kill " + node_name_).c_str());
        std::system("roslaunch your_package your_launch_file.launch");
    }

    ros::NodeHandle nh_;
    std::string node_name_;
    pid_t pid_;
    double cpu_threshold_;
    double mem_threshold_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "monitor");
    ros::NodeHandle nh;

    std::string node_name = "monitor_node";
    double cpu_threshold = 80.0;  // CPU使用率阈值（百分比）
    double mem_threshold = 80.0;  // 内存使用率阈值（百分比）
    NodeMonitor monitor(node_name, cpu_threshold, mem_threshold);

    ros::Rate rate(1);  // 监控频率
    while (ros::ok()) {
        monitor.monitor();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
