// autorccar_util/src/process_manager_node.cpp
//
// 1) Process management: start/stop each package/rosbag via GCS command (/util/process_command)
//    Status is published as JSON on /util/process_status at 1Hz
// 2) System monitoring: publish CPU/memory/disk/temperature as JSON on /util/system_status at 1Hz
// 3) Power control: receive restart/shutdown commands on /util/system_command -> sudo reboot/shutdown
//
// Command  (std_msgs/String, JSON): {"name": "<id>", "action": "start"|"stop"}
// Status   (std_msgs/String, JSON): {"<id>": "running"|"stopped", ...}
// SystemCommand (std_msgs/String, JSON): {"action": "restart"|"shutdown"}
// SystemStatus  (std_msgs/String, JSON):
//   {"cpu_percent":.., "mem_percent":.., "mem_used_mb":.., "mem_total_mb":..,
//    "disk_percent":.., "disk_used_gb":.., "disk_total_gb":.., "temp_celsius":..}

#include <signal.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

struct CpuTimes {
    long user = 0, nice = 0, sys = 0, idle = 0;
    long iowait = 0, irq = 0, softirq = 0, steal = 0;

    long Total() const { return user + nice + sys + idle + iowait + irq + softirq + steal; }
};

class ProcessManagerNode : public rclcpp::Node {
   public:
    ProcessManagerNode() : Node("autorccar_process_manager") {
        configs_["gscam"] = {"ros2", "run", "gscam", "gscam_node"};
        configs_["livox"] = {"ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"};
        configs_["ublox"] = {"ros2", "run", "autorccar_ubloxf9r", "ubloxf9r"};
        configs_["lio_sam"] = {"ros2", "launch", "lio_sam", "run.launch.py"};
        configs_["ins_gnss"] = {"ros2", "launch", "autorccar_ins_gnss", "ins_gnss_nav.launch.py"};
        configs_["planning_control"] = {"ros2", "launch", "autorccar_planning_control", "planning_control.launch.py"};
        configs_["hardware_control"] = {"ros2", "launch", "autorccar_hardware_control", "hardware_control.launch.py"};
        configs_["costmap"] = {"ros2", "launch", "autorccar_costmap", "costmap.launch.py"};
        // "rosbag" is not put into configs_; its command is generated dynamically at start time

        // ── Process management ──────────────────────────────────────
        process_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "util/process_command", 10, std::bind(&ProcessManagerNode::OnProcessCommand, this, std::placeholders::_1));

        process_status_pub_ = create_publisher<std_msgs::msg::String>("util/process_status", 10);

        // ── System monitoring / power control ──────────────────────────
        system_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "util/system_command", 10, std::bind(&ProcessManagerNode::OnSystemCommand, this, std::placeholders::_1));

        system_status_pub_ = create_publisher<std_msgs::msg::String>("util/system_status", 10);

        // Capture an initial value for CPU usage calculation
        prev_cpu_ = ReadCpuTimes();

        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ProcessManagerNode::OnTimer, this));

        RCLCPP_INFO(get_logger(), "process_manager_node started");
    }

    ~ProcessManagerNode() override {
        for (auto& kv : pids_) {
            StopProcessByPid(kv.second);
        }
    }

   private:
    // ═══════════════════════════════════════════════════════
    // 1) Process management
    // ═══════════════════════════════════════════════════════

    void OnProcessCommand(const std_msgs::msg::String::SharedPtr msg) {
        string name, action;
        if (!ParseField(msg->data, "name", name) || !ParseField(msg->data, "action", action)) {
            RCLCPP_ERROR(get_logger(), "invalid process command: %s", msg->data.c_str());
            return;
        }

        if (action == "start") {
            StartProcess(name);
        } else if (action == "stop") {
            StopProcess(name);
        } else {
            RCLCPP_WARN(get_logger(), "unknown action: %s", action.c_str());
        }
    }

    void StartProcess(const string& name) {
        ReapFinished();

        auto it = pids_.find(name);
        if (it != pids_.end() && it->second > 0) {
            RCLCPP_INFO(get_logger(), "[%s] already running", name.c_str());
            return;
        }

        vector<string> argv_vec;
        if (name == "rosbag") {
            argv_vec = BuildRosbagCommand();
        } else if (configs_.count(name)) {
            argv_vec = configs_[name];
        } else {
            RCLCPP_WARN(get_logger(), "unknown process id: %s", name.c_str());
            return;
        }

        pid_t pid = fork();
        if (pid < 0) {
            RCLCPP_ERROR(get_logger(), "fork failed for %s", name.c_str());
            return;
        }

        if (pid == 0) {
            // ── Child process ──────────────────────────────────
            setsid();  // Become the leader of a new session/process group

            vector<char*> argv;
            argv.reserve(argv_vec.size() + 1);
            for (auto& a : argv_vec) argv.push_back(const_cast<char*>(a.c_str()));
            argv.push_back(nullptr);

            execvp(argv[0], argv.data());
            _exit(127);  // exec failed
        }

        // ── Parent process ────────────────────────────────────
        pids_[name] = pid;

        std::ostringstream cmdline;
        for (auto& a : argv_vec) cmdline << a << " ";
        RCLCPP_INFO(get_logger(), "[%s] started (pid=%d): %s", name.c_str(), pid, cmdline.str().c_str());
    }

    void StopProcess(const string& name) {
        ReapFinished();

        auto it = pids_.find(name);
        if (it == pids_.end() || it->second <= 0) {
            RCLCPP_INFO(get_logger(), "[%s] not running", name.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "[%s] stop signal sent (pid=%d)", name.c_str(), it->second);
        StopProcessByPid(it->second);
    }

    void StopProcessByPid(pid_t pid) {
        if (pid > 0) {
            // Since setsid() makes the group leader's pid equal to the group id,
            // killpg sends SIGINT to all child processes (including nodes launched by ros2 launch)
            killpg(pid, SIGINT);
        }
    }

    void ReapFinished() {
        for (auto& kv : pids_) {
            pid_t& pid = kv.second;
            if (pid <= 0) continue;
            int status;
            pid_t res = waitpid(pid, &status, WNOHANG);
            if (res == pid) {
                pid = -1;  // 종료됨
            }
        }
    }

    vector<string> BuildRosbagCommand() {
        auto now = std::time(nullptr);
        std::tm tm_buf;
        localtime_r(&now, &tm_buf);

        std::ostringstream name_oss;
        name_oss << "rosbag2_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");

        const char* home_env = std::getenv("HOME");
        string home = home_env ? home_env : "/tmp";
        string outdir = home + "/bags/" + name_oss.str();

        return {"ros2", "bag", "record", "-a", "-o", outdir};
    }

    void PublishProcessStatus() {
        ReapFinished();

        std::ostringstream oss;
        oss << "{";
        bool first = true;

        auto append_status = [&](const string& name) {
            if (!first) oss << ",";
            first = false;
            auto it = pids_.find(name);
            bool running = (it != pids_.end() && it->second > 0);
            oss << "\"" << name << "\":\"" << (running ? "running" : "stopped") << "\"";
        };

        for (auto& kv : configs_) append_status(kv.first);
        append_status("rosbag");

        oss << "}";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        process_status_pub_->publish(msg);
    }

    // ═══════════════════════════════════════════════════════
    // 2) System monitoring
    // ═══════════════════════════════════════════════════════

    CpuTimes ReadCpuTimes() {
        CpuTimes t{};
        std::ifstream file("/proc/stat");
        if (!file.is_open()) return t;

        string line;
        std::getline(file, line);  // first line: "cpu  user nice system idle iowait irq softirq steal ..."
        std::istringstream iss(line);
        string label;
        iss >> label >> t.user >> t.nice >> t.sys >> t.idle >> t.iowait >> t.irq >> t.softirq >> t.steal;
        return t;
    }

    double ComputeCpuUsage() {
        CpuTimes cur = ReadCpuTimes();

        long prev_idle = prev_cpu_.idle + prev_cpu_.iowait;
        long cur_idle = cur.idle + cur.iowait;
        long prev_total = prev_cpu_.Total();
        long cur_total = cur.Total();

        long total_diff = cur_total - prev_total;
        long idle_diff = cur_idle - prev_idle;

        double usage = 0.0;
        if (total_diff > 0) {
            usage = 100.0 * static_cast<double>(total_diff - idle_diff) / total_diff;
        }
        prev_cpu_ = cur;
        return usage;
    }

    void ReadMemInfo(double& mem_percent, double& used_mb, double& total_mb) {
        mem_percent = used_mb = total_mb = 0.0;

        std::ifstream file("/proc/meminfo");
        if (!file.is_open()) return;

        long mem_total_kb = 0, mem_available_kb = 0;
        string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            string key;
            long value;
            string unit;
            iss >> key >> value >> unit;
            if (key == "MemTotal:")
                mem_total_kb = value;
            else if (key == "MemAvailable:")
                mem_available_kb = value;
        }

        total_mb = mem_total_kb / 1024.0;
        used_mb = (mem_total_kb - mem_available_kb) / 1024.0;
        if (mem_total_kb > 0) {
            mem_percent = 100.0 * (mem_total_kb - mem_available_kb) / mem_total_kb;
        }
    }

    void ReadDiskInfo(double& disk_percent, double& used_gb, double& total_gb) {
        disk_percent = used_gb = total_gb = 0.0;

        struct statvfs st;
        if (statvfs("/", &st) != 0) return;

        const double GB = 1024.0 * 1024.0 * 1024.0;
        unsigned long block_size = st.f_frsize;
        double total = static_cast<double>(st.f_blocks) * block_size;
        double free = static_cast<double>(st.f_bfree) * block_size;
        double used = total - free;

        total_gb = total / GB;
        used_gb = used / GB;
        if (total > 0) {
            disk_percent = 100.0 * used / total;
        }
    }

    double ReadTemperature() {
        // Jetson: /sys/class/thermal/thermal_zone0/temp (millidegree C)
        std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
        if (!file.is_open()) return -1.0;

        long milli_c = 0;
        file >> milli_c;
        return milli_c / 1000.0;
    }

    void PublishSystemStatus() {
        double cpu = ComputeCpuUsage();

        double mem_percent, mem_used_mb, mem_total_mb;
        ReadMemInfo(mem_percent, mem_used_mb, mem_total_mb);

        double disk_percent, disk_used_gb, disk_total_gb;
        ReadDiskInfo(disk_percent, disk_used_gb, disk_total_gb);

        double temp = ReadTemperature();

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1);
        oss << "{"
            << "\"cpu_percent\":" << cpu << ","
            << "\"mem_percent\":" << mem_percent << ","
            << "\"mem_used_mb\":" << mem_used_mb << ","
            << "\"mem_total_mb\":" << mem_total_mb << ","
            << "\"disk_percent\":" << disk_percent << ","
            << "\"disk_used_gb\":" << disk_used_gb << ","
            << "\"disk_total_gb\":" << disk_total_gb << ","
            << "\"temp_celsius\":" << temp << "}";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        system_status_pub_->publish(msg);
    }

    // ═══════════════════════════════════════════════════════
    // 3) Power control (restart / shutdown)
    // ═══════════════════════════════════════════════════════

    void OnSystemCommand(const std_msgs::msg::String::SharedPtr msg) {
        string action;
        if (!ParseField(msg->data, "action", action)) {
            RCLCPP_ERROR(get_logger(), "invalid system command: %s", msg->data.c_str());
            return;
        }

        if (action == "restart") {
            RCLCPP_WARN(get_logger(), "REBOOT requested via GCS");
            std::system("sudo reboot");
        } else if (action == "shutdown") {
            RCLCPP_WARN(get_logger(), "SHUTDOWN requested via GCS");
            std::system("sudo shutdown -h now");
        } else {
            RCLCPP_WARN(get_logger(), "unknown system action: %s", action.c_str());
        }
    }

    // ═══════════════════════════════════════════════════════
    // Common
    // ═══════════════════════════════════════════════════════

    void OnTimer() {
        PublishProcessStatus();
        PublishSystemStatus();
    }

    // Extract a string field from a very simple flat JSON {"key":"value",...}
    bool ParseField(const string& json, const string& key, string& out) {
        string pattern = "\"" + key + "\"";
        auto pos = json.find(pattern);
        if (pos == string::npos) return false;
        pos = json.find(':', pos);
        if (pos == string::npos) return false;
        pos = json.find('"', pos);
        if (pos == string::npos) return false;
        auto end = json.find('"', pos + 1);
        if (end == string::npos) return false;
        out = json.substr(pos + 1, end - pos - 1);
        return true;
    }

    map<string, vector<string>> configs_;
    map<string, pid_t> pids_;
    CpuTimes prev_cpu_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr process_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr process_status_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProcessManagerNode>());
    rclcpp::shutdown();
    return 0;
}
