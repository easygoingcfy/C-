/**
 * @file antwork_updater.h
 * @author caofangyu (caofy@antwork.link)
 * @brief
 * @version 0.1
 * @date 2024-03-06
 *
 * Copyright (c) 2015-2024 Xunyi Ltd. All rights reserved.
 *
 */

#pragma once

#include <sys/stat.h>

#include <bitset>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <curl/curl.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <define.h>
#include "conn/interface.h"
#include "json.hpp"
#include "pugixml.hpp"
#include "util.h"

using namespace boost::filesystem;
using namespace flight_brain::mountable;

class AntworkUpdater {
   public:
    using json = nlohmann::json;

    AntworkUpdater();   // Constructor
    ~AntworkUpdater();  // Destructor
    void run();

    size_t write_data(void *ptr, size_t size, size_t nmemb, FILE *stream);
    // static size_t write_data_wrapper(void *ptr, size_t size, size_t nmemb, FILE *stream);

    int progress_callback(void *clientp, double dltotal, double dlnow, double ultotal, double ulnow);
    // static int progress_callback_wrapper(void *clientp, double dltotal, double dlnow, double ultotal, double ulnow);

   private:
    void connection_callback();
    void closed_callback();
    void receive_callback(char *data, size_t len);

    /**
     * @brief 从cache/link.info中解析ip和port
     *
     */
    void set_ip_and_port();

    void parse_install_info();
    void parse_hardware_xml();
    void parse_update_config();

    /**
     * @brief 从/etc/xyi/comm_env.xml中解析url
     *
     */
    void set_upload_url();

    void send_info();

    /**
     * @brief 按照协议发送数据到云端
     *
     * 每个消息由两部分组成：
        数据长度：4字节，表示整个消息的长度，包括数据长度字段本身(4)和JSON字符串数据的长度。
        数据：一个JSON字符串，包含消息的具体内容。
     *
     * @param msg
     */
    void send_to_cloud(const std::string &msg);

    void send_log_tree();

    /**
     * @brief 打印json数据，调试用
     *
     * @param j
     */
    void print_json(json &j, const std::string &str = "");

    /**
     * @brief 文件夹用json表示,同级文件放在列表里
     *
     * @param dir_path
     * @return json::array
     */
    json get_dir(const std::string &dir_path);

    /**
     * @brief 0x0000
     *
     */
    void report_authenticate_information();
    /**
     * @brief 0x0002
     *
     */
    void report_version_info();
    /**
     * @brief 0x0004
     *
     */
    void report_device_status();
    /**
     * @brief 0x0006
     *
     */
    void report_config_of_update();
    /**
     * @brief 0x0008
     *
     */
    void set_config_of_update(json &msg);
    /**
     * @brief
     *
     * @param return_code : 0 - success
     *                      1 - failed
     */
    void report_result_of_set_config(Res return_code);
    /**
     * @brief 0x000A
     *
     */
    void update_firmware(json &msg);
    void report_result_of_update_firmware(UpdateFirmwareRes return_code);

    void report_progress_of_update(double percent, double dl_speed);
    void report_status_of_update(UpdateStatus status);
    void report_heartbeat();
    void report_result_of_change_id(Res return_code);

    void handle_message_0000(json &msg);
    void handle_message_0002(json &msg);
    void handle_message_0102(json &msg);
    /**
     * @brief handle msg 0x0104
     *
     * @param msg
     */
    void clear_log(json &msg);

    /**
     * @brief  handle msg 0x0106
     *
     * @param msg
     */
    void change_id(json &msg);

    /**
     * @brief 使用curl上传文件
     * @info 备份-上传-删除
     *
     * @param file_path
     * @param url
     * @param name  表单字段的值
     * @return true
     * @return false
     */
    bool uplod_file(const std::string &file_path, const std::string &url, const std::string &name);

    // util func
    /**
     * @brief 解析固件包更新的URL
     *
     */
    bool parse_firmware_url();

    void download_file_by_curl(const std::string &url, const std::string &file_path);


    /**
     * @brief 使用curl对URL进行编码，处理非ASCII字符
     *
     * @param url
     * @return std::string
     */
    std::string encode_url(const std::string &url);

    // member variables
    ros::NodeHandle nh_;
    std::shared_ptr<flight_brain::mountable::ConnInterface> conn_;
    std::string ip_;
    int port_;
    // file path  TODO(caofy): avoid hard-coding
    std::string ws_path_ = "/root/Antwork/ws";
    std::string link_info_path_ = "cache/link.info";
    std::string comm_env_path_ = "/etc/xyi/comm_env.xml";
    std::string install_info_path_ = "/etc/xyi/install.info";
    std::string hardware_xml_path_ = "/etc/xyi/hardware.xml";
    std::string update_config_path_ = "/root/Antwork/ws/config/updater/config.json";

    std::string upload_url_ = "http://antlink-sandbox.xyitech.com/uploader/oss/upload";

    // app data
    // TODO(caofy): 快速开发，写在同一个类中，后续考虑使用单独的app data类进行管理
    // note: 原版本中，app data只在启动的时候读取了一次, 先沿用这套逻辑
    double firmware_size_;
    std::string firmware_name_;  // 升级包的包名
    std::string firmware_url_;   // 升级包的url
    std::string software_version_;
    std::string installation_path_;
    std::string device_platform_;  // Device platform type: TX2, XU4, IPC, IPC2, etc.
    std::string device_model_;     // Device model type: RA3C, etc.
    int device_status_;
    int device_family_;  // Device family: 0 - Reserved, 1 - UAV, 2 - UAP, 3 - SRC, 4 - Autonomous Vehicle
    int device_id_;      // Device ID
    std::string serial_num_;
    // /root/Antwork/ws/config/updater/config.json
    json update_config_;
    std::string update_policy_str_;
    UpdatePolicy update_policy_;

    std::function<size_t(void *, size_t, size_t, FILE *)> write_callback_;
    std::function<int(void *, double, double, double, double)> progress_callback_;

    json msg_;

    CURL* curl_;
    // last report time
    std::chrono::time_point<std::chrono::system_clock> last_report_time_;
};