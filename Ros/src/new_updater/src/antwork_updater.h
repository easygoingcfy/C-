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

    // Add your member functions here

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
    void print_json(json &j);

    /**
     * @brief 文件夹用json表示,同级文件放在列表里
     *
     * @param dir_path
     * @return json::array
     */
    json get_dir(const std::string &dir_path);

    void report_authenticate_information();
    void report_version_info();
    void report_device_status();
    void report_config_of_update();
    void set_config_of_update(json &msg);
    void report_heartbeat();

    void handle_message_0000(json &msg);
    void handle_message_0002(json &msg);
    void handle_message_0102(json &msg);

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
    double fireware_size_;
    std::string firmware_name_;  // 升级包的包名
    std::string url_;            // 升级包的url
    std::string software_version_;
    std::string installation_path_;
    std::string device_platform_;  // Device platform type: TX2, XU4, IPC, IPC2, etc.
    std::string device_model_;     // Device model type: RA3C, etc.
    int device_status_;
    int device_family_;            // Device family: 0 - Reserved, 1 - UAV, 2 - UAP, 3 - SRC, 4 - Autonomous Vehicle
    int device_id_;                // Device ID
    std::string serial_num_;
    // /root/Antwork/ws/config/updater/config.json
    json update_config_;
    std::string update_policy_str_;
    UpdatePolicy update_policy_;


    json msg_;
};