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

#include "pugixml.hpp"
#include "conn/interface.h"
#include "json.hpp"
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

    /**
     * @brief 从/etc/xyi/comm_env.xml中解析url
     * 
     */
    void set_upload_url();

    void send_heartbeat();
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
    std::string ws_path_ = "/root/Antwork/ws";
    std::string link_info_path_ = "cache/link.info";
    std::string comm_env_path_ = "/etc/xyi/comm_env.xml";
    std::string upload_url_ = "http://antlink-sandbox.xyitech.com/uploader/oss/upload";

    json msg_;
};