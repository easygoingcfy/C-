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

#include <bitset>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <boost/filesystem.hpp>
#include <ros/ros.h>

#include "new_updater/conn/interface.h"
#include "new_updater/json.hpp"
#include "new_updater/util.h"

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

    // member variables
    ros::NodeHandle nh_;
    std::shared_ptr<flight_brain::mountable::ConnInterface> conn_;
    std::string ip_;
    int port_;

    json msg_;
};