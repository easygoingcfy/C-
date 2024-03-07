/**
 * @file antwork_updater.cpp
 * @author caofangyu (caofy@antwork.link)
 * @brief
 * @version 0.1
 * @date 2024-03-06
 *
 * Copyright (c) 2015-2024 Xunyi Ltd. All rights reserved.
 *
 */

#include "antwork_updater.h"


AntworkUpdater::AntworkUpdater() {
    ROS_INFO("AntworkUpdater constructor");
    getRosParam("ip", nh_, ip_, std::string("47.96.186.209"));
    getRosParam("port", nh_, port_, 10896);
    msg_ = {
        {"id", 1643},
        {"dev_type", 1},
        {"ack", 0},
        {"msg set", 0},
        {"msg id", 1},
        {"msg data", {{"Model", "SM1B-A"}, {"Platform", "TX2"}}},
    };
    conn_ = ConnInterface::create_client(ip_, port_, "tcp");
    conn_->set_conn_callback(std::bind(&AntworkUpdater::connection_callback, this));
    conn_->set_closed_callback(std::bind(&AntworkUpdater::closed_callback, this));
    conn_->set_receive_callback(
        std::bind(&AntworkUpdater::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    conn_->connect();
}

AntworkUpdater::~AntworkUpdater() { ROS_INFO("AntworkUpdater destructor"); }

void AntworkUpdater::run() {
    ROS_INFO("AntworkUpdater run");
    std::thread run_thread(&ConnInterface::run, conn_);
    run_thread.detach();


    conn_->run_every(5000, std::bind(&AntworkUpdater::send_heartbeat, this));
}

void AntworkUpdater::connection_callback() { ROS_INFO("AntworkUpdater connect succeess!"); }

void AntworkUpdater::closed_callback() { ROS_INFO("AntworkUpdater closed!"); }

void AntworkUpdater::receive_callback(char *data, size_t len) {
    // 接收的4字节数据长度没有意义，略过
    if (len == 4) {
        ROS_INFO_STREAM("receive data len: " << len);
        return;
    }
    std::string data_str(data, len);
    ROS_INFO("AntworkUpdater receive data: %s", data_str.c_str());
    json msg;
    try {
        msg = json::parse(data_str);
    } catch (json::parse_error &e) {
        ROS_ERROR("AntworkUpdater parse error: %s", e.what());
        return;
    }
    short total_id = (short)((msg["msg set"].get<int>() << 8) | msg["msg id"].get<int>());
    ROS_INFO("msg total id : %d", total_id);
    switch (total_id) {
        case 0x0100:
            send_log_tree();
            break;
        default:
            break;
    }
}

void AntworkUpdater::send_heartbeat() {
    msg_["ack"] = 0;
    msg_["msg set"] = 0;
    msg_["msg id"] = 9;
    msg_.erase("msg data");
    send_to_cloud(msg_.dump());
}


void AntworkUpdater::send_to_cloud(const std::string &msg) {
    int len = msg.length() + 4;
    conn_->send_bytes(&len, 4);
    conn_->send_message(msg);
    ROS_DEBUG_STREAM("AntworkUpdater send to cloud: " << msg);
}

/**
 * @brief  依照旧逻辑发送设备信息，从自测看，非必要
 * 
 */
void AntworkUpdater::send_info() {
    ROS_INFO("send info");
    json msg_data;

    msg_data["Model"] = "SM1B-A";
    msg_data["Platform"] = "TX2";
    int msg_id = 1;
    msg_["msg data"] = msg_data;
    msg_["msg id"] = msg_id;
    send_to_cloud(msg_.dump());

    msg_data.clear();
    msg_data["Version"] = "TX2-test-0301-7-g9b719e3";
    msg_id = 3;
    msg_["msg data"] = msg_data;
    msg_["msg id"] = msg_id;
    send_to_cloud(msg_.dump());

    msg_data.clear();
    msg_data["Status"] = 0;
    msg_id = 5;
    msg_["msg data"] = msg_data;
    msg_["msg id"] = msg_id;
    send_to_cloud(msg_.dump());
}

void AntworkUpdater::send_log_tree() {
    ROS_INFO("send log tree");
    json dir_info;
    dir_info["Tree"] = json({});
    dir_info["Tree"]["log"] = get_dir("/root/Antwork/ws/log");
    std::cout << dir_info["Tree"]["log"].dump(4) << std::endl;
    msg_["msg set"] = 1;
    msg_["msg id"] = 1;
    msg_["msg data"] = dir_info;
    send_to_cloud(msg_.dump());
}

/**
 * @brief 文件夹用json表示,同级文件放在列表里
 * 
 * @param dir_path 
 * @return json::array
 */
nlohmann::json AntworkUpdater::get_dir(const std::string &dir_path) {
    path p(dir_path);
    if (!is_directory(p)) {
        ROS_WARN("dir_path: %s is not a directory", dir_path.c_str());
        return json();
    }
    json dir_list = json::array();
    for (auto&& it : directory_iterator(p)) {
        if (is_symlink(it.path())) {
            continue;
        }
        if (is_regular_file(it.path())) {
            dir_list.emplace_back(it.path().filename().c_str());
        } else if (is_directory(it.path())) {
            json sub_dir = {{it.path().filename().c_str(), get_dir(it.path().string())}};
            dir_list.emplace_back(sub_dir);

        }
    }
    return dir_list;
}

void AntworkUpdater::print_json(json &j) {
    std::cout << j.dump(4) << std::endl;
}