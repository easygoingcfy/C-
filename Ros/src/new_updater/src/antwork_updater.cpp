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

AntworkUpdater::AntworkUpdater() : device_status_(0) {
    ROS_INFO("AntworkUpdater constructor");

    parse_update_config();
    std::ofstream f("/tmp/update_config.json");
    f << update_config_.dump(4);
    exit(1);
    parse_install_info();
    parse_hardware_xml();
    set_ip_and_port();
    set_upload_url();

    msg_ = {
        {"id", device_id_},
        {"dev_type", device_family_},
        {"ack", 0},
        {"msg set", 0},
        {"msg id", 1},
        {"msg data", {{"Model", device_model_.c_str()}, {"Platform", device_platform_.c_str()}}},
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

    conn_->run_every(500, std::bind(&AntworkUpdater::report_heartbeat, this));
}

void AntworkUpdater::set_ip_and_port() {
    path abs_link_path = path(ws_path_) / path(link_info_path_);
    std::ifstream f(abs_link_path.string());
    try {
        json link_info = json::parse(f);
        print_json(link_info);
        ip_ = link_info["msg"]["update"]["ip"].get<std::string>();
        port_ = std::stoi(link_info["msg"]["update"]["port"].get<std::string>());
    } catch (std::exception &e) {
        ROS_ERROR("parse link_info error : %s, use default ip and port!", e.what());
        ip_ = "47.96.186.209";
        port_ = 10896;
    }
    ROS_INFO_STREAM("ip: " << ip_ << " port: " << port_);
}

void AntworkUpdater::set_upload_url() {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(comm_env_path_.c_str());
    if (!result) {
        ROS_ERROR("load comm_env.xml error: %s", result.description());
        ROS_INFO("use default upload_url: %s", upload_url_.c_str());
        return;
    }

    pugi::xml_node root = doc.child("root");
    pugi::xml_node upload = root.child("file_upload_rpc");
    pugi::xml_attribute url = upload.attribute("upload_dir");
    upload_url_ = url.as_string();
    ROS_INFO_STREAM("upload_url: " << upload_url_);
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
        case 0x0000:
            report_authenticate_information();
        case 0x0002:
            report_version_info();
        case 0x0004:
            report_device_status();
        case 0x0006:
            report_config_of_update();
        case 0x0008:
            set_config_of_update(msg);
        case 0x0100:
            send_log_tree();
            break;
        case 0x0102:
            handle_message_0102(msg);
        default:
            break;
    }
}

void AntworkUpdater::send_to_cloud(const std::string &msg) {
    int len = msg.length() + 4;
    conn_->send_bytes(&len, 4);
    conn_->send_message(msg);
    ROS_DEBUG_STREAM("AntworkUpdater send to cloud: " << msg);
}

void AntworkUpdater::report_heartbeat() {
    msg_["ack"] = static_cast<int>(Ack::NotAck);
    msg_["msg set"] = static_cast<int>(MsgSet::Update);
    msg_["msg id"] = static_cast<int>(MsgId::ReportHeartbeat);
    msg_.erase("msg data");
    send_to_cloud(msg_.dump());
}

void AntworkUpdater::report_authenticate_information() {
    msg_["ack"] = static_cast<int>(Ack::NotAck);
    msg_["msg set"] = static_cast<int>(MsgSet::Update);
    msg_["msg id"] = static_cast<int>(MsgId::ReportAuthenticateInformation);
    msg_["msg data"] = {{"Model", device_model_.c_str()}, {"Platform", device_platform_.c_str()}};
    send_to_cloud(msg_.dump());
}

void AntworkUpdater::report_version_info() {
    msg_["ack"] = static_cast<int>(Ack::NotAck);
    msg_["msg set"] = static_cast<int>(MsgSet::Update);
    msg_["msg id"] = static_cast<int>(MsgId::ReportVersionInfo);
    msg_["msg data"] = {{"Version", software_version_}};
    send_to_cloud(msg_.dump());
}

void AntworkUpdater::report_device_status() {
    msg_["ack"] = static_cast<int>(Ack::NotAck);
    msg_["msg set"] = static_cast<int>(MsgSet::Update);
    msg_["msg id"] = static_cast<int>(MsgId::ReportDeviceStatus);
    msg_["msg data"] = {{"Status", device_status_}};
    send_to_cloud(msg_.dump());
}

void AntworkUpdater::report_config_of_update() {
    msg_["ack"] = static_cast<int>(Ack::NotAck);
    msg_["msg set"] = static_cast<int>(MsgSet::Update);
    msg_["msg id"] = static_cast<int>(MsgId::ReportConfigOfUpdate);
    msg_["msg data"] = {
        {"Policy", static_cast<int>(update_policy_)},
        {"Open Time", update_config_["Open Time"].get<double>()},
        {"Close Time", update_config_["Close Time"].get<double>()},
    };
    send_to_cloud(msg_.dump());
}

void AntworkUpdater::set_config_of_update(json &msg) {
    if (!msg.contains("msg data") || !msg["msg data"].contains("Policy") || !msg["msg data"]["Policy"].is_number()) {
        ROS_ERROR("msg data error");
        return;
    }
    // set policy
    try {
        int policy = msg["msg data"]["Policy"].get<int>();
        if (policy < 0 || policy > 2) {
            ROS_ERROR("update_policy_ error: %d", update_policy_);
            return;
        }
        update_policy_ = static_cast<UpdatePolicy>(policy);
        switch(update_policy_) {
            case UpdatePolicy::Auto:
                update_policy_str_ = "Auto";
                break;
            case UpdatePolicy::Manual:
                update_policy_str_ = "Manual";
                break;
            case UpdatePolicy::All:
                update_policy_str_ = "All";
                break;
        }
        update_config_["Policy"] = update_policy_str_;
    } catch (std::exception &e) {
        ROS_ERROR("set policy error: %s", e.what());
        return;
    }

    // set time
    try {
        int open_time = msg["msg data"]["Open Time"].get<double>();
        int close_time = msg["msg data"]["Close Time"].get<double>();
    } catch (std::exception &e) {
        ROS_ERROR("get time error: %s", e.what());
        return;
    }

    if (open_time > 24 || open_time < 0 || close_time > 24 || close_time < 0) {
        ROS_ERROR("open_time or close_time error: %d %d", open_time, close_time);
        return;
    }

    update_config_["Open Time"] = open_time;
    update_config_["Close Time"] = close_time;
    print_json(update_config_);
    std::ofstream f(update_config_path_);
    if (!file) {
        ROS_ERROR("open update_config.json error");
        return;
    }
    file << update_config_.dump(4);
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
    msg_["msg data"] = msg_data;
    msg_["msg id"] = static_cast<int>(MsgId::ReportAuthenticateInformation);
    send_to_cloud(msg_.dump());

    msg_data.clear();
    msg_data["Version"] = "TX2-test-0301-7-g9b719e3";
    msg_["msg data"] = msg_data;
    msg_["msg id"] =static_cast<int>(MsgId::ReportVersionInfo);
    send_to_cloud(msg_.dump());

    msg_data.clear();
    msg_data["Status"] = 0;
    msg_["msg data"] = msg_data;
    msg_["msg id"] = static_cast<int>(MsgId::ReportDeviceStatus);
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
    for (auto &&it : directory_iterator(p)) {
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

void AntworkUpdater::print_json(json &j) { ROS_INFO_STREAM(j.dump(4)); }

bool AntworkUpdater::uplod_file(const std::string &file_path, const std::string &url, const std::string &name) {
    CURL *curl;
    CURLcode res;

    // Create a backup file
    std::string backup_file_path = file_path + ".bak";
    std::ifstream src(file_path, std::ios::binary);
    std::ofstream dst(backup_file_path, std::ios::binary);
    dst << src.rdbuf();
    if (!src.good() || !dst.good()) {
        ROS_ERROR("Failed to create backup file '%s'", backup_file_path.c_str());
        return false;
    }

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if (!curl) {
        ROS_ERROR("curl_easy_init() failed");
        return false;
    }

    // 设置表单数据，file和name字段
    curl_mime *form = curl_mime_init(curl);
    curl_mimepart *field = curl_mime_addpart(form);
    curl_mime_name(field, "file");
    curl_mime_filedata(field, backup_file_path.c_str());

    field = curl_mime_addpart(form);
    curl_mime_name(field, "name");
    curl_mime_data(field, name.c_str(), CURL_ZERO_TERMINATED);

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        ROS_ERROR("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
        return false;
    }

    curl_easy_cleanup(curl);
    curl_mime_free(form);

    // Delete the backup file
    if (remove(backup_file_path.c_str()) != 0) {
        ROS_ERROR("Error deleting file %s\n", backup_file_path.c_str());
    }

    curl_global_cleanup();
    return true;
}


void AntworkUpdater::handle_message_0002(json &msg) {
    ROS_INFO("handle_message_0002");
    json msg_data;
    msg["ack"] = 0;
    msg_data["Version"] = software_version_;
    msg["msg data"] = msg_data;
    send_to_cloud(msg.dump());
}

void AntworkUpdater::handle_message_0102(json &msg) {
    std::string file_path = msg["msg data"]["Filename"].get<std::string>();
    path abs_path = path(ws_path_) / path(file_path);
    std::cout << "abs_path: " << abs_path.string() << std::endl;
    bool res = uplod_file(abs_path.string(), upload_url_, file_path);
    msg["ack"] = 1;
    if (res) {
        msg["msg data"]["Return Code"] = 3;
    } else {
        msg["msg data"]["Return Code"] = 2;
    }
    print_json(msg);
    send_to_cloud(msg.dump());
}

void AntworkUpdater::parse_install_info() {
    std::ifstream file(install_info_path_);
    if (!file) {
        ROS_ERROR("open install_info.json error");
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key, equal, value;
        if (std::getline(iss, key, "=") && std::getline(iss, value)) {
            ROS_INFO_STREAM("key: " << key << " value: " << value);
            if (key == "VERSION") software_version_ = value;
            if (key == "PLATFORM") device_platform_ = value;
            if (key == "INSTALLATION_PATH") installation_path_ = value;
        }
    }
    ROS_INFO_STREAM("software_version_: " << software_version_ << std::endl
                                          << "device_platform_: " << device_platform_ << std::endl
                                          << "installation_path_: " << installation_path_);
}

void AntworkUpdater::parse_hardware_xml() {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(hardware_xml_path_.c_str());
    if (!result) {
        ROS_ERROR("load hardware.xml error: %s", result.description());
        return;
    }
    pugi::xml_node root = doc.child("root");

    pugi::xml_node flight_id = root.child("flight_ID");
    pugi::xml_node airport_id = root.child("airport_ID");

    device_family_ = 0;
    if (!root.child("flight_ID").empty()) {
        device_id_ = root.child("flight_ID").first_attribute().as_int();
        device_family_ = 1;
    } else if (!root.child("airport_ID").empty()) {
        device_id_ = root.child("airport_ID").first_attribute().as_int();
        device_family_ = 2;
    } else if (!root.child("ugv_ID").empty()) {
        device_id_ = root.child("ugv_ID").first_attribute().as_int();
        device_family_ = 3;
    } else if (!root.child("ground_station_info").empty()) {
        device_id_ = root.child("ground_station_info").first_attribute().as_int();
        device_family_ = 4;
    } else {
        ROS_ERROR("device_family_ is 0, hardware.xml error");
    }

    pugi::xml_node varient_type = root.child("varient_type");
    device_model_ = varient_type.attribute("type").as_string();

    pugi::xml_node serial_num = root.child("serial_num");
    serial_num_ = serial_num.attribute("number").as_string();

    ROS_INFO_STREAM("device_family_: " << device_family_ << std::endl
                                       << "device_id_: " << device_id_ << std::endl
                                       << "device_model_: " << device_model_ << std::endl
                                       << "serial_num_: " << serial_num_);
}

void AntworkUpdater::parse_update_config() {
    std::ifstream f(update_config_path_);
    try {
        update_config_ = json::parse(f);
        print_json(update_config);
        update_policy_str_ = update_config["Policy"].get<std::string>();
        if (update_policy_str_ == "Auto") {
            update_policy_ = UpdatePolicy::Auto;
        } else if (update_policy_str_ == "Manual") {
            update_policy_ = UpdatePolicy::Manual;
        } else if (update_policy_str_ == "All") {
            update_policy_ = UpdatePolicy::All;
        } else {
            ROS_ERROR("update_policy_str error: %s", update_policy_str_);
        }
    } catch (std::exception &e) {
        ROS_ERROR("parse update_config error : %s", e.what());
    }
    print_json(update_config_);
}