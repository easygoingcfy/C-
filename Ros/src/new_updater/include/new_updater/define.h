/**
 * @file define.h
 * @brief 
 * @version 0.1
 * @date 2024-03-08
 * 
 * Copyright (c) 2015-2024 Xunyi Ltd. All rights reserved.
 * 
 */

#pragma once

enum class UpdatePolicy {
    Auto = 0,
    Manual = 1,
    All = 2
};

enum class Ack {
    NotAck = 0,
    Ack = 1
};

enum class Res {
    Success = 0,
    Fail = 1
};

enum class UpdateStatus {
    Downloading = 0,
    DownloadFailed = 1,
    DownloadSuccessfully = 2,
};

enum class UpdateFirmwareRes {
    Success = 0,
    Reject = 1,
    ParamError = 2
};

enum class MsgSet {
    Update = 0x00,
    Other = 0x01
};

enum class MsgId {
    QueryAuthenticateInformation = 0x00,
    ReportAuthenticateInformation = 0x01,
    QueryVersionInfo = 0x02,
    ReportVersionInfo = 0x03,
    QueryDeviceStatus = 0x04,
    ReportDeviceStatus = 0x05,
    QueryConfigOfUpdate = 0x06,
    ReportConfigOfUpdate = 0x07,
    RequestToSetConfigOfUpdate = 0x08,
    ReportHeartbeat = 0x09,
    RequestToUpdateFirmware = 0x0A,
    ReportProgressOfUpdate = 0x0B,
    Reboot = 0x0C,
    ReportStatusOfUpdate = 0x0D,
};

enum class OtherMsgId {
    QueryLogTree = 0x00,
    ReportLogTree = 0x01,
    RequestFile = 0x02,
    Reserved1 = 0x03,
    RequestToClearLogs = 0x04,
    Reserved2 = 0x05,
    RequestToChangeId = 0x06
};