/* 品灵相机协议：
     * 1、如果采用串口通信协议，协议形如：串口头部 + 数据体 + 串口校验和
     * 2、如果采用网络通信，协议形如：网络头部 + 串口头部 + 数据体 + 串口校验和 + 网络校验和
    */
#include <stdint.h>
#include <string.h>
#include <iostream>

    #pragma pack(1)
    typedef struct
    {
        uint8_t header1 = 0x55;
        uint8_t header2 = 0xaa;
        uint8_t header3 = 0xdc;
        uint8_t len;
        uint8_t cmd_id;
    }SerialPortHeader;

    typedef struct
    {
        uint8_t header1 = 0xeb;
        uint8_t header2 = 0x90;
        uint8_t len;
    }NetHeader;

    typedef struct
    {
        uint8_t servo_ctrl  = 0x0f; // 默认是不改变伺服状态：0x0f
        int16_t ctrl_param1 = 0x00;
        int16_t ctrl_param2 = 0x00;
        int16_t ctrl_param3 = 0x00;
        int16_t ctrl_param4 = 0x00;
    }A1Pack;

    typedef struct
    {
        struct
        {
            uint8_t servo_ctrl_cmd : 5;
            uint8_t feedback_ctrl  : 1;
            uint8_t frame_cnt      : 2;
        };
        int8_t adjustable_quantity; // 调整量
    }A2Pack;

    typedef struct
    {
        struct
        {
            uint8_t roll_degree_4 : 4; // 滚转角高4位
            uint8_t servo_state   : 4;
        };
        uint8_t roll_degree; // 滚转角低8位
        int16_t yaw_degree;
        int16_t pitch_degree;
    }B1Pack;

    typedef struct
    {
        uint16_t sensor           : 3; // 使用哪种传感器
        uint16_t zoom_speed       : 3; // 变焦/聚焦 速度
        uint16_t camera_cmd       : 7;
        uint16_t measure_distance : 3; // 此款相机无此功能，保留
    }C1Pack; // c1 包位光学控制包，和相机相关，例如拍照，录像，变焦......

    typedef struct
    {
        uint8_t  cmd;
        uint16_t param1;
    }C2Pack;

    typedef struct
    {
        uint8_t reserve1;
        uint8_t reserve2;
        // struct
        // {
        //     uint16_t shot_state : 2; // 录像状态
        //     uint16_t reserve3   : 4;
        //     uint16_t reserve4   : 10;
        // };
        uint16_t state;
        uint16_t reserve5;
        uint32_t reserve6;
        uint16_t zoom_times; // 放大倍数
    }D1Pack;

    typedef struct
    {
        struct
        {
            uint8_t track_src : 3; // 跟踪源
            uint8_t param1    : 5;
        };
        uint8_t track_cmd ; // 跟踪指令
        uint8_t param2;
    }E1Pack;

    typedef struct
    {
        uint8_t  cmd ; // 扩展指令1
        uint16_t param1;
        uint16_t param2;
    }E2Pack;

    typedef struct
    {
        uint8_t  track_state;
    }F1Pack;

    typedef struct
    {
        uint8_t  gps_info[22];
    }T1Pack;

    typedef struct
    {
        NetHeader        net_head;
        SerialPortHeader serial_head;
        A1Pack           a1_pack;
        C1Pack           c1_pack;
        E1Pack           e1_pack;
        uint8_t          serial_checksum; // 串口校验和
        uint8_t          net_checksum; // 网络校验和
    }CombinationControlA; // A组合控制

void initCombinationCtrlACmd(CombinationControlA& combination_ctrl)
{
    memset((void*)&combination_ctrl, 0, sizeof(CombinationControlA));
    combination_ctrl.serial_head.len = 0x11;
    combination_ctrl.serial_head.cmd_id = 0x30;
    combination_ctrl.serial_head.header1 = 0x55;
    combination_ctrl.serial_head.header2 = 0xaa;
    combination_ctrl.serial_head.header3 = 0xdc;
    combination_ctrl.net_head.len = 0x14;
    combination_ctrl.net_head.header1 = 0xeb;
    combination_ctrl.net_head.header2 = 0x90;
    combination_ctrl.a1_pack.servo_ctrl = 0x0f;
}

void swap16(uint16_t *val)
{
    *val = ((*val & 0xff) << 8) | (*val >> 8);
}

uint8_t serialProtocolChecksum(uint8_t* buf)
{
    uint8_t len = buf[3];
    uint8_t checksum = len;
    for(uint8_t i = 0; i < len - 2; i++)
    {
        checksum = checksum ^ buf[4 + i];
    }
    return checksum;
}

// 网络协议校验和
uint8_t netProtocolChecksum(uint8_t* buf, size_t len)
{
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    uint8_t checksum = sum & 0x00ff;
    return checksum;
}

// 一键朝下
CombinationControlA get_cmd()
{
    std::cout << "get cmd" << std::endl;
    CombinationControlA combination_ctrl;
    initCombinationCtrlACmd(combination_ctrl);
    combination_ctrl.a1_pack.servo_ctrl = 0x0b;
    combination_ctrl.a1_pack.ctrl_param1 = 0;
    combination_ctrl.a1_pack.ctrl_param2 = 90 * (65536 / 360);
    swap16((uint16_t*)&combination_ctrl.a1_pack.ctrl_param1); // 大端对齐
    swap16((uint16_t*)&combination_ctrl.a1_pack.ctrl_param2); // 大端对齐

    std::cout << "return cmd" << std::endl;
    combination_ctrl.c1_pack.camera_cmd = 0;
    swap16((uint16_t*)&combination_ctrl.c1_pack); // 大端对齐
    combination_ctrl.serial_checksum = serialProtocolChecksum((uint8_t*)&combination_ctrl.serial_head);
    combination_ctrl.net_checksum = netProtocolChecksum((uint8_t*)&combination_ctrl.serial_head, combination_ctrl.net_head.len);
    return combination_ctrl;
}