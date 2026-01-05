// src/test.cpp
#include <iostream>
#include "mavlink/ardupilotmega/mavlink.h"

int main(int argc, char **argv) {
    // 1. Dòng này để báo với trình biên dịch là "tôi biết biến này không dùng, đừng báo lỗi"
    (void)argc; 
    (void)argv;

    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    
    // Giả lập gán dữ liệu
    heartbeat.type = MAV_TYPE_GCS;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    // 2. Gọi hàm encode để thực sự "sử dụng" biến heartbeat và msg
    // Hàm này sẽ đóng gói dữ liệu từ heartbeat vào msg
    mavlink_msg_heartbeat_encode(1, 200, &msg, &heartbeat);

    std::cout << "MAVLink library included and linked successfully!" << std::endl;
    std::cout << "Message ID created: " << (int)msg.msgid << std::endl;
    
    return 0;
}