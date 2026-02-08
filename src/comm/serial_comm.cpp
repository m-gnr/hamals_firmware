#include "serial_comm.h"

void SerialComm::begin(unsigned long baud) {
    (void)baud;   
    rx_len_ = 0;
    last_cmd_.valid = false;
}

void SerialComm::update() {
    while (Serial.available()) {
        char c = Serial.read();
        processChar(c);
    }
}

void SerialComm::processChar(char c) {
    if (c == '\n' || c == '\r') {
        if (rx_len_ > 0) {
            rx_buf_[rx_len_] = '\0';
            parseLine(rx_buf_);
            rx_len_ = 0;
        }
        return;
    }

    if (rx_len_ < RX_BUF_SIZE - 1) {
        rx_buf_[rx_len_++] = c;
    } else {
        // overflow
        rx_len_ = 0;
    }
}

void SerialComm::parseLine(const char* line) {
    // cmd,v,w
    if (strncmp(line, "cmd,", 4) == 0) {
        float v, w;
        if (sscanf(line + 4, "%f,%f", &v, &w) == 2) {
            last_cmd_.v = v;
            last_cmd_.w = w;
            last_cmd_.valid = true;
        }
    }
}

bool SerialComm::hasCmdVel() const {
    return last_cmd_.valid;
}

CmdVel SerialComm::getCmdVel() {
    last_cmd_.valid = false;   // consume
    return last_cmd_;
}

void SerialComm::sendOdom(float x, float y, float yaw,
                          float v, float w) {
    Serial.print("odom,");
    Serial.print(x, 3);   Serial.print(',');
    Serial.print(y, 3);   Serial.print(',');
    Serial.print(yaw, 3); Serial.print(',');
    Serial.print(v, 3);   Serial.print(',');
    Serial.print(w, 3);
    Serial.print('\n');
}