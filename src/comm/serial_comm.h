#pragma once
#include <Arduino.h>

// -------------------- DATA -----------------------------
struct CmdVel {
    float v = 0.0f;
    float w = 0.0f;
    bool  valid = false;
};

// -------------------- CLASS ----------------------------
class SerialComm {
public:
    void begin(unsigned long baud);
    void update();

    bool hasCmdVel() const;
    CmdVel getCmdVel();

    void sendOdom(float x, float y, float yaw,
                  float v, float w);

private:
    static constexpr size_t RX_BUF_SIZE = 64;
    char rx_buf_[RX_BUF_SIZE];
    size_t rx_len_ = 0;

    CmdVel last_cmd_;

    void processChar(char c);
    void parseLine(const char* line);
};