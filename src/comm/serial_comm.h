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

    void sendOdom(uint32_t t_us,
                  float x, float y, float yaw,
                  float v, float w);

private:
    static constexpr size_t RX_BUF_SIZE = 128;

    char payload_[RX_BUF_SIZE];
    size_t payload_len_ = 0;

    bool in_frame_ = false;

    // Last received command
    CmdVel last_cmd_;

    // Internal helpers
    void processChar(char c);
    void parsePayload(const char* payload);
};