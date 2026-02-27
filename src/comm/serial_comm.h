#pragma once
#include <Arduino.h>
#include <cstddef>

// -------------------- DATA -----------------------------
struct CmdVel {
    float v = 0.0f;
    float w = 0.0f;
    bool  valid = false;
};

// -------------------- CLASS ----------------------------
class SerialComm {
public:
    // Note: Serial.begin(baud) is done in main; this just resets internal state
    void begin(unsigned long baud);
    void update();

    bool hasCmdVel() const;
    CmdVel getCmdVel();   // returns last cmd and clears valid flag

    void sendOdom(uint32_t t_us,
                  float x, float y, float yaw,
                  float v, float w);

private:
    static constexpr size_t RX_BUF_SIZE = 128;

    char   payload_[RX_BUF_SIZE]{};
    size_t payload_len_ = 0;

    CmdVel last_cmd_;

    // Internal helpers
    void processChar(char c);
    void parsePayload(const char* payload);


};

