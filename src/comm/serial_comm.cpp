#include "serial_comm.h"
#include <string.h>

// -------------------- PARSER STATE --------------------

namespace {
    enum class ParseState {
        WAIT_START,
        READ_PAYLOAD,
        READ_CHECKSUM_1,
        READ_CHECKSUM_2
    };
}

// -------------------- INTERNAL STATE ------------------

static ParseState state_ = ParseState::WAIT_START;
static uint8_t calc_checksum_ = 0;
static uint8_t recv_checksum_ = 0;

// -------------------- UTILS ---------------------------

static uint8_t hexToByte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// -------------------- SERIAL COMM ---------------------

void SerialComm::begin(unsigned long baud) {
    (void)baud;
    last_cmd_.valid = false;
}

void SerialComm::update() {
    while (Serial.available()) {
        char c = Serial.read();
        processChar(c);
    }
}

void SerialComm::processChar(char c) {
    switch (state_) {

        case ParseState::WAIT_START:
            if (c == '$') {
                this->payload_len_ = 0;
                calc_checksum_ = 0;
                state_ = ParseState::READ_PAYLOAD;
            }
            break;

        case ParseState::READ_PAYLOAD:
            if (c == '*') {
                this->payload_[this->payload_len_] = '\0';
                state_ = ParseState::READ_CHECKSUM_1;
            } else {
                if (this->payload_len_ < SerialComm::RX_BUF_SIZE - 1) {
                    this->payload_[this->payload_len_++] = c;
                    calc_checksum_ ^= (uint8_t)c;
                } else {
                    // overflow → reset parser
                    state_ = ParseState::WAIT_START;
                }
            }
            break;

        case ParseState::READ_CHECKSUM_1:
            recv_checksum_ = hexToByte(c) << 4;
            state_ = ParseState::READ_CHECKSUM_2;
            break;

        case ParseState::READ_CHECKSUM_2:
            recv_checksum_ |= hexToByte(c);

            // Expect newline next but validate immediately
            if (recv_checksum_ == calc_checksum_) {
                parsePayload(this->payload_);
            }

            state_ = ParseState::WAIT_START;
            break;
    }
}

void SerialComm::parsePayload(const char* payload) {
    // Expected format: CMD,v,w
    if (strncmp(payload, "CMD,", 4) == 0) {
        float v, w;
        if (sscanf(payload + 4, "%f,%f", &v, &w) == 2) {
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
    last_cmd_.valid = false;
    return last_cmd_;
}

void SerialComm::sendOdom(float x, float y, float yaw,
                          float v, float w) {
    char payload[96];
    snprintf(payload, sizeof(payload),
             "ODOM,%.3f,%.3f,%.3f,%.3f,%.3f",
             x, y, yaw, v, w);

    uint8_t cs = 0;
    for (size_t i = 0; payload[i] != '\0'; ++i)
        cs ^= (uint8_t)payload[i];

    Serial.print('$');
    Serial.print(payload);
    Serial.print('*');

    if (cs < 16) Serial.print('0');
    Serial.print(cs, HEX);

    Serial.print('\n');
}