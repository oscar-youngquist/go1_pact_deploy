#pragma once

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/joystick.h"
#include "joystick/joystick.h"

class XBoxJoystick
{
public:
    XBoxJoystick(std::string device, int bits = 15)
    {
        max_value_ = 1 << (bits - 1);

        if (device.rfind("udp://", 0) == 0) {
            const std::string port_text = device.substr(std::string("udp://").size());
            try {
                udp_port_ = std::stoi(port_text);
            } catch (const std::exception &) {
                std::cout << "Warning: Invalid UDP joystick device '" << device
                          << "'. Running with neutral joystick input." << std::endl;
                return;
            }
            openUdpSocket();
            return;
        }

        js_ = std::make_unique<Joystick>(device);
        if (!js_->isFound()) {
            std::cout << "Warning: Joystick open failed. Running simulation with neutral joystick input." << std::endl;
            return;
        }

        found_ = true;
    }

    ~XBoxJoystick()
    {
        if (udp_fd_ >= 0) {
            close(udp_fd_);
        }
    }

    void update(xRockerBtnDataStruct &key_data)
    {
        if (udp_enabled_) {
            updateUdp(key_data);
            return;
        }

        if (!found_) {
            key_data = {};
            return;
        }

        js_->getState();

        key_data.btn.components.A      = js_->button_[0];
        key_data.btn.components.B      = js_->button_[1];
        key_data.btn.components.X      = js_->button_[2];
        key_data.btn.components.Y      = js_->button_[3];
        key_data.btn.components.L1     = js_->button_[4];
        key_data.btn.components.R1     = js_->button_[5];
        key_data.btn.components.select = js_->button_[6];
        key_data.btn.components.start  = js_->button_[7];

        key_data.lx = float(js_->axis_[0]) / max_value_;
        key_data.ly = float(js_->axis_[1]) / max_value_;
        key_data.rx = float(js_->axis_[3]) / max_value_;
        key_data.ry = float(js_->axis_[4]) / max_value_;

        key_data.btn.components.L2 = js_->axis_[2] > 0;
        key_data.btn.components.R2 = js_->axis_[5] > 0;
        key_data.btn.components.up    = js_->axis_[7] < 0;
        key_data.btn.components.down  = js_->axis_[7] > 0;
        key_data.btn.components.left  = js_->axis_[6] < 0;
        key_data.btn.components.right = js_->axis_[6] > 0;
    }

private:
    void openUdpSocket()
    {
        udp_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd_ < 0) {
            std::cout << "Warning: UDP joystick socket failed: " << std::strerror(errno)
                      << ". Running with neutral joystick input." << std::endl;
            return;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(udp_port_));

        if (bind(udp_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            std::cout << "Warning: UDP joystick bind failed on port " << udp_port_ << ": "
                      << std::strerror(errno) << ". Running with neutral joystick input." << std::endl;
            close(udp_fd_);
            udp_fd_ = -1;
            return;
        }

        const int flags = fcntl(udp_fd_, F_GETFL, 0);
        fcntl(udp_fd_, F_SETFL, flags | O_NONBLOCK);
        udp_enabled_ = true;
        found_ = true;
        std::cout << "Using UDP joystick input on port " << udp_port_ << std::endl;
    }

    void updateUdp(xRockerBtnDataStruct &key_data)
    {
        char buffer[512];
        while (true) {
            const ssize_t bytes = recv(udp_fd_, buffer, sizeof(buffer) - 1, 0);
            if (bytes < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cout << "Warning: UDP joystick receive failed: " << std::strerror(errno) << std::endl;
                }
                break;
            }
            buffer[bytes] = '\0';
            parseUdpPacket(buffer);
        }

        key_data = udp_key_data_;
    }

    void parseUdpPacket(const std::string &packet)
    {
        std::istringstream stream(packet);
        std::string magic;
        stream >> magic;
        if (magic != "GPAD1") {
            return;
        }

        float lx = 0.0f;
        float ly = 0.0f;
        float rx = 0.0f;
        float ry = 0.0f;
        int A = 0;
        int B = 0;
        int X = 0;
        int Y = 0;
        int L1 = 0;
        int R1 = 0;
        int select = 0;
        int start = 0;
        int L2 = 0;
        int R2 = 0;
        int up = 0;
        int down = 0;
        int left = 0;
        int right = 0;

        if (!(stream >> lx >> ly >> rx >> ry
              >> A >> B >> X >> Y >> L1 >> R1 >> select >> start
              >> L2 >> R2 >> up >> down >> left >> right)) {
            return;
        }

        udp_key_data_ = {};
        udp_key_data_.lx = std::clamp(lx, -1.0f, 1.0f);
        udp_key_data_.ly = std::clamp(ly, -1.0f, 1.0f);
        udp_key_data_.rx = std::clamp(rx, -1.0f, 1.0f);
        udp_key_data_.ry = std::clamp(ry, -1.0f, 1.0f);
        udp_key_data_.btn.components.A = A != 0;
        udp_key_data_.btn.components.B = B != 0;
        udp_key_data_.btn.components.X = X != 0;
        udp_key_data_.btn.components.Y = Y != 0;
        udp_key_data_.btn.components.L1 = L1 != 0;
        udp_key_data_.btn.components.R1 = R1 != 0;
        udp_key_data_.btn.components.select = select != 0;
        udp_key_data_.btn.components.start = start != 0;
        udp_key_data_.btn.components.L2 = L2 != 0;
        udp_key_data_.btn.components.R2 = R2 != 0;
        udp_key_data_.btn.components.up = up != 0;
        udp_key_data_.btn.components.down = down != 0;
        udp_key_data_.btn.components.left = left != 0;
        udp_key_data_.btn.components.right = right != 0;
    }

    std::unique_ptr<Joystick> js_;
    int max_value_ = 1;
    bool found_ = false;

    int udp_fd_ = -1;
    int udp_port_ = 0;
    bool udp_enabled_ = false;
    xRockerBtnDataStruct udp_key_data_ = {};
};