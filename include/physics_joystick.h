#pragma once

#include <iostream>
#include "../3rd_parties/unitree_legged_sdk/include/unitree_legged_sdk/joystick.h"
#include "joystick/joystick.h"
#include <memory>


class XBoxJoystick
{
public:
    XBoxJoystick(std::string device, int bits = 15)
	{
		js_ = std::make_unique<Joystick>(device);
		if(!js_->isFound()) {
			std::cout << "Error: Joystick open failed." << std::endl;
			exit(1);
		}
        max_value_ = 1 << (bits - 1);
	}

    void update(xRockerBtnDataStruct &key_data)
    {
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
	std::unique_ptr<Joystick> js_;
	int max_value_;
};


// class SwitchJoystick
// {
// public:
//     SwitchJoystick(std::string device, int bits = 15)
// 	: unitree::common::UnitreeJoystick()
// 	{
// 		js_ = std::make_unique<Joystick>(device);
// 		if(!js_->isFound()) {
// 			std::cout << "Error: Joystick open failed." << std::endl;
// 			exit(1);
// 		}
//         max_value_ = 1 << (bits - 1);
// 	}

//     void update() override
//     {
//         js_->getState();
//         back(js_->button_[10]);
//         start(js_->button_[11]);
//         LB(js_->button_[6]);
//         RB(js_->button_[7]);
//         A(js_->button_[0]);
//         B(js_->button_[1]); 
//         X(js_->button_[3]);
//         Y(js_->button_[4]);
//         up(js_->axis_[7] < 0);
//         down(js_->axis_[7] > 0);
//         left(js_->axis_[6] < 0);
//         right(js_->axis_[6] > 0);
//         LT(js_->axis_[5] > 0);
//         RT(js_->axis_[4] > 0);
//         lx(double(js_->axis_[0]) / max_value_);
//         ly(-double(js_->axis_[1]) / max_value_);
//         rx(double(js_->axis_[2]) / max_value_);
//         ry(-double(js_->axis_[3]) / max_value_);
//     }
// private:
// 	std::unique_ptr<Joystick> js_;
// 	int max_value_;
// };

// // Yahboom USB wireless joystick, red light mode
// class YahboomJoystick : public unitree::common::UnitreeJoystick
// {
// public:
//     YahboomJoystick(std::string device, int bits = 15)
// 	: unitree::common::UnitreeJoystick()
// 	{
// 		js_ = std::make_unique<Joystick>(device);
// 		if(!js_->isFound()) {
// 			std::cout << "Error: Joystick open failed." << std::endl;
// 			exit(1);
// 		}
//         max_value_ = 1 << (bits - 1);
// 	}

//     void update() override
//     {
//         js_->getState();
//         back(js_->button_[10]); // select button
//         start(js_->button_[11]);
//         LB(js_->button_[6]);
//         RB(js_->button_[7]);
//         A(js_->button_[0]);
//         B(js_->button_[1]); 
//         X(js_->button_[3]);
//         Y(js_->button_[4]);
//         up(js_->axis_[7] < 0);
//         down(js_->axis_[7] > 0);
//         left(js_->axis_[6] < 0);
//         right(js_->axis_[6] > 0);
//         LT(js_->button_[8]);
//         RT(js_->button_[9]);
//         lx(double(js_->axis_[0]) / max_value_);
//         ly(-double(js_->axis_[1]) / max_value_);
//         rx(double(js_->axis_[2]) / max_value_);
//         ry(-double(js_->axis_[3]) / max_value_);
//     }
// private:
// 	std::unique_ptr<Joystick> js_;
// 	int max_value_;
// };

// // BeTop USB joystick
// class BeTopJoystick : public unitree::common::UnitreeJoystick
// {
// public:
//     BeTopJoystick(std::string device, int bits = 15)
// 	: unitree::common::UnitreeJoystick()
// 	{
// 		js_ = std::make_unique<Joystick>(device);
// 		if(!js_->isFound()) {
// 			std::cout << "Error: Joystick open failed." << std::endl;
// 			exit(1);
// 		}
//         max_value_ = 1 << (bits - 1);
// 	}

//     void update() override
//     {
//         js_->getState();
//         back(js_->button_[6]); // select button
//         start(js_->button_[7]);
//         LB(js_->button_[4]);
//         RB(js_->button_[5]);
//         A(js_->button_[0]);
//         B(js_->button_[1]); 
//         X(js_->button_[2]);
//         Y(js_->button_[3]);
//         up(js_->axis_[7] < 0);
//         down(js_->axis_[7] > 0);
//         left(js_->axis_[6] < 0);
//         right(js_->axis_[6] > 0);
//         LT(js_->axis_[2] > 0);
//         RT(js_->axis_[5] > 0);
//         lx(double(js_->axis_[0]) / max_value_);
//         ly(-double(js_->axis_[1]) / max_value_);
//         rx(double(js_->axis_[3]) / max_value_);
//         ry(-double(js_->axis_[4]) / max_value_);
//     }
// private:
// 	std::unique_ptr<Joystick> js_;
// 	int max_value_;
// };