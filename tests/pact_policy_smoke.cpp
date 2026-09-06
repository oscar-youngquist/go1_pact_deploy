#include <cassert>
#include <cmath>
#include "pact_controller.hpp"

int main(int argc, char** argv)
{
    assert(argc == 2);
    PACTController controller(argv[1]);
    controller.loadParam();
    controller.loadPolicy();
    BasicRobotInterface robot;
    robot.jpos = controller.stand_pos;
    Gamepad gamepad;
    gamepad.ly = -0.4f;
    gamepad.lx = 0.2f;
    gamepad.rx = -0.1f;
    controller.reset(robot, gamepad);
    for (int step = 0; step < 25; ++step)
    {
        controller.GetInput(robot, gamepad);
        const auto command = controller.GetLog();
        assert(command[0] == 0.4f);
        assert(command[1] == -0.2f);
        assert(command[2] == 0.1f);
        controller.Calculate();
        for (size_t i = 0; i < 12; ++i)
        {
            assert(std::isfinite(controller.jpos_des[i]));
            assert(std::isfinite(controller.jtau_des[i]));
            assert(controller.jpos_des[i] >= controller.jpos_lim_min[i]);
            assert(controller.jpos_des[i] <= controller.jpos_lim_max[i]);
            assert(std::abs(controller.jtau_des[i]) <= controller.jtau_lim[i]);
        }
    }
    std::cout << "PACT inference and simulation command checks passed" << std::endl;
}
