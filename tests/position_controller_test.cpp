#include <cassert>
#include <cmath>
#include "posistion_controller.hpp"

int main(int argc, char** argv)
{
    assert(argc == 2);
    PositionController controller(argv[1]);
    controller.loadParam();
    // A deterministic single-head export checks actual observation/history inputs.
    controller.policy = torch::jit::script::Module("PositionTestPolicy");
    controller.policy.define(R"JIT(
        def forward(self, obs: Tensor, history: Tensor) -> Tensor:
            assert obs.size(1) == 45
            assert history.size(1) == 900
            assert torch.equal(obs, history[:, -45:])
            return obs[:, 33:45] + 1.0
    )JIT");
    BasicRobotInterface robot;
    robot.gyro.fill(0.0f);
    robot.jpos = controller.stand_pos;
    robot.jvel.fill(0.0f);
    Gamepad gamepad;
    gamepad.ly = -0.4f;
    controller.GetInput(robot, gamepad);
    controller.DummyCalculate();
    controller.Calculate();
    assert(controller.single_step_obs[0] == 0.4f);
    assert(controller.single_step_obs[5] == -1.0f);
    for (size_t i = 0; i < 12; ++i)
    {
        assert(controller.pos_actions[i] == 1.0f);
        assert(std::abs(controller.jpos_des[i] - controller.stand_pos[i] - 0.25f) < 1e-6f);
        assert(controller.jtau_des[i] == 0.0f);
        assert(controller.jvel_des[i] == 0.0f);
    }
    controller.Calculate();
    assert(controller.single_step_obs[33] == 1.0f);
    assert(controller.pos_actions[0] == 2.0f);
    controller.reset(robot, gamepad);
    for (const auto& frame : controller.history_obs)
        for (float value : frame) assert(value == 0.0f);
    for (float value : controller.pos_actions) assert(value == 0.0f);

    controller.policy = torch::jit::script::Module("ClippedPositionPolicy");
    controller.policy.define(R"JIT(
        def forward(self, obs: Tensor, history: Tensor) -> Tensor:
            return torch.ones((1, 12)) * 1000.0
    )JIT");
    controller.Calculate();
    for (size_t i = 0; i < 12; ++i)
    {
        assert(controller.pos_actions[i] == 50.0f);
        assert(controller.jpos_des[i] == controller.jpos_lim_max[i]);
        assert(controller.jtau_des[i] == 0.0f);
    }
    controller.policy = torch::jit::script::Module("WrongHeadPolicy");
    controller.policy.define(R"JIT(
        def forward(self, obs: Tensor, history: Tensor) -> Tensor:
            return torch.zeros((1, 24))
    )JIT");
    bool rejected = false;
    try { controller.Calculate(); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
    std::cout << "Position controller checks passed" << std::endl;
}
