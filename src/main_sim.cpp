#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <optional>
#include <chrono>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>

#include "torch/script.h"

#include "PACTMJSimulationBridge.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
  std::cout << "Made it past the start of main!" << std::endl;
  
  std::string param_folder = THIS_COM"go1_pact_deploy/params";
  fs::path param = fs::current_path() / param_folder;
  std::string ctrl_config_file_path = param / "pact_config.yaml";

  std::string sim_param_folder = THIS_COM"Configs";
  fs::path sim_param = fs::current_path() / sim_param_folder;
  const std::string sim_config_file_path = sim_param / "pact_mujoco_sim_config.yaml";

  std::cout << sim_config_file_path.c_str() << std::endl;
  std::cout << ctrl_config_file_path.c_str() << std::endl;

  PACTMJSimulationBridge* mj_simulation_bridge = new PACTMJSimulationBridge(sim_config_file_path, ctrl_config_file_path);

  mj_simulation_bridge->run();
  
  return 0;
}