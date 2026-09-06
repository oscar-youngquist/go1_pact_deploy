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

#include "HardwareBridge.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
  std::cout << "MAde it past the start of main!" << std::endl;
  
  std::string param_folder = "../params";
  fs::path param = fs::current_path() / param_folder;
  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help"))
  {
    std::cout << "Usage: pact_deploy [controller-config.yaml]" << std::endl;
    return argc > 2 ? 1 : 0;
  }
  const std::string config_file_path = argc == 2
      ? fs::absolute(argv[1]).string()
      : (param / "pact_config.yaml").string();

  HardwareBridge* hardware_bridge = new HardwareBridge(config_file_path);

  hardware_bridge->run();
  
  return 0;
}
