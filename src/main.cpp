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
  std::string config_file_path = param / "pact_config.yaml";

  HardwareBridge* hardware_bridge = new HardwareBridge(config_file_path);

  hardware_bridge->run();
  
  return 0;
}