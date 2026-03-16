#ifndef __MUJOCO_SIMULATION_BRIDGE_H__
#define __MUJOCO_SIMULATION_BRIDGE_H__

#include <stdbool.h>
#include <unistd.h>
#include <Configuration.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include "stdio.h"
#include "stdlib.h"
#include <string>
#include <chrono>
#include <map>
#include <thread>
#include <memory>
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <Eigen/Dense>

#include "array_safety_daros.h"
#include "Utils/VisualGeom.h"

namespace mju = ::mujoco::sample_util;
using namespace std;

enum SimMode
{
  PLAY = 0,
  RECORD = 1
};

// Command for recording
// ffmpeg -f rawvideo -pixel_format rgb24 -video_size 1200x900 -framerate 60 -i rgb.out -vf "vflip,format=yuv420p" video.mp4
// ffmpeg -f rawvideo -pixel_format rgb24 -video_size 2560x1440 -framerate 60 -i rgb.out -vf "vflip,format=yuv420p" video.mp4

class MujocoSimulationBridge
{
public:
  MujocoSimulationBridge(const std::string &setup_file);
  virtual ~MujocoSimulationBridge();

  void init_mj_viz();
  void run();
  void handleMouseMovement(GLFWwindow *w, double xpos, double ypos, mjModel *m, mjvScene &s, mjvCamera &c);
  void handleScroll(GLFWwindow *w, double xoffset, double yoffset, mjModel *m, mjvScene &s, mjvCamera &c);

  double _sim_duration;


protected:
  virtual void _onestep_forward()
  {
    if (_mjData->time >= _ctrl_time)
    {
      _ctrl_time += _getCtrlDt();
      _UpdateSystemObserver(); // copy to state_save

      _runCtrl();

      _UpdateControlCommand();
      // std::cout << "stepping ctrl" << std::endl;
    }
    // _ApplyExternalForces();
    // std::cout << "stepping simulation" << std::endl;
    mj_step(_mjModel, _mjData);
  }
  void _ReadConfig(const std::string &file_name);
  mjModel *_mjModel = nullptr;
  mjData *_mjData = nullptr;

  char error[1000];

  mjvCamera mj_cam;
  mjvOption viz_opt;
  mjvScene mj_scene;
  mjrContext mj_context;
  GLFWwindow *mj_window;

  // Update Sensor Data
  virtual void _UpdateSystemObserver() = 0;

  // Write Control Command to Sim
  virtual void _UpdateControlCommand() = 0;

  // Update System's Visualization Data
  virtual void _RenderCameraData() {}
  virtual void _ApplyExternalForces() {}

  virtual float _getCtrlDt() = 0;
  virtual void _runCtrl() = 0;

  virtual void _LeftKeyAction(){}
  virtual void _RightKeyAction(){}
  virtual void _UpKeyAction(){}
  virtual void _DownKeyAction(){}
  virtual void _A_KeyAction(){}
  virtual void _D_KeyAction(){}
  virtual void _W_KeyAction(){}
  virtual void _S_KeyAction(){}
  virtual void _Comma_KeyAction(){}
  virtual void _Slash_KeyAction(){}

  virtual void _Y_KeyAction(){}
  virtual void _U_KeyAction(){}
  virtual void _K_KeyAction(){}

  virtual void _Z_KeyAction(){}
  virtual void _X_KeyAction(){}

  // Used for user-input styel control (replicating simialr behavior to the Unity simulation)
  virtual void _LeftKeyRelease(){}
  virtual void _RightKeyRelease(){}
  virtual void _UpKeyRelease(){}
  virtual void _DownKeyRelease(){}
  virtual void _Comma_KeyRelease(){}
  virtual void _Slash_KeyRelease(){}

  double _ctrl_time = 0.0;

private:
  SimMode _sim_mode;
  std::string _mj_xml_file;
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  bool _b_play = true;
  double lastx = 0;
  double lasty = 0;

  static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);
  static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
};

#endif
