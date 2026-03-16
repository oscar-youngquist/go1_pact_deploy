#include <MujocoSimulationBridge.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include "Utils/utilities.h"
#include "VisualGeomManager.hpp"

MujocoSimulationBridge::MujocoSimulationBridge(const std::string &setup_file)
{
  _ReadConfig(setup_file);
  _mjModel = mj_loadXML(_mj_xml_file.c_str(), NULL, error, 1000);
  _mjData = mj_makeData(_mjModel);
  mj_forward(_mjModel, _mjData);
  mj_resetDataKeyframe(_mjModel, _mjData, 0);
}

MujocoSimulationBridge::~MujocoSimulationBridge()
{
  mjv_freeScene(&mj_scene);
  mjr_freeContext(&mj_context);
  mj_deleteData(_mjData);
  mj_deleteModel(_mjModel);
  glfwTerminate();
}

void MujocoSimulationBridge::init_mj_viz()
{
  if (!glfwInit())
    mju_error("Could not initialize GLFW");

  if (_sim_mode == SimMode::RECORD)
  {
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  }
  else
  {
    glfwWindowHint(GLFW_SAMPLES, 4);
  }
  mj_window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
  // mj_window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);


  if (!mj_window)
  {
    mju_error("Could not create GLFW window!");
    glfwTerminate();
    std::exit(1);
  }
  glfwMakeContextCurrent(mj_window);
  glfwSwapInterval(1);

  int xPos = 50; 
  int yPos = 50; 
  glfwSetWindowPos(mj_window, xPos, yPos);

  mjv_defaultOption(&viz_opt);

  mjv_defaultScene(&mj_scene);
  mjr_defaultContext(&mj_context);
  mjv_makeScene(_mjModel, &mj_scene, 2000);

  mjv_defaultCamera(&mj_cam);
  mj_cam.azimuth = 90;
  mj_cam.elevation = -30;
  mj_cam.distance = 5.0; // Adjust based on your scene
  mj_cam.lookat[0] = 0.0;
  mj_cam.lookat[1] = 0.0;
  mj_cam.lookat[2] = 1.0;

  mjr_makeContext(_mjModel, &mj_context, mjFONTSCALE_150);
  glfwSetWindowUserPointer(mj_window, this);
  glfwSetScrollCallback(mj_window, scrollCallback);
  glfwSetKeyCallback(mj_window, keyCallback);
}

void MujocoSimulationBridge::run()
{
  init_mj_viz();

  _mjModel->opt.gravity[0] = 0.0; // Gravity in the X-axis
  _mjModel->opt.gravity[1] = 0.0; // Gravity in the Y-axis
  _mjModel->opt.gravity[2] = -9.81;

  // set contact options
  _mjModel->opt.jacobian = mjJAC_DENSE;
  _mjModel->opt.cone = mjCONE_ELLIPTIC;

  double frametime = 0;
  int framecount = 0;
  double fps = 60;

  // // Off screen rendering
  // if (_sim_mode == SimMode::RECORD)
  // {
  //   // change mj context to be high resolution
  //   mjr_resizeOffscreen(2560, 1440, &mj_context);

  //   // set rendering to offscreen buffer
  //   mjr_setBuffer(mjFB_OFFSCREEN, &mj_context);
  //   if (mj_context.currentBuffer != mjFB_OFFSCREEN)
  //   {
  //     std::printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");
  //   }
  // }
  // get size of active renderbuffer
  mjrRect viewport = mjr_maxViewport(&mj_context);
  int W = viewport.width;
  int H = viewport.height;
  printf("Resolution: %dx%d. Please use these number when converting.....\n", mj_context.offWidth, mj_context.offHeight);

  // allocate rgb and depth buffers
  unsigned char *rgb = (unsigned char *)std::malloc(3 * W * H);
  float *depth = (float *)std::malloc(sizeof(float) * W * H);
  if (!rgb || !depth)
  {
    mju_error("Could not allocate buffers");
  }

  // create output rgb file
  std::FILE *fp = std::fopen("rgb.out", "wb");
  if (!fp)
  {
    mju_error("Could not open rgbfile for writing");
  }

  while (!glfwWindowShouldClose(mj_window) && _mjData->time < _sim_duration)
  {
    // render new frame if it is time (or first frame)
    if ((_mjData->time - frametime) > 1. / fps || frametime == 0)
    {
      if (_sim_mode == SimMode::RECORD)
      { // Record mode .....................
        // update abstract scene
        mjv_updateScene(_mjModel, _mjData, &viz_opt, NULL, &mj_cam, mjCAT_ALL, &mj_scene);
        // updateScene will clear geoms
        _RenderCameraData();

        // render scene in offscreen buffer
        mjr_render(viewport, &mj_scene, &mj_context);

        // add time stamp in upper-left corner
        char stamp[50];
        mju::sprintf_arr(stamp, "Time = %.3f", _mjData->time);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &mj_context);

        // read rgb and depth buffers
        mjr_readPixels(rgb, depth, viewport, &mj_context);

        // write rgb image to file
        std::fwrite(rgb, 3, W * H, fp);

        // print every 10 frames: '.' if ok, 'x' if OpenGL error
        if (((framecount++) % 10) == 0)
        {
          if (mjr_getError())
          {
            std::printf("x");
          }
          else
          {
            std::printf(".");
          }
        }
        frametime = _mjData->time;
      }
      else if (_sim_mode == SimMode::PLAY)
      { // Play mode ...............
        // updateScene will clear geoms
        mjv_updateScene(_mjModel, _mjData, &viz_opt, NULL, &mj_cam, mjCAT_ALL, &mj_scene);
        // updateGeoms should be called after updateScene before render
        VisualGeomManager::GetInstance()->updateGeoms(mj_scene);
        mjr_render(viewport, &mj_scene, &mj_context);
        glfwSwapBuffers(mj_window);
        glfwPollEvents();

        // Get current mouse position
        double xpos, ypos;
        glfwGetCursorPos(mj_window, &xpos, &ypos);

        // Call the function to handle rotation and panning
        handleMouseMovement(mj_window, xpos, ypos, _mjModel, mj_scene, mj_cam);
        glfwSwapBuffers(mj_window);
        _RenderCameraData();

        // add time stamp in upper-left corner
        char stamp[50];
        mju::sprintf_arr(stamp, "Time = %.3f", _mjData->time);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &mj_context);

        // // read rgb and depth buffers
        // mjr_readPixels(rgb, depth, viewport, &mj_context);

        // // write rgb image to file
        // std::fwrite(rgb, 3, W * H, fp);

        // // print every 10 frames: '.' if ok, 'x' if OpenGL error
        // if (((framecount++) % 10) == 0)
        // {
        //   if (mjr_getError())
        //   {
        //     std::printf("x");
        //   }
        //   else
        //   {
        //     std::printf(".");
        //   }
        // }

#ifdef __APPLE__
        glfwSwapBuffers(mj_window);
#endif
        frametime = _mjData->time;
      }
    } // end of if ((_mjData->time-frametime) > 1./fps || frametime==0)
    _onestep_forward();
  } // end of while

  std::fclose(fp);
  std::free(rgb);
  std::free(depth);
} // end of run

void MujocoSimulationBridge::handleMouseMovement(GLFWwindow *window, double xpos, double ypos, mjModel *m, mjvScene &scn, mjvCamera &cam)
{
  static double lastx = 0, lasty = 0;
  static bool button_left = false, button_right = false;

  // Update the state of the mouse buttons
  if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
  {
    button_left = true;
  }
  else
  {
    button_left = false;
  }

  if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
  {
    button_right = true;
  }
  else
  {
    button_right = false;
  }

  // If no buttons are pressed, do nothing
  if (!button_left && !button_right)
  {
    lastx = xpos;
    lasty = ypos;
    return;
  }
  // Compute the mouse displacement
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // Get the current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Determine action based on the mouse button pressed
  mjtMouse action;
  if (button_right)
  {
    action = mjMOUSE_MOVE_V; // Vertical pan when right button is pressed
  }
  else if (button_left)
  {
    action = mjMOUSE_ROTATE_V; // Rotate when left button is pressed
  }
  // Move the camera based on mouse input
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void MujocoSimulationBridge::handleScroll(GLFWwindow *window, double xoffset, double yoffset, mjModel *m, mjvScene &scene, mjvCamera &cam)
{
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.04 * yoffset, &scene, &cam);
}

void MujocoSimulationBridge::_ReadConfig(const std::string &file_name)
{
  ParamHandler param_handler(file_name);
  std::string mujoco_file_localpath;
  param_handler.getString("Mujoco_XML_FileName", mujoco_file_localpath);
  this->_mj_xml_file = THIS_COM + mujoco_file_localpath;

  double input;
  param_handler.getValue("sim_mode", input);
  this->_sim_mode = static_cast<SimMode>(input);
  param_handler.getValue("sim_duration", input);
  this->_sim_duration = input;
}

void MujocoSimulationBridge::scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
  MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
  instance->handleScroll(window, xoffset, yoffset, instance->_mjModel, instance->mj_scene, instance->mj_cam);
}

void MujocoSimulationBridge::keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
  {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
  if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    // switch between play and pause
    instance->_b_play = !(instance->_b_play);
  }
  if (key == GLFW_KEY_LEFT && action == GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_LeftKeyAction();
  }
  if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_RightKeyAction();
  }
  if (key == GLFW_KEY_UP && action == GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_UpKeyAction();
  }
  if (key == GLFW_KEY_DOWN && action == GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_DownKeyAction();
  }
  if (key==GLFW_KEY_A && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_A_KeyAction();
  }
  if (key==GLFW_KEY_D && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_D_KeyAction();
  }
  if (key==GLFW_KEY_W && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_W_KeyAction();
  }
  if (key==GLFW_KEY_S && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_S_KeyAction();
  }
  if (key==GLFW_KEY_COMMA && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_Comma_KeyAction();
  }
  if (key==GLFW_KEY_SLASH && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_Slash_KeyAction();
  }

  // Used to zero-out keyboard input upcon control key release
  if (key == GLFW_KEY_LEFT && action == GLFW_RELEASE)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_LeftKeyRelease();
  }
  if (key == GLFW_KEY_RIGHT && action == GLFW_RELEASE)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_RightKeyRelease();
  }
  if (key == GLFW_KEY_UP && action == GLFW_RELEASE)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_UpKeyRelease();
  }
  if (key == GLFW_KEY_DOWN && action == GLFW_RELEASE)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_DownKeyRelease();
  }
  if (key==GLFW_KEY_COMMA && action==GLFW_RELEASE)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_Comma_KeyRelease();
  }
  if (key==GLFW_KEY_SLASH && action==GLFW_RELEASE)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_Slash_KeyRelease();
  }

  if (key==GLFW_KEY_K && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_K_KeyAction();
  }
  
  if (key==GLFW_KEY_Y && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_Y_KeyAction();
  }
  if (key==GLFW_KEY_U && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_U_KeyAction();
  }
  if (key==GLFW_KEY_Z && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_Z_KeyAction();
  }
  if (key==GLFW_KEY_X && action==GLFW_PRESS)
  {
    MujocoSimulationBridge *instance = static_cast<MujocoSimulationBridge *>(glfwGetWindowUserPointer(window));
    instance->_X_KeyAction();
  }
}