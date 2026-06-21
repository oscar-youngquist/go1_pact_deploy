// #include <unistd.h>
// #include <iostream>
// #include <map>
// #include "joystick.h"

// #define GAMEPAD_TYPE 1 // 1: XBOX, 0: SWITCH
// #define MAX_AXES_VALUE 32768
// #define MIN_AXES_VALUE -32768
// using namespace std;

// typedef union
// {
//   struct
//   {
//     uint8_t R1 : 1;
//     uint8_t L1 : 1;
//     uint8_t start : 1;
//     uint8_t select : 1;
//     uint8_t R2 : 1;
//     uint8_t L2 : 1;
//     uint8_t F1 : 1;
//     uint8_t F2 : 1;
//     uint8_t A : 1;
//     uint8_t B : 1;
//     uint8_t X : 1;
//     uint8_t Y : 1;
//     uint8_t up : 1;
//     uint8_t right : 1;
//     uint8_t down : 1;
//     uint8_t left : 1;
//   } components;
//   uint16_t value;
// } xKeySwitchUnion;

// typedef struct {
//     uint8_t head[2];
//     xKeySwitchUnion btn;
//     float lx;
//     float rx;
//     float ry;
//     float L2;
//     float ly;

//     uint8_t idle[16];
// } xRockerBtnDataStruct;

// int main(int argc, char **argv)
// {
//   // Create an instance of Joystick
//   Joystick joystick("/dev/input/js0");

//   // Ensure that it was found and that we can use it
//   if (!joystick.isFound())
//   {
//     printf("open failed.\n");
//     exit(1);
//   }

//   xKeySwitchUnion unitree_key;
//   map<string, int> AxisId =
//       {
//           {"LX", 0}, // Left stick axis x
//           {"LY", 1}, // Left stick axis y
//           {"RX", 3}, // Right stick axis x
//           {"RY", 4}, // Right stick axis y
//           {"LT", 2}, // Left trigger
//           {"RT", 5}, // Right trigger
//           {"DX", 6}, // Directional pad x
//           {"DY", 7}, // Directional pad y
//       };

//   map<string, int> ButtonId =
//       {
//           {"X", 2},
//           {"Y", 3},
//           {"B", 1},
//           {"A", 0},
//           {"LB", 4},
//           {"RB", 5},
//           {"SELECT", 6},
//           {"START", 7},
//       };

//   while (true)
//   {

//     // Attempt to sample an event from the joystick
//     joystick.getState();

//     unitree_key.components.R1 = joystick.button_[ButtonId["RB"]];
//     unitree_key.components.L1 = joystick.button_[ButtonId["LB"]];
//     unitree_key.components.start = joystick.button_[ButtonId["START"]];
//     unitree_key.components.select = joystick.button_[ButtonId["SELECT"]];
//     unitree_key.components.R2 = (joystick.axis_[AxisId["RT"]] > 0);
//     unitree_key.components.L2 = (joystick.axis_[AxisId["LT"]] > 0);
//     unitree_key.components.F1 = 0;
//     unitree_key.components.F2 = 0;
//     unitree_key.components.A = joystick.button_[ButtonId["A"]];
//     unitree_key.components.B = joystick.button_[ButtonId["B"]];
//     unitree_key.components.X = joystick.button_[ButtonId["X"]];
//     unitree_key.components.Y = joystick.button_[ButtonId["Y"]];
//     unitree_key.components.up = (joystick.axis_[AxisId["DY"]] < 0);
//     unitree_key.components.right = (joystick.axis_[AxisId["DX"]] > 0);
//     unitree_key.components.down = (joystick.axis_[AxisId["DY"]] > 0);
//     unitree_key.components.left = (joystick.axis_[AxisId["DX"]] < 0);

//     cout << unitree_key.value << endl;

//     // Restrict rate
//     usleep(10000);
//   }
//   return 0;
// };

#include <unistd.h>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <iomanip>
#include <cstdint>
#include "joystick.h"

#define GAMEPAD_TYPE 1 // 1: XBOX, 0: SWITCH
#define MAX_AXES_VALUE 32768.0f
#define MIN_AXES_VALUE -32768.0f

using namespace std;

typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

typedef struct
{
  uint8_t head[2];
  xKeySwitchUnion btn;
  float lx;
  float rx;
  float ry;
  float L2;
  float ly;

  uint8_t idle[16];
} xRockerBtnDataStruct;

static float normalizeAxis(int raw)
{
  // Clamp just in case
  if (raw > 32767)
    raw = 32767;
  if (raw < -32768)
    raw = -32768;

  return static_cast<float>(raw) / MAX_AXES_VALUE;
}

int main(int argc, char **argv)
{
  Joystick joystick("/dev/input/js0");

  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  xKeySwitchUnion unitree_key;
  unitree_key.value = 0;

  map<string, int> AxisId =
      {
          {"LX", 0}, // Left stick axis x
          {"LY", 1}, // Left stick axis y
          {"RX", 3}, // Right stick axis x
          {"RY", 4}, // Right stick axis y
          {"LT", 2}, // Left trigger
          {"RT", 5}, // Right trigger
          {"DX", 6}, // Directional pad x
          {"DY", 7}, // Directional pad y
      };

  map<string, int> ButtonId =
      {
          {"A", 0},
          {"B", 1},
          {"X", 2},
          {"Y", 3},
          {"LB", 4},
          {"RB", 5},
          {"SELECT", 6},
          {"START", 7},
      };

  while (true)
  {
    joystick.getState();

    unitree_key.components.R1 = joystick.button_[ButtonId["RB"]];
    unitree_key.components.L1 = joystick.button_[ButtonId["LB"]];
    unitree_key.components.start = joystick.button_[ButtonId["START"]];
    unitree_key.components.select = joystick.button_[ButtonId["SELECT"]];
    unitree_key.components.R2 = (joystick.axis_[AxisId["RT"]] > 0);
    unitree_key.components.L2 = (joystick.axis_[AxisId["LT"]] > 0);
    unitree_key.components.F1 = 0;
    unitree_key.components.F2 = 0;
    unitree_key.components.A = joystick.button_[ButtonId["A"]];
    unitree_key.components.B = joystick.button_[ButtonId["B"]];
    unitree_key.components.X = joystick.button_[ButtonId["X"]];
    unitree_key.components.Y = joystick.button_[ButtonId["Y"]];
    unitree_key.components.up = (joystick.axis_[AxisId["DY"]] < 0);
    unitree_key.components.right = (joystick.axis_[AxisId["DX"]] > 0);
    unitree_key.components.down = (joystick.axis_[AxisId["DY"]] > 0);
    unitree_key.components.left = (joystick.axis_[AxisId["DX"]] < 0);

    float lx = normalizeAxis(joystick.axis_[AxisId["LX"]]);
    float ly = normalizeAxis(joystick.axis_[AxisId["LY"]]);
    float rx = normalizeAxis(joystick.axis_[AxisId["RX"]]);
    float ry = normalizeAxis(joystick.axis_[AxisId["RY"]]);

    vector<string> pressed_buttons;

    for (const auto &kv : ButtonId)
    {
      if (joystick.button_[kv.second])
      {
        pressed_buttons.push_back(kv.first);
      }
    }

    if (unitree_key.components.L2)
      pressed_buttons.push_back("LT");
    if (unitree_key.components.R2)
      pressed_buttons.push_back("RT");
    if (unitree_key.components.up)
      pressed_buttons.push_back("DPAD_UP");
    if (unitree_key.components.right)
      pressed_buttons.push_back("DPAD_RIGHT");
    if (unitree_key.components.down)
      pressed_buttons.push_back("DPAD_DOWN");
    if (unitree_key.components.left)
      pressed_buttons.push_back("DPAD_LEFT");

    cout << fixed << setprecision(3);
    cout << "Left Stick  (x, y): (" << lx << ", " << ly << ")   "
         << "Right Stick (x, y): (" << rx << ", " << ry << ")   "
         << "Pressed: ";

    if (pressed_buttons.empty())
    {
      cout << "None";
    }
    else
    {
      for (size_t i = 0; i < pressed_buttons.size(); ++i)
      {
        cout << pressed_buttons[i];
        if (i + 1 < pressed_buttons.size())
          cout << ", ";
      }
    }

    cout << endl;

    usleep(10000);
  }

  return 0;
}
