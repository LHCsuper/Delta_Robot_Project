// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <vector>
#include"inverse_kin.h"


// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context

// PID 控制相关（笛卡尔空间 3 维：x,y,z）
int ee_body_id = -1;          // 末端刚体 ID（movingplatform_body）
mjtNum Kp_cart[3] = {200.0, 200.0, 300.0};
mjtNum Kd_cart[3] = {40.0,  40.0,  60.0};
mjtNum Ki_cart[3] = {0.0,   0.0,   0.0};   // 初期可以先设为 0，避免积分饱和
mjtNum e_int_cart[3] = {0.0, 0.0, 0.0};    // 误差积分




// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// main function
int main(int argc, const char **argv) {
  
  //inverse kinematics test
  inverse_kin ik;
  double position_data[3] = {0, 0, -380};
  double fi_array[3];;
  auto ret = ik.calculations(position_data, fi_array);
  for(int i=0;i<3;i++){
    std::cout << "joint["<<i<<"]:"<<fi_array[i]<<std::endl;
  }
  std::cout << "Inverse Kinematics Test:" <<ret<< std::endl;



  char error[1000] = "Could not load binary model";
  m = mj_loadXML("/home/robot/mujoco_WS/Model/delta_robot_V2.xml", 0, error, 1000);
  // make data
  d = mj_makeData(m);

   // 获取末端刚体 ID（作为控制目标）
  ee_body_id = mj_name2id(m, mjOBJ_BODY, "movingplatform_body");
  if (ee_body_id < 0) {
    mju_error("could not find body 'movingplatform_body' in model");
  }

  // 末端期望轨迹端点：A 和 B（世界坐标系下）
  mjtNum A[3] = { 0.5,  0.0, 1.05 };
  mjtNum B[3] = {-0.5,  0.0, 1.05 };


  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  float cnt = 0;
  auto step_start = std::chrono::high_resolution_clock::now();
  //仿真主体
  while (!glfwWindowShouldClose(window)) {

double t = d->time;          // MuJoCo 内部当前仿真时间
    double Thalf = 2.0;          // 半周期 2s：A->B 用 2s，B->A 用 2s，总周期 4s

    double s = fmod(t, 2.0 * Thalf);   // [0, 4) 区间
    double alpha;                      // 插值因子 ∈ [0,1]
    double dir;                        // 运动方向（1: A->B, -1: B->A）

    if (s < Thalf) {
      // A -> B
      alpha = s / Thalf;
      dir = 1.0;
    } else {
      // B -> A
      alpha = (2.0 * Thalf - s) / Thalf;
      dir = -1.0;
    }

    mjtNum x_des[3];    // 期望位置
    mjtNum xd_des[3];   // 期望速度（常值，沿直线）
    for (int i = 0; i < 3; ++i) {
      x_des[i]  = A[i] + alpha * (B[i] - A[i]);
      xd_des[i] = dir * (B[i] - A[i]) / Thalf;
    }

    //------------------------------------------------------------------
    // 二、读取当前末端实际位置、速度
    //------------------------------------------------------------------
    // 末端刚体的世界坐标位置 d->xpos 是按 body 顺序存储的，每个 body 3 个元素
    mjtNum x[3];
    mjtNum* xpos = d->xpos + 3 * ee_body_id;
    x[0] = xpos[0];
    x[1] = xpos[1];
    x[2] = xpos[2];

    // 末端线速度：使用官方函数 mj_objectVelocity 计算线速度
    mjtNum xd[3];
    mj_objectVelocity(m, d, mjOBJ_BODY, ee_body_id, xd, NULL);
    // 若版本不支持 mj_objectVelocity，可从 d->cvel 中取线速度，这里不展开
    //------------------------------------------------------------------
    // 三、笛卡尔空间 PID（对位置做 PID）
    //------------------------------------------------------------------
    mjtNum e[3];      // 位置误差
    mjtNum edot[3];   // 速度误差
    for (int i = 0; i < 3; ++i) {
      e[i]    = x_des[i]  - x[i];
      edot[i] = xd_des[i] - xd[i];
    }

    // 误差积分（简单矩形积分）
    double dt = m->opt.timestep;
    for (int i = 0; i < 3; ++i) {
      e_int_cart[i] += e[i] * dt;
    }

    // 得到笛卡尔空间等效“力”F（这里理解为末端需要的笛卡尔力指令）
    mjtNum F[3];
    for (int i = 0; i < 3; ++i) {
      F[i] = Kp_cart[i] * e[i]
           + Kd_cart[i] * edot[i]
           + Ki_cart[i] * e_int_cart[i];
    }

    //------------------------------------------------------------------
    // 四、雅可比：笛卡尔力 -> 关节广义力（tau）
    //------------------------------------------------------------------
    int nv = m->nv;
    std::vector<mjtNum> Jp(3 * nv);   // 末端质心位置 Jacobian（线速度部分）
    std::vector<mjtNum> tau(nv, 0.0);

    // 计算 movingplatform_body 的质心位置雅可比（只要线性部分）
    mj_jacBodyCom(m, d, Jp.data(), nullptr, ee_body_id);
    // Jp: 3 x nv，行主序：Jp[row*nv + col]

    // tau = J^T * F
    for (int j = 0; j < nv; ++j) {
      for (int i = 0; i < 3; ++i) {
        tau[j] += Jp[i * nv + j] * F[i];
      }
    }

    //------------------------------------------------------------------
    // 五、将广义力 tau 映射到 actuator ctrl（这里是 3 个 motor -> J1/J2/J3）
    //------------------------------------------------------------------
    // motor 在 XML 中定义为：
    // <motor name="motor1" joint="J1" .../>
    // <motor name="motor2" joint="J2" .../>
    // <motor name="motor3" joint="J3" .../>
    //
    // MuJoCo 中 actuator_trnid 存的是传动对象 ID（这里是 joint ID），
    // jnt_dofadr 将 joint ID 映射到对应的 DOF 索引。

    for (int u = 0; u < m->nu; ++u) {
      int jnt_id = m->actuator_trnid[2 * u];   // 对应的关节 ID
      int dof_adr = m->jnt_dofadr[jnt_id];     // 该关节第一个自由度在 tau 中的索引
      // 假设每个 J1/J2/J3 只有 1 个自由度
      d->ctrl[u] = tau[dof_adr];
    }

    //------------------------------------------------------------------
    // 六、推进仿真一步
    //------------------------------------------------------------------
     mj_step(m, d);

//------------------------------------------------------------------
// 七、调试输出（每隔 N 步打印一次）
//------------------------------------------------------------------
static int print_counter = 0;
int PRINT_INTERVAL = 100;   // 每 100 次仿真循环打印一次

if (print_counter % PRINT_INTERVAL == 0) {

    std::cout << "time = " << d->time << std::endl;

    std::cout << "Desired pos = ["
              << x_des[0] << ", "
              << x_des[1] << ", "
              << x_des[2] << "]" << std::endl;

    std::cout << "Actual pos  = ["
              << x[0] << ", "
              << x[1] << ", "
              << x[2] << "]" << std::endl;

    std::cout << "Error       = ["
              << e[0] << ", "
              << e[1] << ", "
              << e[2] << "]" << std::endl;

    std::cout << "----------------------------------------" << std::endl;
}
print_counter++;














    //同步时间
    auto current_time = std::chrono::high_resolution_clock::now();
    double elapsed_sec =
        std::chrono::duration<double>(current_time - step_start).count();
    double time_until_next_step = m->opt.timestep*5 - elapsed_sec;
    if (time_until_next_step > 0.0) {
      auto sleep_duration = std::chrono::duration<double>(time_until_next_step);
      std::this_thread::sleep_for(sleep_duration);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
