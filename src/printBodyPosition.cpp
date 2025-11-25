#include <mujoco/mujoco.h>
#include <iostream>

void printBodyPosition(const mjModel* m, const mjData* d, const char* body_name)
{
    // 1. 获取 body ID
    int id = mj_name2id(m, mjOBJ_BODY, body_name);
    if (id < 0) {
        std::cout << "Body " << body_name << " not found.\n";
        return;
    }

    // 2. 获取世界坐标下的位置（xpos）
    const double* p = d->xpos[id];    // 3 个 double，世界坐标 xyz

    std::cout << "Body [" << body_name << "] world position: "
              << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}