#include <mujoco/mujoco.h>
#include <iostream>
#include "inverse_kin.h"

void inverse_kin::printBodyPosition(const mjModel* m, const mjData* d, const char* body_name)
{
    // 1. 查 body id
    int id = mj_name2id(m, mjOBJ_BODY, body_name);
    if (id < 0) {
        std::cout << "Body " << body_name << " not found.\n";
        return;
    }

    // 2. 正确的索引方式（你的 MuJoCo 版本）
    const double* p = d->xpos + 3 * id;

    std::cout << "Body [" << body_name << "] position = "
              << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}
