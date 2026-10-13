#pragma once
#include <cmath>
#include <algorithm>

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

inline bool bad_orientation(ManagerBasedRLEnv* env, float limit_angle = 1.0)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::fabs(std::acos(std::clamp(-data[2], -1.0f, 1.0f))) > limit_angle;
}

} 
} 