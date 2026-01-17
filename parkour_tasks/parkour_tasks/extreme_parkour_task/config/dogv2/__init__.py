# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configurations for dogV2.2.4 robot parkour environments.
原始配置在 ../go2/ 目录下，保留作为参考
"""

import gymnasium as gym

from . import agents

##
# Register Gym environments for dogV2.2.4 robot
##

# Teacher任务
gym.register(
    id="Isaac-Extreme-Parkour-Teacher-DogV2-v0",
    entry_point="parkour_isaaclab.envs:ParkourManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.parkour_teacher_cfg_custom:DogV2TeacherParkourEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_teacher_ppo_cfg_custom:DogV2ParkourTeacherPPORunnerCfg",
        "skrl_cfg_entry_point": "parkour_tasks.extreme_parkour_task.config.go2.agents:skrl_parkour_ppo_cfg.yaml",  # 使用go2的yaml配置
    },
)

gym.register(
    id="Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0",
    entry_point="parkour_isaaclab.envs:ParkourManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.parkour_teacher_cfg_custom:DogV2TeacherParkourEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_teacher_ppo_cfg_custom:DogV2ParkourTeacherPPORunnerCfg",
        "skrl_cfg_entry_point": "parkour_tasks.extreme_parkour_task.config.go2.agents:skrl_parkour_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0",
    entry_point="parkour_isaaclab.envs:ParkourManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.parkour_teacher_cfg_custom:DogV2TeacherParkourEnvCfg_EVAL",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_teacher_ppo_cfg_custom:DogV2ParkourTeacherPPORunnerCfg",
        "skrl_cfg_entry_point": "parkour_tasks.extreme_parkour_task.config.go2.agents:skrl_parkour_ppo_cfg.yaml",
    },
)

# Student任务
gym.register(
    id="Isaac-Extreme-Parkour-Student-DogV2-v0",
    entry_point="parkour_isaaclab.envs:ParkourManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.parkour_student_cfg_custom:DogV2StudentParkourEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_student_ppo_cfg_custom:DogV2ParkourStudentPPORunnerCfg",
        "skrl_cfg_entry_point": "parkour_tasks.extreme_parkour_task.config.go2.agents:skrl_parkour_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Extreme-Parkour-Student-DogV2-Play-v0",
    entry_point="parkour_isaaclab.envs:ParkourManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.parkour_student_cfg_custom:DogV2StudentParkourEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_student_ppo_cfg_custom:DogV2ParkourStudentPPORunnerCfg",
        "skrl_cfg_entry_point": "parkour_tasks.extreme_parkour_task.config.go2.agents:skrl_parkour_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Extreme-Parkour-Student-DogV2-Eval-v0",
    entry_point="parkour_isaaclab.envs:ParkourManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.parkour_student_cfg_custom:DogV2StudentParkourEnvCfg_EVAL",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_student_ppo_cfg_custom:DogV2ParkourStudentPPORunnerCfg",
        "skrl_cfg_entry_point": "parkour_tasks.extreme_parkour_task.config.go2.agents:skrl_parkour_ppo_cfg.yaml",
    },
)
