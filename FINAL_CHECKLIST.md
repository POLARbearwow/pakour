# 最终检查清单 - dogV2.2.4机器人替换

## ✅ 已完成的工作

### 1. 核心配置文件 ✅
- [x] `parkour_tasks/parkour_tasks/custom_robot_cfg.py` - 自定义机器人配置
- [x] `parkour_tasks/parkour_tasks/default_cfg_custom.py` - 自定义默认场景配置

### 2. MDP配置文件 ✅
- [x] `dogv2/parkour_mdp_cfg_custom.py` - 更新了关节和身体名称匹配
- [x] `dogv2/parkour_teacher_cfg_custom.py` - Teacher环境配置
- [x] `dogv2/parkour_student_cfg_custom.py` - Student环境配置

### 3. Agents配置 ✅
- [x] `dogv2/agents/rsl_teacher_ppo_cfg_custom.py` - Teacher PPO配置
- [x] `dogv2/agents/rsl_student_ppo_cfg_custom.py` - Student PPO配置
- [x] `dogv2/agents/skrl_parkour_ppo_cfg.yaml` - SKRL配置（复制自go2）

### 4. 任务注册 ✅
- [x] `dogv2/__init__.py` - 注册了6个新任务

### 5. 测试文件 ✅
- [x] `parkour_test/test_camera_custom.py` - 自定义相机测试
- [x] `parkour_test/test_terrain_generator_custom.py` - 自定义地形测试

## 🎯 新的任务名称（--task参数）

### Teacher任务
- `Isaac-Extreme-Parkour-Teacher-DogV2-v0` - 训练
- `Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0` - 演示
- `Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0` - 评估

### Student任务
- `Isaac-Extreme-Parkour-Student-DogV2-v0` - 训练
- `Isaac-Extreme-Parkour-Student-DogV2-Play-v0` - 演示
- `Isaac-Extreme-Parkour-Student-DogV2-Eval-v0` - 评估

## ⚠️ 需要验证和调整的事项

### 1. USD文件中的prim路径 ⚠️ **重要**
**必须检查**：在Isaac Sim中打开USD文件，确认base link的实际prim路径

**当前配置使用**：`DOGV2_2_4_SLDASM_base_link`（来自URDF）

**如果不同，需要修改**：
- `parkour_tasks/parkour_tasks/default_cfg_custom.py` 中的 `BASE_LINK_NAME` 变量

**常见情况**：
- 如果USD中使用 `base`，改为：`BASE_LINK_NAME = "base"`
- 如果USD中使用 `base_link`，改为：`BASE_LINK_NAME = "base_link"`

### 2. 关节和身体名称验证 ⚠️
运行测试后，打印并验证：
```python
asset = env.scene["robot"]
print("Joint names:", asset.joint_names)
print("Body names:", asset.body_names)
```

**期望的关节名称**（12个）：
- `LF_HipA_joint`, `LF_HipF_joint`, `LF_Knee_joint`
- `LR_HipA_joint`, `LR_HipF_joint`, `LR_Knee_joint`
- `RF_HipA_joint`, `RF_HipF_joint`, `RF_Knee_joint`
- `RR_HipA_joint`, `RR_HipF_joint`, `RR_Knee_joint`

**期望的身体名称**（包含base和4个脚）：
- Base: 包含 `base` 的名称
- Feet: 包含 `LF_Foot`, `RF_Foot`, `LR_Foot`, `RR_Foot` 的名称

### 3. 关节顺序验证 ⚠️
确保关节顺序与URDF一致：
1. LF_HipA, LF_HipF, LF_Knee
2. LR_HipA, LR_HipF, LR_Knee
3. RF_HipA, RF_HipF, RF_Knee
4. RR_HipA, RR_HipF, RR_Knee

## 🧪 测试步骤

### 步骤1: 验证任务注册
```bash
python list_envs.py | grep DogV2
```
应该看到6个dogV2任务。

### 步骤2: 测试场景加载
```bash
python parkour_test/test_terrain_generator_custom.py
```

### 步骤3: 测试相机
```bash
python parkour_test/test_camera_custom.py
```

### 步骤4: 测试训练（小规模）
```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --seed 1 \
    --num_envs 4 \
    --headless
```

## 📝 如果遇到问题

### 问题1: 找不到base link
**症状**：相机或传感器无法找到base link
**解决**：
1. 在Isaac Sim中检查USD文件的prim路径
2. 更新 `default_cfg_custom.py` 中的 `BASE_LINK_NAME`

### 问题2: 关节名称不匹配
**症状**：执行器无法找到关节
**解决**：
1. 打印实际关节名称
2. 检查USD文件中的关节名称
3. 调整 `custom_robot_cfg.py` 中的 `joint_names_expr`

### 问题3: 身体名称不匹配
**症状**：奖励或观察无法找到身体
**解决**：
1. 打印实际身体名称
2. 调整 `parkour_mdp_cfg_custom.py` 中的 `body_names` 匹配规则

### 问题4: 任务未注册
**症状**：`--task` 参数找不到任务
**解决**：
1. 检查 `dogv2/__init__.py` 是否正确
2. 确保包已正确安装：`pip install -e .`
3. 运行 `python list_envs.py` 查看所有任务

## 📚 相关文档

- `CUSTOM_CONFIG_README.md` - 自定义配置文件说明
- `TASK_USAGE_GUIDE.md` - 任务使用指南
- `PRIM_PATH_EXPLANATION.md` - Prim路径说明
- `ROBOT_REPLACEMENT_GUIDE.md` - 详细替换指南

## ✨ 总结

所有配置文件已创建完成，**--task参数已更新**！

您现在可以使用新的任务名称来训练和运行dogV2机器人了。

**最重要的下一步**：
1. ✅ 验证USD文件中的prim路径
2. ✅ 运行测试脚本验证配置
3. ✅ 开始训练！
