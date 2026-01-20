# 任务使用指南 - dogV2.2.4机器人

## 📋 新的任务名称

已为dogV2.2.4机器人创建了新的任务注册，**--task参数已更新**！

### Teacher任务（训练/评估/演示）

| 任务类型 | 任务名称 | 说明 |
|---------|---------|------|
| 训练 | `Isaac-Extreme-Parkour-Teacher-DogV2-v0` | 用于训练Teacher策略 |
| 评估 | `Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0` | 用于评估Teacher策略 |
| 演示 | `Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0` | 用于演示/播放Teacher策略 |

### Student任务（训练/评估/演示）

| 任务类型 | 任务名称 | 说明 |
|---------|---------|------|
| 训练 | `Isaac-Extreme-Parkour-Student-DogV2-v0` | 用于训练Student策略 |
| 评估 | `Isaac-Extreme-Parkour-Student-DogV2-Eval-v0` | 用于评估Student策略 |
| 演示 | `Isaac-Extreme-Parkour-Student-DogV2-Play-v0` | 用于演示/播放Student策略 |

## 🚀 使用方法

### 训练Teacher策略

```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --seed 1 \
    --headless
```

### 训练Student策略

```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-v0 \
    --seed 1 \
    --headless
```

### 评估Teacher策略

```bash
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0
```

### 评估Student策略

```bash
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Eval-v0
```

### 演示/播放策略

```bash
# Teacher演示
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0 \
    --num_envs 16

# Student演示
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Play-v0 \
    --num_envs 16
```

### 交互式演示

```bash
# Teacher交互式演示
python scripts/rsl_rl/demo.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0

# Student交互式演示
python scripts/rsl_rl/demo.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Play-v0
```

## 📊 对比：原始任务 vs 新任务

### 原始任务（Unitree Go2）
- `Isaac-Extreme-Parkour-Teacher-Unitree-Go2-v0`
- `Isaac-Extreme-Parkour-Teacher-Unitree-Go2-Play-v0`
- `Isaac-Extreme-Parkour-Teacher-Unitree-Go2-Eval-v0`
- `Isaac-Extreme-Parkour-Student-Unitree-Go2-v0`
- `Isaac-Extreme-Parkour-Student-Unitree-Go2-Play-v0`
- `Isaac-Extreme-Parkour-Student-Unitree-Go2-Eval-v0`

### 新任务（dogV2.2.4）
- `Isaac-Extreme-Parkour-Teacher-DogV2-v0` ✅
- `Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0` ✅
- `Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0` ✅
- `Isaac-Extreme-Parkour-Student-DogV2-v0` ✅
- `Isaac-Extreme-Parkour-Student-DogV2-Play-v0` ✅
- `Isaac-Extreme-Parkour-Student-DogV2-Eval-v0` ✅

## 🔍 查看所有可用任务

运行以下命令查看所有注册的任务：

```bash
python list_envs.py
```

您应该能看到所有dogV2相关的任务。

## ⚙️ 配置文件位置

所有dogV2的配置文件位于：
- `parkour_tasks/parkour_tasks/extreme_parkour_task/config/dogv2/`

包含：
- `parkour_mdp_cfg_custom.py` - MDP配置
- `parkour_teacher_cfg_custom.py` - Teacher环境配置
- `parkour_student_cfg_custom.py` - Student环境配置
- `agents/rsl_teacher_ppo_cfg_custom.py` - Teacher PPO配置
- `agents/rsl_student_ppo_cfg_custom.py` - Student PPO配置
- `__init__.py` - 任务注册

## ✅ 完成状态

- ✅ 创建了所有自定义配置文件
- ✅ 注册了新的任务名称
- ✅ 创建了agents配置
- ✅ **--task参数已更新，可以使用新的任务名称**

## 🎯 下一步

1. **验证USD文件路径**：
   - 在Isaac Sim中打开USD文件
   - 确认base link的实际prim路径
   - 更新 `default_cfg_custom.py` 中的 `BASE_LINK_NAME`

2. **运行测试**：
   ```bash
   python parkour_test/test_terrain_generator_custom.py
   ```

3. **开始训练**：
   ```bash
   python scripts/rsl_rl/train.py --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 --seed 1
   ```
