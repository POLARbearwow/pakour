# DogV2 Parkour 任务使用指南

本文档说明了 DogV2 机器人在 Parkour 环境中的所有可用任务配置及其使用方法。

---

## 📋 任务概览

项目共包含 **6 个任务**，分为两大类：

- **Teacher 任务（3个）**：使用完整状态信息（height scan + 本体感知）
- **Student 任务（3个）**：使用深度相机图像 + 本体感知

每类包含 3 种配置：
- **v0**：用于训练
- **Eval-v0**：用于评估
- **Play-v0**：用于可视化展示

---

## 🎓 Teacher 任务

Teacher 策略使用高度扫描传感器和本体感知信息，适合在仿真环境中使用。

### 1. Isaac-Extreme-Parkour-Teacher-DogV2-v0

**配置类：** `DogV2TeacherParkourEnvCfg`  
**配置文件：** `parkour_tasks/parkour_tasks/extreme_parkour_task/config/dogv2/parkour_teacher_cfg_custom.py`  
**用途：** ✅ 训练 Teacher 策略

#### 配置特点
- 环境数量：6144
- 地形规模：10行 × 40列 = 400块
- 难度范围：0.0 - 1.0（启用 curriculum learning）
- Episode 时长：20秒
- 命令切换间隔：6秒
- Domain Randomization：✅ 完全启用
  - 质量随机化：base_link ±1-3kg
  - 质心随机化：±2cm
  - 推动扰动：每8秒，±0.5m/s
  - 摩擦系数：0.6-2.0

#### 使用命令
```bash
# 从头开始训练
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --headless \
    --num_envs 4096

# 恢复训练
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --headless \
    --num_envs 4096 \
    --resume \
    --load_run <run_folder_name> \
    --checkpoint model_10000.pt
```

---

### 2. Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0

**配置类：** `DogV2TeacherParkourEnvCfg_EVAL`  
**用途：** 📊 评估训练中的 Teacher 模型

#### 配置特点
- 环境数量：256（减少环境数以加快评估）
- 地形规模：5行 × 5列 = 25块
- 难度范围：0.0 - 1.0（随机难度）
- Episode 时长：20秒
- 可视化：开启
- Domain Randomization：❌ 禁用（评估时保持一致性）

#### 使用命令
```bash
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0 \
    --num_envs 256 \
    --load_run <run_folder_name> \
    --checkpoint model_10000.pt
```

---

### 3. Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0

**配置类：** `DogV2TeacherParkourEnvCfg_PLAY`  
**用途：** 🎮 可视化展示已训练好的 Teacher 模型

#### 配置特点
- 环境数量：16（便于观察）
- 地形规模：5行 × 5列 = 25块
- 难度范围：0.7 - 1.0（展示高难度场景）
- Episode 时长：60秒（长时间观察）
- 命令切换间隔：60秒（稳定展示）
- Domain Randomization：❌ 禁用
- 可视化：完全开启

#### 使用命令
```bash
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/dogv2_parkour/<run_name>/model_50000.pt

# 录制视频
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0 \
    --num_envs 4 \
    --checkpoint <path_to_checkpoint> \
    --video \
    --video_length 400
```

---

## 🎓 Student 任务

Student 策略使用深度相机图像作为主要输入，通过知识蒸馏从 Teacher 学习，适合部署到实际机器人。

### 1. Isaac-Extreme-Parkour-Student-DogV2-v0

**配置类：** `DogV2StudentParkourEnvCfg`  
**配置文件：** `parkour_tasks/parkour_tasks/extreme_parkour_task/config/dogv2/parkour_student_cfg_custom.py`  
**用途：** ✅ 训练 Student 策略（知识蒸馏）  
**算法：** DistillationWithExtractor

#### 配置特点
- 环境数量：192（深度相机渲染消耗大）
- 地形规模：10行 × 20列 = 200块
- 水平缩放：0.1（更密集的地形）
- 深度相机：✅ 启用（87×58 分辨率）
- Episode 时长：20秒
- 动作延迟：✅ 启用（history_length=8）
- 简化地形：✅ 启用（use_simplified=True）
- Domain Randomization：✅ 完全启用

#### 输入观测
- 深度相机图像（87×58，2帧历史）
- 本体感知信息
- 动作历史（8帧）

#### 使用命令
```bash
# 需要先训练好 Teacher 模型
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-v0 \
    --headless \
    --num_envs 192 \
    --load_run <teacher_run_name> \
    --checkpoint <teacher_checkpoint.pt>
```

---

### 2. Isaac-Extreme-Parkour-Student-DogV2-Eval-v0

**配置类：** `DogV2StudentParkourEnvCfg_EVAL`  
**用途：** 📊 评估 Student 模型性能

#### 配置特点
- 环境数量：256
- 地形规模：5行 × 5列 = 25块
- 深度相机可视化：✅ 开启
- 相机扰动：轻微俯仰角噪声（0-1度）
- Domain Randomization：❌ 禁用

#### 使用命令
```bash
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Eval-v0 \
    --num_envs 256 \
    --load_run <student_run_name> \
    --checkpoint model_10000.pt
```

---

### 3. Isaac-Extreme-Parkour-Student-DogV2-Play-v0

**配置类：** `DogV2StudentParkourEnvCfg_PLAY`  
**用途：** 🎮 可视化展示已训练好的 Student 模型

#### 配置特点
- 环境数量：16
- 难度范围：0.7 - 1.0（高难度展示）
- Episode 时长：60秒
- 深度相机显示：✅ 可看到实际输入图像
- Domain Randomization：❌ 禁用

#### 使用命令
```bash
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Play-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/dogv2_parkour/<student_run>/model_50000.pt
```

---

## 📊 Teacher vs Student 对比

| 特性 | Teacher | Student |
|------|---------|---------|
| **主要输入** | Height scan + 本体感知 | 深度相机图像 + 本体感知 |
| **训练环境数** | 4096-6144 | 192 |
| **地形规模** | 10×40（400块） | 10×20（200块） |
| **训练算法** | PPO（从零开始） | Distillation（从Teacher学习） |
| **动作延迟** | 可选 | ✅ 启用（history=8） |
| **深度相机** | ❌ 不需要 | ✅ 需要 |
| **训练依赖** | 无 | 需要已训练的Teacher模型 |
| **计算成本** | 中等 | 高（需要渲染深度图） |
| **部署场景** | 仿真环境 | 实际机器人 |
| **Reward** | 完整奖励函数 | 简化奖励（collision权重=0） |

---

## 🔄 完整训练流程

### 阶段 1：训练 Teacher

```bash
# Step 1: 启动 Teacher 训练
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --headless \
    --num_envs 4096

# Step 2: 定期评估 Teacher
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0 \
    --num_envs 256 \
    --load_run 2026-01-20_10-30-15 \
    --checkpoint model_20000.pt

# Step 3: 训练完成后可视化
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/dogv2_parkour/2026-01-20_10-30-15/model_50000.pt
```

### 阶段 2：训练 Student（可选）

```bash
# Step 1: 使用训练好的 Teacher 进行知识蒸馏
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-v0 \
    --headless \
    --num_envs 192 \
    --load_run 2026-01-20_10-30-15 \
    --checkpoint model_50000.pt

# Step 2: 评估 Student
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Eval-v0 \
    --num_envs 256 \
    --load_run 2026-01-21_15-20-30 \
    --checkpoint model_15000.pt

# Step 3: 可视化 Student（可以看到深度图）
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-Play-v0 \
    --num_envs 16 \
    --checkpoint logs/rsl_rl/dogv2_parkour/2026-01-21_15-20-30/model_50000.pt
```

---

## 🎯 使用场景建议

### 仅在仿真中测试
只需要训练 **Teacher**：

```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --headless --num_envs 4096
```

### 部署到真实机器人
需要训练 **Teacher → Student**：

```bash
# 1. 先训练 Teacher
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --headless --num_envs 4096

# 2. 等 Teacher 训练完成后，用其训练 Student
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Student-DogV2-v0 \
    --headless --num_envs 192 \
    --load_run <teacher_run> \
    --checkpoint <teacher_model.pt>
```

### 快速调试/测试
使用 **Eval-v0** 任务：

```bash
python scripts/rsl_rl/evaluation.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0 \
    --num_envs 64  # 更少环境，更快
```

### 展示效果/录制视频
使用 **Play-v0** 任务：

```bash
python scripts/rsl_rl/play.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Play-v0 \
    --num_envs 4 \
    --video \
    --video_length 400
```

---

## ⚠️ 常见错误

### 错误 1：用 train.py 训练 Play-v0 配置
- **问题：** Play-v0 配置不适合训练（地形少、难度高、episode长）
- **解决：** 改用 `v0` 配置进行训练

### 错误 2：没有 Teacher 就训练 Student
- **问题：** Student 使用知识蒸馏，需要 Teacher 模型
- **解决：** 先训练 Teacher，再用 `--load_run` 和 `--checkpoint` 指定 Teacher 模型

### 错误 3：Student 使用过多环境
- **问题：** 深度相机渲染消耗大，过多环境会导致 GPU 内存不足
- **解决：** 保持 192 个环境或更少

### 错误 4：混淆 v0、Eval-v0 和 Play-v0
- **v0：** 专门为训练优化（大量地形、curriculum、domain randomization）
- **Eval-v0：** 用于评估性能（中等环境数、禁用随机化）
- **Play-v0：** 用于可视化展示（少量环境、高难度、长 episode）

### 错误 5：Debug模式下slope地形不显示
- **问题：** 使用 `--debug` 参数时，slope地形（parkour_slope）不出现
- **原因：** Debug模式下 `num_cols=5`，而slope地形的比例范围是0.85-1.0，需要至少7列才能显示
- **解决：** 已在 `scripts/rsl_rl/train.py` 中修复，debug模式下自动设置为8列（为未来可能新增的地形类型预留空间）

### 错误 6：Evaluation脚本找不到checkpoint文件
- **问题：** 使用 `--checkpoint model_5500.pt` 时提示文件未找到
- **解决：** 使用完整路径或相对路径：
  ```bash
  # 使用相对路径（推荐）
  python scripts/rsl_rl/evaluation.py \
      --task Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0 \
      --num_envs 16 \
      --checkpoint logs/rsl_rl/dogv2_parkour/2026-01-20_18-20-56/model_5500.pt
  
  # 或使用绝对路径
  --checkpoint /home/ares/IsaacLab/Isaaclab_Parkour/logs/rsl_rl/dogv2_parkour/2026-01-20_18-20-56/model_5500.pt
  ```

---

## 📁 相关配置文件

### Teacher 配置
```
parkour_tasks/parkour_tasks/extreme_parkour_task/config/dogv2/
├── parkour_teacher_cfg_custom.py          # 环境配置
├── parkour_mdp_cfg_custom.py              # MDP配置（奖励、观测等）
└── agents/
    └── rsl_teacher_ppo_cfg_custom.py      # PPO算法配置
```

### Student 配置
```
parkour_tasks/parkour_tasks/extreme_parkour_task/config/dogv2/
├── parkour_student_cfg_custom.py          # 环境配置
├── parkour_mdp_cfg_custom.py              # MDP配置（共享）
└── agents/
    └── rsl_student_ppo_cfg_custom.py      # Distillation算法配置
```

### 地形配置
```
parkour_isaaclab/terrains/extreme_parkour/config/
└── parkour.py                              # 地形类型和比例配置
```

#### 地形类型说明
- **parkour_gap**: 间隙地形（15%）
- **parkour_hurdle**: 跨栏地形（20%）
- **parkour_flat**: 平坦跨栏地形（20%）
- **parkour_step**: 台阶地形（15%）
- **parkour**: 复杂障碍地形（15%）
- **parkour_slope**: 斜坡地形（15%）- 前进方向的上坡/下坡
- **parkour_demo**: 演示地形（0%，已禁用）

#### 修改斜坡坡度
斜坡的坡度在 `parkour.py` 文件的 `parkour_slope` 配置中修改：

```python
"parkour_slope": ExtremeParkourSlopeTerrainCfg(
    proportion=0.15,
    apply_roughness=True,
    x_range=(1.0, 2.0),
    half_valid_width=(0.5, 1.0),
    slope_range="-0.15 - 0.05 * difficulty, 0.15 + 0.05 * difficulty",  # ← 修改这里
    segment_width_range="0.8 + 0.2 * difficulty, 1.6 + 0.4 * difficulty",
    noise_range=(0.01, 0.05),
),
```

**参数说明：**
- `slope_range`: 斜率范围（单位：米高度/米前进方向）
  - 格式：`"最小值, 最大值"`，支持使用 `difficulty` 变量
  - 正值 = 上坡，负值 = 下坡
  - 当前值：`"-0.15 - 0.05 * difficulty, 0.15 + 0.05 * difficulty"`
    - difficulty=0 时：-0.15 到 0.15（约 ±8.5°）
    - difficulty=1 时：-0.2 到 0.2（约 ±11.3°）

**修改示例：**
```python
# 更陡的坡度（±30度左右）
slope_range="-0.5 - 0.1 * difficulty, 0.5 + 0.1 * difficulty"

# 固定坡度（不随难度变化）
slope_range="-0.2, 0.2"

# 只有上坡
slope_range="0.1, 0.3"

# 只有下坡
slope_range="-0.3, -0.1"
```

**注意：** 斜坡方向已修复为在机器人前进方向（x方向）上变化，不再是左右倾斜。

---

## 📈 训练监控

### TensorBoard
训练过程中的指标会自动记录到：
```
logs/rsl_rl/dogv2_parkour/<timestamp>/
```

查看训练曲线：
```bash
tensorboard --logdir logs/rsl_rl/dogv2_parkour
```

### 关键指标
- `Loss/value_function`：值函数损失
- `Loss/surrogate`：策略损失
- `Policy/mean_reward`：平均奖励
- `Policy/mean_episode_length`：平均 episode 长度
- `Train/mean_std`：动作标准差

---

## 🔧 调试技巧

### 减少环境数进行快速测试
```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --num_envs 64  # 最少64个环境
```

### 启用可视化调试（Debug模式）
使用 `--debug` 参数可以：
- 自动减少环境数到64（确保是4的倍数）
- 设置地形为5行×8列（确保所有地形类型都能显示，包括slope，并为未来新增类型预留空间）
- 切换到tensorboard日志（禁用wandb以节省内存）

```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-v0 \
    --debug
```

### 启用可视化调试（Eval配置）
```bash
python scripts/rsl_rl/train.py \
    --task Isaac-Extreme-Parkour-Teacher-DogV2-Eval-v0 \
    --num_envs 16  # 使用Eval配置，自动开启可视化
```

### 检查生成的配置文件
训练启动后，实际使用的配置会保存在：
```
logs/rsl_rl/dogv2_parkour/<timestamp>/params/
├── agent.yaml  # 算法配置
└── env.yaml    # 环境配置
```

可以检查这些文件确认配置是否正确。

---

## ⚙️ 重要配置说明

### Teacher动作延迟（Delay）设置

Teacher策略默认**不启用**动作延迟：

**配置文件：** `parkour_tasks/parkour_tasks/extreme_parkour_task/config/dogv2/parkour_teacher_cfg_custom.py`

```python
def __post_init__(self):
    # ...
    self.actions.joint_pos.use_delay = False  # Teacher不使用delay
    self.actions.joint_pos.history_length = 1
```

**对比：**
- **Teacher**: `use_delay = False`, `history_length = 1`
- **Student**: `use_delay = True`, `history_length = 8`

如果需要为Teacher启用delay，修改上述配置即可。

### 斜坡地形修复说明

**修复内容：**
1. **方向修复**：斜坡从左右倾斜（y方向）改为前进方向（x方向）的上坡/下坡
2. **Debug模式修复**：Debug模式下 `num_cols` 从5改为7，确保slope地形能正常显示

**相关文件：**
- `parkour_isaaclab/terrains/extreme_parkour/extreme_parkour_terrians.py` - 斜坡生成函数
- `parkour_isaaclab/terrains/extreme_parkour/extreme_parkour_terrains_cfg.py` - 斜坡配置类
- `scripts/rsl_rl/train.py` - Debug模式配置

---

## 📞 技术支持

如有问题，请检查：
1. 配置文件是否正确修改
2. 任务名称是否拼写正确
3. checkpoint 路径是否存在（使用完整路径）
4. GPU 内存是否足够
5. Debug模式下是否能看到所有地形类型（包括slope）

相关文档：
- [Isaac Lab 官方文档](https://isaac-sim.github.io/IsaacLab/)
- [RSL-RL 文档](https://github.com/leggedrobotics/rsl_rl)
