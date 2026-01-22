#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""
Parkour机器人完整控制脚本
整合功能:
1. Raycaster深度相机可视化
2. Joystick手柄控制
3. ONNX策略推理
4. Parkour地形
"""
import mujoco
import mujoco_viewer
import numpy as np
import time
import os
import struct
import threading
import argparse
from pathlib import Path
from collections import deque
from scipy.spatial.transform import Rotation as R

try:
    import onnxruntime as ort

    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    print("⚠ 警告: onnxruntime未安装，策略推理功能不可用")

# 获取脚本目录
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent
S2S_DIR = SCRIPT_DIR.parent


# ============================================================================ #
#                              Joystick Interface                              #
# ============================================================================ #


class JoystickInterface:
    """游戏手柄接口，用于读取手柄输入"""

    def __init__(
        self, device_path="/dev/input/js0", max_v_x=1.0, max_v_y=0.5, max_omega=1.0
    ):
        self.device_path = device_path
        self.running = True

        # 当前指令缓存 (线程安全)
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_yaw = 0.0

        # 速度限制
        self.MAX_V_X = max_v_x  # m/s
        self.MAX_V_Y = max_v_y  # m/s
        self.MAX_OMEGA = max_omega  # rad/s

        # 摇杆原始数值范围
        self.JOY_MAX = 32767.0

        # 启动读取线程
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()

    def _read_loop(self):
        """后台线程：持续读取手柄输入"""
        if not os.path.exists(self.device_path):
            print(f"[Joystick] ⚠ 未找到设备 {self.device_path}")
            print(f"[Joystick] 将以键盘模式运行")
            self.running = False
            return

        print(f"[Joystick] ✓ 监听设备: {self.device_path}")

        event_format = "IhBB"  # struct: time, value, type, number
        event_size = struct.calcsize(event_format)

        try:
            with open(self.device_path, "rb") as js_file:
                while self.running:
                    event_data = js_file.read(event_size)
                    if event_data:
                        time_evt, value, type_evt, number = struct.unpack(
                            event_format, event_data
                        )

                        # 过滤掉初始化信号 (0x80)
                        if type_evt & 0x80:
                            continue

                        # 处理轴事件 (Type 2 = Axis)
                        if type_evt == 0x02:
                            # 归一化到 -1.0 ~ 1.0
                            norm_val = value / self.JOY_MAX

                            # 死区处理 (防止漂移)
                            if abs(norm_val) < 0.1:
                                norm_val = 0.0

                            # Xbox 映射
                            # Axis 1: 左摇杆上下 -> 前进后退(x)
                            if number == 1:
                                self.cmd_x = -norm_val * self.MAX_V_X

                            # Axis 0: 左摇杆左右 -> 左右平移(y)
                            elif number == 0:
                                self.cmd_y = -norm_val * self.MAX_V_Y

                            # Axis 3: 右摇杆左右 -> 旋转(yaw)
                            elif number == 3:
                                self.cmd_yaw = -norm_val * self.MAX_OMEGA

        except Exception as e:
            print(f"[Joystick] 读取错误: {e}")

    def get_command(self):
        """获取当前手柄指令，返回 (vx, vy, dyaw)"""
        return self.cmd_x, self.cmd_y, self.cmd_yaw

    def stop(self):
        """停止手柄读取线程"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join()


# ============================================================================ #
#                           Remapping Indices                                  #
# ============================================================================ #

# MuJoCo -> Policy 顺序映射
sim2policy_indices = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)

# Policy -> MuJoCo 顺序映射
policy2sim_indices = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)


# ============================================================================ #
#                              Configuration                                   #
# ============================================================================ #


class PolicyConfig:
    """策略配置"""

    class sim_config:
        sim_duration = 120.0
        dt = 0.005
        decimation = 4  # 策略频率 = 200Hz / 4 = 50Hz

    class robot_config:
        num_actions = 12
        default_dof_pos = np.zeros(12, dtype=np.double)
        kps = np.full(12, 25.0, dtype=np.double)
        kds = np.full(12, 0.5, dtype=np.double)
        tau_limit = np.array([17, 17, 25] * 4, dtype=np.double)

    class normalization:
        class isaac_obs_scales:
            lin_vel = 1.0
            ang_vel = 1.0
            projected_gravity = 1.0
            commands = 1.0
            joint_pos = 1.0
            joint_vel = 1.0
            actions = 1.0

        clip_observations = 100.0
        clip_actions = 100.0

    class env:
        frame_stack = 10
        num_single_obs = 45

    class control:
        action_scale = 0.25


# ============================================================================ #
#                              Utility Functions                               #
# ============================================================================ #


def get_gravity_orientation(quat):
    """计算重力在基座坐标系下的投影"""
    r = R.from_quat(quat)
    gravity_vec = np.array([0.0, 0.0, -1.0])
    return r.apply(gravity_vec, inverse=True)


def get_obs(data):
    """从MuJoCo数据中提取观测值，并映射到Policy顺序"""
    q_sim = data.qpos[7:].astype(np.double)
    dq_sim = data.qvel[6:].astype(np.double)

    q_policy = q_sim[sim2policy_indices]
    dq_policy = dq_sim[sim2policy_indices]

    mj_quat = data.qpos[3:7]
    quat = np.array([mj_quat[1], mj_quat[2], mj_quat[3], mj_quat[0]])

    omega = data.sensor("angular-velocity").data.astype(np.double)

    return q_policy, dq_policy, quat, omega


def pd_control(target_q, q, kp, target_dq, dq, kd, default_pos):
    """PD控制器"""
    return (target_q - q) * kp + (target_dq - dq) * kd


# ============================================================================ #
#                                 Main Function                                #
# ============================================================================ #


def main(args):
    """主函数"""
    cfg = PolicyConfig()

    # ---------------- 配置区域 ----------------
    possible_plugin_paths = [
        "/home/ares/mujoco_ray_caster/lib/libsensor_ray.so",
        (PROJECT_ROOT / "lib" / "libsensor_ray.so").as_posix(),
        (S2S_DIR.parent / "lib" / "libsensor_ray.so").as_posix(),
    ]

    model_xml_path = (S2S_DIR / "robot_parkour_with_raycaster.xml").as_posix()
    SENSOR_NAME = "ray_caster_camera"
    # ----------------------------------------

    print("=" * 60)
    print("Parkour机器人 - 策略控制 + Raycaster可视化")
    print("=" * 60)

    # 1. 加载raycaster插件
    plugin_loaded = False
    for plugin_path in possible_plugin_paths:
        if not Path(plugin_path).exists():
            continue
        try:
            mujoco.mj_loadPluginLibrary(plugin_path)
            print(f"✓ Raycaster插件: {plugin_path}")
            plugin_loaded = True
            break
        except Exception as e:
            continue

    if not plugin_loaded:
        print(f"⚠ Raycaster插件未加载")

    # 2. 加载ONNX模型
    ort_session = None
    if args.load_model and ONNX_AVAILABLE:
        try:
            ort_session = ort.InferenceSession(args.load_model)
            print(f"✓ ONNX模型: {args.load_model}")
        except Exception as e:
            print(f"✗ ONNX模型加载失败: {e}")
            if args.require_model:
                return
            print("  继续运行（无策略模式）")
    else:
        if args.require_model:
            print("⚠ 未指定ONNX模型")
            print("  使用 --load_model 参数指定模型路径")
            return
        print("⚠ 无策略模式运行（仅PD控制）")

    # 3. 加载MuJoCo模型
    try:
        model = mujoco.MjModel.from_xml_path(model_xml_path)
        model.opt.timestep = cfg.sim_config.dt
        data = mujoco.MjData(model)
        print(f"✓ MuJoCo模型: {model_xml_path}")
    except Exception as e:
        print(f"✗ 模型加载失败: {e}")
        return

    # 初始化关节位置
    data.qpos[7:] = cfg.robot_config.default_dof_pos

    # 4. 预热仿真
    print("\n预热仿真 (50 steps)...")
    for _ in range(50):
        mujoco.mj_step(model, data)

    # 5. 传感器诊断
    try:
        sensor_data = data.sensor(SENSOR_NAME).data
        valid_data = sensor_data[(sensor_data > 0.01) & (sensor_data < 2.0)]
        print("-" * 40)
        print(f"📊 传感器诊断:")
        if len(valid_data) > 0:
            print(f"   ✓ 检测到有效障碍物点数: {len(valid_data)}")
            print(f"   - 最小距离: {np.min(valid_data):.3f} m")
            print(f"   - 最大距离: {np.max(valid_data):.3f} m")
        else:
            print(f"   ⚠ 未检测到障碍物")
        print("-" * 40)
    except Exception as e:
        print(f"⚠ 无法读取传感器数据: {e}")

    # 6. 初始化手柄
    joy = JoystickInterface(
        device_path="/dev/input/js0", max_v_x=2.0, max_v_y=1.0, max_omega=1.5
    )

    print("\n" + "=" * 60)
    print("控制说明:")
    print("  手柄:")
    print("    - 左摇杆: 前后左右移动")
    print("    - 右摇杆(左右): 旋转")
    print("  Viewer:")
    print("    - 左键双击: 跟踪物体")
    print("    - 右键拖动: 平移")
    print("    - Ctrl+右键: 旋转")
    print("    - 滚轮: 缩放")
    print("    - Tab: 打开GUI (查看射线可视化)")
    print("=" * 60)

    # 7. 启动Viewer
    print("\n启动Viewer...")
    viewer = mujoco_viewer.MujocoViewer(model, data)

    # 设置相机
    viewer.cam.lookat[:] = [1.0, 0, 0.5]
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 130
    viewer.cam.elevation = -15

    # 8. 初始化状态变量
    action_policy = np.zeros(cfg.robot_config.num_actions, dtype=np.double)
    target_q_sim = np.zeros(cfg.robot_config.num_actions, dtype=np.double)

    hist_obs = deque(maxlen=cfg.env.frame_stack)
    for _ in range(cfg.env.frame_stack):
        hist_obs.append(np.zeros(cfg.env.num_single_obs, dtype=np.float32))

    count_lowlevel = 0
    scales = cfg.normalization.isaac_obs_scales

    print("✓ Viewer已启动\n")

    # 9. 仿真循环
    try:
        while viewer.is_alive:
            # 获取观测
            q_policy, dq_policy, quat, omega = get_obs(data)

            # 计算实际速度
            vel_world = data.qvel[:3]
            r_temp = R.from_quat(quat)
            vel_body = r_temp.apply(vel_world, inverse=True)

            # 获取手柄指令
            cmd_x, cmd_y, cmd_yaw = joy.get_command()

            # 策略推理 (50Hz)
            if ort_session and count_lowlevel % cfg.sim_config.decimation == 0:
                obs_list = []

                # 角速度
                obs_list.append(omega * scales.ang_vel)

                # 重力投影
                obs_list.append(
                    get_gravity_orientation(quat) * scales.projected_gravity
                )

                # 手柄指令
                current_cmd = np.array([cmd_x, cmd_y, cmd_yaw], dtype=np.double)
                obs_list.append(current_cmd * scales.commands)

                # 关节位置
                dof_pos_rel = q_policy - cfg.robot_config.default_dof_pos
                obs_list.append(dof_pos_rel * scales.joint_pos)

                # 关节速度
                obs_list.append(dq_policy * scales.joint_vel)

                # 上一帧动作
                obs_list.append(action_policy * scales.actions)

                # 构造观测
                current_obs = np.concatenate(obs_list).astype(np.float32)
                current_obs = np.clip(
                    current_obs,
                    -cfg.normalization.clip_observations,
                    cfg.normalization.clip_observations,
                )
                hist_obs.append(current_obs)

                # ONNX推理
                policy_input = np.concatenate(hist_obs)[None, :]
                input_name = ort_session.get_inputs()[0].name
                raw_action = ort_session.run(None, {input_name: policy_input})[0][0]

                action_policy = np.clip(
                    raw_action,
                    -cfg.normalization.clip_actions,
                    cfg.normalization.clip_actions,
                )

                # Policy -> Sim 顺序
                action_sim = action_policy[policy2sim_indices]
                target_q_sim = (
                    action_sim * cfg.control.action_scale
                    + cfg.robot_config.default_dof_pos
                )

            # PD控制
            q_sim_raw = data.qpos[7:]
            dq_sim_raw = data.qvel[6:]

            tau = pd_control(
                target_q_sim,
                q_sim_raw,
                cfg.robot_config.kps,
                np.zeros_like(dq_sim_raw),
                dq_sim_raw,
                cfg.robot_config.kds,
                0.0,
            )
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)

            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
            viewer.render()

            # 打印状态
            if count_lowlevel % 20 == 0:
                mode = "策略" if ort_session else "PD"
                print(
                    f"\r[{mode}] 指令: x={cmd_x:+.2f} y={cmd_y:+.2f} yaw={cmd_yaw:+.2f} | "
                    f"速度: x={vel_body[0]:+.2f} y={vel_body[1]:+.2f} z={vel_body[2]:+.2f}  ",
                    end="",
                    flush=True,
                )

            count_lowlevel += 1

    except KeyboardInterrupt:
        print("\n\n程序已停止")
    except Exception as e:
        print(f"\n运行时错误: {e}")
        import traceback

        traceback.print_exc()
    finally:
        joy.stop()
        if "viewer" in locals() and viewer.is_alive:
            viewer.close()

    print("\n✓ 程序已退出")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Parkour机器人策略控制（带Raycaster可视化）"
    )
    parser.add_argument(
        "--load_model",
        type=str,
        help="ONNX模型路径（可选，不提供则使用PD控制）",
    )
    parser.add_argument(
        "--require_model",
        action="store_true",
        help="是否必须加载模型（如果设置，没有模型时退出）",
    )
    args = parser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback

        traceback.print_exc()
