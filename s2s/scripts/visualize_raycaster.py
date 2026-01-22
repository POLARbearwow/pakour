#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""
Raycaster相机可视化脚本 (基于 mujoco-python-viewer)
"""
import mujoco
import mujoco_viewer  # 使用你验证成功的第三方viewer
import sys
import time
import numpy as np
from pathlib import Path

# 获取脚本目录和项目根目录
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent
S2S_DIR = SCRIPT_DIR.parent


def main():
    """主函数"""
    # ---------------- 配置区域 ----------------
    # 配置文件路径 - 尝试多个可能的插件路径
    possible_plugin_paths = [
        "/home/ares/mujoco_ray_caster/lib/libsensor_ray.so",
        (PROJECT_ROOT / "lib" / "libsensor_ray.so").as_posix(),
        (S2S_DIR.parent / "lib" / "libsensor_ray.so").as_posix(),
    ]

    model_xml_path = (S2S_DIR / "robot_parkour_with_raycaster.xml").as_posix()
    SENSOR_NAME = "ray_caster_camera"
    # ----------------------------------------

    print("=" * 60)
    print("Raycaster可视化 (mujoco-python-viewer版)")
    print("=" * 60)

    # 1. 加载raycaster插件
    plugin_loaded = False
    for plugin_path in possible_plugin_paths:
        if not Path(plugin_path).exists():
            continue
        try:
            mujoco.mj_loadPluginLibrary(plugin_path)
            print(f"✓ 成功加载插件: {plugin_path}")
            plugin_loaded = True
            break
        except Exception as e:
            print(f"  尝试 {plugin_path}: 失败 ({e})")
            continue

    if not plugin_loaded:
        print(f"✗ 无法加载插件，请检查路径或重新编译。")
        return

    # 2. 加载MuJoCo模型
    try:
        model = mujoco.MjModel.from_xml_path(model_xml_path)
        data = mujoco.MjData(model)
        print(f"✓ 成功加载模型: {model_xml_path}")
    except Exception as e:
        print(f"✗ 加载模型失败: {e}")
        return

    # 3. 预热仿真 (关键步骤)
    # 就像你的 test_raycaster.py 里做的一样，先跑50步让插件初始化
    print("3. 正在预热仿真 (50 steps) 以初始化传感器...")
    for _ in range(50):
        mujoco.mj_step(model, data)

    # 4. 数据诊断 (确认射线是否工作)
    try:
        sensor_data = data.sensor(SENSOR_NAME).data
        # 过滤有效数据 (0.01 < dist < 2.0)
        valid_data = sensor_data[(sensor_data > 0.01) & (sensor_data < 2.0)]

        print("-" * 40)
        print(f"📊 传感器诊断:")
        if len(valid_data) > 0:
            print(f"   ✓ 检测到有效障碍物点数: {len(valid_data)}")
            print(f"   - 最小距离: {np.min(valid_data):.3f} m")
            print(f"   - 最大距离: {np.max(valid_data):.3f} m")
        else:
            print(f"   ⚠ 未检测到障碍物 (数据全为0或inf)，请检查相机前方是否有遮挡")
        print("-" * 40)
    except Exception as e:
        print(f"⚠ 无法读取传感器数据 (可能名称不匹配): {e}")

    # 5. 启动 Viewer
    print("\n启动 Viewer...")
    print("👉 操作指南:")
    print("   - 左键双击: 跟踪物体")
    print("   - 右键拖动: 平移")
    print("   - Ctrl+右键: 旋转")
    print("   - 滚轮: 缩放")
    print("   - Tab键: 如果看不到红色射线，请按Tab打开GUI检查 Geoms -> Group 2")

    # 使用 mujoco_viewer (你验证过可行的方法)
    viewer = mujoco_viewer.MujocoViewer(model, data)

    # 设置初始视角 (可选)
    viewer.cam.lookat[:] = [1.0, 0, 0.5]  # 看向前方
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 130

    # 仿真循环
    try:
        while viewer.is_alive:
            step_start = time.time()

            # 物理步进
            mujoco.mj_step(model, data)

            # 渲染
            viewer.render()

            # 简单的帧率控制 (非严格)
            # time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n程序已停止")
    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        if "viewer" in locals() and viewer.is_alive:
            viewer.close()


if __name__ == "__main__":
    main()
