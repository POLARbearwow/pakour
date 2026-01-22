import mujoco
import mujoco_viewer
import numpy as np
import time
import sys

# ================= 配置路径 =================
# 请确保这里指向真实存在的 .so 文件路径
PLUGIN_PATH = "/home/ares/mujoco_ray_caster/lib/libsensor_ray.so"
XML_PATH = "robot_parkour_with_raycaster.xml"
SENSOR_NAME = "ray_caster_camera"
# ===========================================


def main():
    print(f"1. 正在加载插件: {PLUGIN_PATH}")
    try:
        mujoco.mj_loadPluginLibrary(PLUGIN_PATH)
    except Exception as e:
        print(f"❌ 致命错误: 插件加载失败 - {e}")
        return

    print(f"2. 正在加载模型: {XML_PATH}")
    try:
        model = mujoco.MjModel.from_xml_path("/home/ares/IsaacLab/Isaaclab_Parkour/s2s/robot_parkour_with_raycaster.xml")
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return

    # 运行 50 步让传感器热身
    print("3. 正在预热仿真 (50 steps)...")
    for _ in range(50):
        mujoco.mj_step(model, data)

    # 检查是否有数据
    sensor_data = data.sensor(SENSOR_NAME).data

    # 过滤掉 0 和 inf，看看有没有有效距离
    valid_data = sensor_data[(sensor_data > 0.01) & (sensor_data < 2.0)]

    print("-" * 40)
    print(f"📊 传感器数据诊断:")
    print(f"   - 原始数据长度: {len(sensor_data)}")
    print(f"   - 原始数据示例 (前10个): {sensor_data[:10]}")

    if len(valid_data) > 0:
        print(f"   - ✅ 成功检测到障碍物! (有效点数: {len(valid_data)})")
        print(f"   -    最小距离: {np.min(valid_data):.3f} m")
        print(f"   -    最大距离: {np.max(valid_data):.3f} m")
    else:
        print(f"   - ⚠️  数据全为 0 或 inf。可能原因：")
        print(f"        1. 面前没有障碍物 (请检查 XML 场景)")
        print(f"        2. 几何体 Group 设置不对 (sensor 配置了 geomgroup)")
        print(f"        3. 插件未正确初始化")
    print("-" * 40)

    print("4. 启动 Viewer...")
    print("👉 请在弹出的窗口中做以下检查：")
    print("   [1] 按 'Tab' 打开右侧菜单")
    print("   [2] 找到 'Rendering' -> 'Geoms'")
    print("   [3] 确保勾选了 'Group 2' (射线通常画在这个组)")
    print("   [4] 勾选 'Decor' (装饰性几何体)")

    # with mujoco.viewer.launch_passive(model, data) as viewer:
    #     # 调整视角看机器人前方
    #     viewer.cam.lookat[:] = [1.0, 0, 0.5]
    #     viewer.cam.distance = 2.0
    #     viewer.cam.azimuth = 130

    viewer = mujoco_viewer.MujocoViewer(model, data)

    while viewer.is_alive:
        step_start = time.time()
        mujoco.mj_step(model, data)
        viewer.render()

        # # 保持 60Hz 刷新
        # time_until_next_step = model.opt.timestep - (time.time() - step_start)
        # if time_until_next_step > 0:
        #     time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
