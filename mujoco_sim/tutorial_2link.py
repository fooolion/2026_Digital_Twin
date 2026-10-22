# -*- coding: utf-8 -*-
"""MuJoCo 入门脚本：两连杆臂 —— 跑起来 + 正运动学(FK) + 动力学演示。

用法（在项目根目录 D:\\Study\\Zju\\Competition\\2026_Digital_Twin 下运行）：

    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/tutorial_2link.py
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/tutorial_2link.py --view

加 --view 会打开可视化窗口，你可以用鼠标直接拖关节玩。
"""
import math
import sys

import numpy as np
import mujoco

MODEL = "mujoco_sim/models/arm_2link.xml"


def deg2rad(d: float) -> float:
    """MuJoCo 的 qpos 内部用弧度，文件里写的是度，所以要换算。"""
    return d * math.pi / 180.0


def main() -> None:
    # 1) 加载模型 + 新建一份仿真数据
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    print("关节数量:", model.njnt)
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"    关节[{i}] = {name}")

    j1 = model.jnt_qposadr[model.joint("joint1").id]   # joint1 在 qpos 数组里的下标
    j2 = model.jnt_qposadr[model.joint("joint2").id]
    site = model.site("end_effector").id              # 末端标记点的 id

    # 2) 正运动学 FK：给一组关节角，算末端在世界坐标里的位置
    data.qpos[j1] = deg2rad(30.0)
    data.qpos[j2] = deg2rad(-60.0)
    mujoco.mj_forward(model, data)
    print("\n[FK] 关节角(30°, -60°)  末端世界坐标 =", np.round(data.site_xpos[site], 4))

    # 3) 动力学演示：把手臂斜着摆，让重力把它拉倒，看末端位置变化
    data.qpos[j1] = deg2rad(45.0)
    data.qpos[j2] = deg2rad(30.0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    before = data.site_xpos[site].copy()
    for _ in range(1000):                       # 1000 步 × 0.002s = 2 秒
        mujoco.mj_step(model, data)
    print("[动力学] 斜摆后自由落体 2 秒：末端", np.round(before, 3),
          "->", np.round(data.site_xpos[site], 3))
    print("   (末端动了，说明 MuJoCo 在算真实的重力/动力学，而不只是摆姿势)\n")

    # 4) 可选：打开可视化窗口（需要 glfw）
    if "--view" in sys.argv:
        try:
            from mujoco import viewer  # 只导入 viewer 名字，避免把 mujoco 变成本地变量
            viewer.launch(model, data)   # launch = 阻塞版，窗口会一直开着，关窗才继续
        except ImportError:
            print("看不到窗口？缺少依赖（glfw），运行下面命令装上：")
            print("   mujoco_sim/.rebot/Scripts/python.exe -m pip install glfw")


if __name__ == "__main__":
    main()