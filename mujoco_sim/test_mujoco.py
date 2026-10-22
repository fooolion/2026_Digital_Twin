"""MuJoCo 环境冒烟测试：验证物理引擎能正常加载模型并步进仿真。

运行方式（在项目根目录）:
    mujoco_sim/.rebot/Scripts/python.exe mujoco_sim/test_mujoco.py
"""
import numpy as np
import mujoco

# 用字符串直接构造一个最小场景：一个平地 + 一个带自由关节的小方块
xml = """
<mujoco>
  <worldbody>
    <light pos="0 0 5"/>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="box" pos="0 0 0.5">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="0.1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

for _ in range(200):
    mujoco.mj_step(model, data)

print("MuJoCo 版本:", mujoco.__version__)
print("方块初始 z=0.5，落地后 z 应约为 0.05（方块半高）")
print("最终 qpos:", np.round(data.qpos, 4))
print("方块 z 坐标:", round(float(data.qpos[2]), 6))
print("OK - MuJoCo 物理引擎可正常运行")