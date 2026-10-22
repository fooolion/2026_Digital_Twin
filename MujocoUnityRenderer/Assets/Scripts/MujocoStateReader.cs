using System.IO;
using UnityEngine;

/// <summary>
/// 读取 sim_bridge.py 写出的 latest_state.json，驱动 Arm9.17 模型（纯渲染，不用 Unity 物理）。
///
/// 为什么删掉 ArticulationBody：
///   队友给模型配的 AB 在 Unity 里会自己跑物理，这套 CAD 模型（多连杆 + 网格碰撞 + 大质量比）
///   的 AB 求解器不稳定，表现为**模型高频抽搐**（把 sim_bridge 停掉也在抖 = 与数据无关）。
///   但我们架构上本来就是「MuJoCo 算物理、Unity 只渲染」，Unity 根本不需要物理。
///   ⇒ 启动时删掉模型上所有 ArticulationBody + 关掉所有 Collider，然后直接改各关节 Transform。
///
/// 用法（Arm9.17 预制体已默认挂好本组件，拖进场景即可）：
///   1. 把 Arm9.17 拖进场景（场景里只留这一个模型）；
///   2. 本组件已挂在 Arm9.17 根节点；joints 留空会自动找 Joint_1~6；
///   3. 先跑 sim_bridge.py，再进 Play。
///
/// 校准（已按实测定稿，若个别关节不对仍可在 Inspector 单独改）：
///   - jointAxes[i]      ：该关节绕自身哪个本地轴转。已校准 = [Y, Y, Y, Y, Z, X]
///   - angleSigns[i]     ：已校准 = [-1, 1, -1, -1, 1, 1]
///   - angleOffsetsDeg[i]：模型零位与 MuJoCo 零位不一致时填固定偏差(度)，当前全 0
/// </summary>
public class MujocoStateReader : MonoBehaviour
{
    [Header("关节（留空自动找 Joint_1~Joint_6）")]
    [SerializeField] private Transform[] joints;
    [Header("夹爪齿轮（留空自动找 Joint_7Gear）")]
    [SerializeField] private Transform gripperGear;
    [Header("夹爪齿条（留空自动找 Joint_7gripperL / Joint_7gripperR）")]
    [SerializeField] private Transform rackLeft;
    [SerializeField] private Transform rackRight;

    [Header("校准：每个关节绕哪个本地轴转（已定稿 J1~J4=Y、J5=Z、J6=X）")]
    [SerializeField] private Vector3[] jointAxes;
    [Tooltip("每个关节符号：MuJoCo 正角与关节正转一致 +1，相反 -1")]
    [SerializeField] private float[] angleSigns;
    [Tooltip("每个关节零位补偿(度)")]
    [SerializeField] private float[] angleOffsetsDeg;

    [Header("夹爪参数")]
    [Tooltip("齿轮节圆半径(米)，与 MuJoCo/sim_bridge 一致")]
    [SerializeField] private float gearRadius = 0.008f;
    [Tooltip("齿轮绕哪个本地轴转（纯视觉，指头由齿条驱动）")]
    [SerializeField] private Vector3 gearAxis = new Vector3(1f, 0f, 0f);
    [Tooltip("两指分开的方向（本地轴）：实测为本地 Y（两指在 Y=∓0.014）。开合反了就把符号取反")]
    [SerializeField] private Vector3 rackAxis = new Vector3(0f, 1f, 0f);

    [Header("数据文件")]
    [SerializeField] private string stateFilePath =
        "D:/Study/Zju/Competition/2026_Digital_Twin/mujoco_sim/output/latest_state.json";
    [SerializeField] private float pollInterval = 0.02f;

    // 力矩反馈（J1~J6，N·m，有符号），供 UI 读取
    public float[] MotorTorque { get; private set; }
    public float[] MotorFriction { get; private set; }
    public float[] MotorLoad { get; private set; }

    private Quaternion[] restRot = new Quaternion[6];
    private Quaternion gearRestRot;
    private Vector3 rackLRestPos, rackRRestPos;
    private bool ready;

    [System.Serializable]
    private class State
    {
        public float t;
        public float[] joints_deg;
        public float gripper_m;
        public float[] motor_torque;
        public float[] motor_friction;
        public float[] motor_load;
    }

    private void Start()
    {
        // 数组字段为空就填默认值（已实测校准：J1~J4 绕本地 Y、J5 绕本地 Z、J6 绕本地 X）
        if (jointAxes == null || jointAxes.Length < 6)
        {
            jointAxes = new Vector3[]
            {
                Vector3.up, Vector3.up, Vector3.up, Vector3.up,   // J1~J4 绕本地 Y
                Vector3.forward,                                   // J5 绕本地 Z
                Vector3.right,                                     // J6 绕本地 X
            };
        }
        if (angleSigns == null || angleSigns.Length < 6)
            angleSigns = new float[] { -1f, 1f, -1f, -1f, 1f, 1f };   // 实测方向
        if (angleOffsetsDeg == null || angleOffsetsDeg.Length < 6)
            angleOffsetsDeg = new float[6];

        if (joints == null || joints.Length < 6 || joints[0] == null)
            joints = FindByName("Joint_", 1, 6);
        if (joints == null || joints.Length < 6 || joints[0] == null)
        {
            Debug.LogError("MujocoStateReader: 找不到 Joint_1~Joint_6。请确认场景里有 Arm9.17 模型，或手动拖进 joints 数组。");
            return;
        }

        if (gripperGear == null) gripperGear = FindOne("Joint_7Gear");
        if (rackLeft == null) rackLeft = FindOne("Joint_7gripperL");
        if (rackRight == null) rackRight = FindOne("Joint_7gripperR");

        // ===== 关键：删掉模型上所有 ArticulationBody + 关掉所有 Collider =====
        // Unity 只渲染、MuJoCo 管物理；不删的话 AB 求解器会自己抖（高频抽搐）。
        Transform root = joints[0].root;
        ArticulationBody[] abs = root.GetComponentsInChildren<ArticulationBody>(true);
        foreach (ArticulationBody ab in abs) Destroy(ab);
        int nCol = 0;
        foreach (Collider col in root.GetComponentsInChildren<Collider>(true)) { col.enabled = false; nCol++; }
        Debug.Log($"MujocoStateReader: 已移除 {abs.Length} 个 ArticulationBody、关闭 {nCol} 个 Collider（Unity 不再跑物理）。" +
                  $"夹爪: 齿轮={(gripperGear != null)}, 左齿条={(rackLeft != null)}, 右齿条={(rackRight != null)}");

        // 几何对表用：打印每个关节的世界坐标（与 MuJoCo/URDF 的关节锚点对照）
        for (int i = 0; i < 6; i++)
        {
            if (joints[i] != null)
                Debug.Log($"[几何对表] Joint_{i + 1} 世界坐标 = ({joints[i].position.x:F4}, {joints[i].position.y:F4}, {joints[i].position.z:F4}) m");
        }
        if (gripperGear != null)
            Debug.Log($"[几何对表] Joint_7Gear 世界坐标 = ({gripperGear.position.x:F4}, {gripperGear.position.y:F4}, {gripperGear.position.z:F4}) m");

        // 记录静止姿态
        for (int i = 0; i < 6; i++) restRot[i] = joints[i].localRotation;
        if (gripperGear != null) gearRestRot = gripperGear.localRotation;
        if (rackLeft != null) rackLRestPos = rackLeft.localPosition;
        if (rackRight != null) rackRRestPos = rackRight.localPosition;

        ready = true;
    }

    private float _timer;
    private void FixedUpdate()
    {
        if (!ready) return;
        _timer += Time.fixedDeltaTime;
        if (_timer < pollInterval) return;
        _timer = 0f;
        TryApply();
    }

    private void TryApply()
    {
        if (!File.Exists(stateFilePath)) return;

        string json;
        try { json = File.ReadAllText(stateFilePath); }
        catch { return; }

        if (string.IsNullOrWhiteSpace(json)) return;

        State s;
        try { s = JsonUtility.FromJson<State>(json); }
        catch { return; }

        if (s == null || s.joints_deg == null || s.joints_deg.Length < 6) return;

        MotorTorque = s.motor_torque;
        MotorFriction = s.motor_friction;
        MotorLoad = s.motor_load;

        // 6 个臂关节：绕各自本地轴转（RestRot * 绕轴旋转）
        for (int i = 0; i < 6; i++)
        {
            if (joints[i] == null) continue;
            Vector3 axis = (jointAxes.Length > i && jointAxes[i].sqrMagnitude > 1e-8f) ? jointAxes[i] : Vector3.right;
            float sign = angleSigns.Length > i ? angleSigns[i] : 1f;
            float offset = angleOffsetsDeg.Length > i ? angleOffsetsDeg[i] : 0f;
            float deg = s.joints_deg[i] * sign + offset;
            joints[i].localRotation = restRot[i] * Quaternion.AngleAxis(deg, axis.normalized);
        }

        // 夹爪：开度(米) -> 齿轮角(rad)；两齿条沿 rackAxis 反向平动（实测是本地 Y）
        float opening = Mathf.Clamp(s.gripper_m, 0f, 0.05f);
        float gearAngle = opening / gearRadius;
        if (gripperGear != null)
        {
            Vector3 gAxis = gearAxis.sqrMagnitude > 1e-8f ? gearAxis.normalized : Vector3.right;
            gripperGear.localRotation = gearRestRot * Quaternion.AngleAxis(gearAngle, gAxis);
        }
        Vector3 ra = rackAxis.sqrMagnitude > 1e-8f ? rackAxis.normalized : Vector3.up;
        float delta = gearAngle * gearRadius;      // 每个指头平移量(米) = 开度
        if (rackLeft != null) rackLeft.localPosition = rackLRestPos - ra * delta;
        if (rackRight != null) rackRight.localPosition = rackRRestPos + ra * delta;
    }

    // 按名字批量找（Joint_1..Joint_6），找不到返回带 null 的数组
    static Transform[] FindByName(string prefix, int from, int to)
    {
        Transform[] result = new Transform[to - from + 1];
        for (int i = from; i <= to; i++)
            result[i - from] = FindOne(prefix + i);
        return result;
    }

    static Transform FindOne(string name)
    {
        GameObject go = GameObject.Find(name);
        if (go == null) { Debug.LogWarning("MujocoStateReader: 找不到 " + name); return null; }
        return go.transform;
    }
}