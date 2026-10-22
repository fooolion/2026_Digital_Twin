using System.IO;
using UnityEngine;

/// <summary>
/// 把 Unity 里的「拖拽目标点 + 夹爪开度」写成 target.json，喂给 sim_bridge.py（MuJoCo 侧）。
///
/// 用法（Arm9.17 预制体已默认挂好本组件）：
///   1. 进 Play 后，Hierarchy 里选中绿色小球「MJ_Target」，用移动工具（W）在 Scene 里拖它；
///   2. 机械臂会实时用 IK 追这个球（MuJoCo 侧算）；
///   3. 夹爪开度在 Inspector 的「Gripper Opening」滑条上拖（0=闭合，0.05=全开）；
///   4. 把「Send Target」取消勾选（或退出 Play）→ MuJoCo 回到演示轨迹。
///
/// 坐标说明：
///   绿球挂在模型的 Base 节点下，**它的局部坐标 = MuJoCo 世界坐标**（Base 局部系 = CAD Z 上系，
///   与 MuJoCo 一致）。脚本用 InverseTransformPoint 自动换算，模型在场景里怎么摆都不影响。
///
/// VR 接入：以后把 PICO 手柄的 Transform 拖进 Target Point，VR 里挥手柄 = 拖目标。
/// </summary>
public class MujocoTargetWriter : MonoBehaviour
{
    [Header("拖拽目标（留空会在模型坐标系里自动生成一个绿球）")]
    [Tooltip("以后接 VR 时，把手柄的 Transform 拖到这里即可")]
    [SerializeField] private Transform targetPoint;

    [Header("坐标参考系（模型 Base 节点，留空自动找）")]
    [SerializeField] private Transform frameOfReference;
    [Header("轴向符号修正：MuJoCo 坐标 = Base局部坐标 × axisSigns")]
    [Tooltip("实测队友 CAD 的 X 轴与 MuJoCo/URDF 相反，故默认 X=-1；哪个轴跟手方向反了就取反哪个")]
    [SerializeField] private Vector3 axisSigns = new Vector3(-1f, 1f, 1f);

    [Header("夹爪开度(米)：0=闭合 0.05=全开")]
    [Range(0f, 0.05f)]
    [SerializeField] private float gripperOpening = 0.02f;

    [Header("输出")]
    [SerializeField] private string targetFilePath =
        "D:/Study/Zju/Competition/2026_Digital_Twin/mujoco_sim/output/target.json";
    [Tooltip("是否下发目标；关掉后 MuJoCo 走演示轨迹")]
    [SerializeField] private bool sendTarget = true;
    [Tooltip("最小写盘间隔(秒)")]
    [SerializeField] private float minSendInterval = 0.05f;
    [Tooltip("目标挪动超过这个距离(米)才写盘")]
    [SerializeField] private float moveThreshold = 0.002f;

    private Vector3 lastSent;
    private float lastGrip = -1f;
    private float timer;
    private bool everSent;

    private void Start()
    {
        if (frameOfReference == null)
        {
            // 模型的 Base 节点：其局部系 = CAD Z 上系 = MuJoCo 世界系
            foreach (Transform t in transform.root.GetComponentsInChildren<Transform>(true))
            {
                if (t.name == "Base") { frameOfReference = t; break; }
            }
        }
        if (frameOfReference == null)
        {
            Debug.LogError("MujocoTargetWriter: 找不到模型 Base 节点，无法换算坐标。");
            enabled = false;
            return;
        }

        // 自动生成拖拽目标小球（挂在 Base 下，局部坐标即 MuJoCo 坐标）
        if (targetPoint == null)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            Destroy(go.GetComponent<Collider>());
            go.name = "MJ_Target";
            targetPoint = go.transform;
            targetPoint.SetParent(frameOfReference, false);
            // 初始位置：经 axisSigns(X=-1) 换算后 = MuJoCo (0.35, 0, 0.20)，实测 IK 误差 0.02mm 可达
            targetPoint.localPosition = new Vector3(-0.35f, 0f, 0.20f);
            targetPoint.localScale = Vector3.one * 0.04f;
            Renderer r = go.GetComponent<Renderer>();
            if (r != null) r.material.color = new Color(0.2f, 1f, 0.35f);
        }

        lastSent = frameOfReference.InverseTransformPoint(targetPoint.position);
        Debug.Log($"MujocoTargetWriter: 就绪。Play 时拖绿球「MJ_Target」即可让机械臂跟随（IK 在 MuJoCo 侧算）。");
    }

    private void Update()
    {
        if (!sendTarget || targetPoint == null || frameOfReference == null) return;

        timer += Time.deltaTime;
        if (timer < minSendInterval) return;

        Vector3 mj = frameOfReference.InverseTransformPoint(targetPoint.position);
        mj = Vector3.Scale(mj, axisSigns);   // 轴向符号修正（队友 CAD 与 MuJoCo 的 X 轴相反）
        bool moved = (mj - lastSent).magnitude > moveThreshold;
        bool gripChanged = Mathf.Abs(gripperOpening - lastGrip) > 0.0005f;
        if (!moved && !gripChanged) return;

        timer = 0f;
        Write(true, mj, gripperOpening);
        lastSent = mj;
        lastGrip = gripperOpening;
    }

    // 写 target.json（active=false 时 MuJoCo 回到演示轨迹）
    private void Write(bool active, Vector3 mj, float grip)
    {
        string json = active
            ? "{\"active\": true, \"x\": " + mj.x.ToString("F4", System.Globalization.CultureInfo.InvariantCulture)
              + ", \"y\": " + mj.y.ToString("F4", System.Globalization.CultureInfo.InvariantCulture)
              + ", \"z\": " + mj.z.ToString("F4", System.Globalization.CultureInfo.InvariantCulture)
              + ", \"gripper\": " + grip.ToString("F4", System.Globalization.CultureInfo.InvariantCulture) + "}"
            : "{\"active\": false}";
        try { File.WriteAllText(targetFilePath, json); everSent = true; }
        catch (System.Exception e) { Debug.LogWarning("MujocoTargetWriter: 写 target.json 失败 " + e.Message); }
    }

    private void OnDisable() { if (everSent) Write(false, Vector3.zero, 0f); }
    private void OnApplicationQuit() { if (everSent) Write(false, Vector3.zero, 0f); }
}