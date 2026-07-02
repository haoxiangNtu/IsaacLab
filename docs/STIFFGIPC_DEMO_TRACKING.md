# StiffGIPC × IsaacLab-3.0 — Demo 移植跟踪

> 跟踪每个 IsaacLab 任务移植到 StiffGIPC（IPC 物理后端，`physics=stiffgipc` preset）的进度。
> **每做完一个 demo，就在这里记录：该 demo 触发/暴露的 bug + 为它做的引擎/solver 修改。**
> 引擎 = `/home/ps/Downloads/Stiff-GIPC-v065`（C++/CUDA，改完 `build_312` 重编）；
> solver = `/home/ps/Downloads/newton/newton/_src/solvers/stiffgipc/`（Python）。
> 运行：`conda activate il3` → `cd IsaacLab-3.0 && ./isaaclab.sh -p ... physics=stiffgipc`。

---

## 总览表

| 任务 | 类型 | 物理preset | 训练 | StiffGIPC 适配 | 移植成本 | 状态 |
|---|---|---|---|---|---|---|
| **cartpole** | 刚体控制 | ✅ | PPO | ✅ 已验证 | 完成 | ✅ 走通 |
| **ant** | 四足运动 | ✅ | PPO | ⚠️ 浮动基座难 | 进行中 | ✅ 能走（收敛fix后） |
| **shadow_hand** | 手内操控 | ✅ | PPO | ⭐最佳（固定基座+接触密集） | 低（加preset） | ✅ smoke-test 通过 |
| allegro_hand | 手内操控 | ✅ | PPO | ⭐ 最佳 | 低 | 待办 |
| shadow_hand_over | 双手传递 | ✅ | PPO | ⭐ 很好 | 低 | 待办 |
| humanoid | 人形运动 | ✅ | PPO | ❌ 比ant更难 | 高 | 待办 |
| franka_cabinet | 臂+柜+接触 | ❌ | PPO | ✅ 好 | 中 | 待办 |
| factory/forge/automate | 精密装配 | ❌ | PPO | ⭐ IPC最能体现但最难 | 高 | 待办 |
| **lift_franka_soft** | 软体抓取 | (manager) | PPO | ⭐ StiffGIPC原始目标 | 中 | 训练中（273+ ckpt）；**统一轮冒烟通过（2026-07-02）**：06me 引擎（v0.6.6+必要5项+per-vertex env隔离）4env×2iter 全绿 — env isolation 生效(2716v/4env, FEM=yes)、dhat ABSOLUTE(eff=9)、0触顶、0 NaN、reward 1.96→4.12 |
| cart_double_pendulum | 简单刚体 | ❌ | PPO | ✅ 容易 | 低 | 待办 |
| anymal_c | 四足运动 | ❌ | PPO | ❌ 运动难 | 高 | 待办 |
| quadcopter | 飞行 | ❌ | PPO | ❌ 无接触 | — | 不建议 |

**约定**：任务来自 **IsaacLab**（`isaaclab_tasks`，非 IsaacSim/Newton 自带）；训练统一 **PPO**（rsl_rl，非 TRPO）。

---

## cartpole — 刚体控制（✅ 已验证）

第一个走通的 StiffGIPC RL 任务。验证了 StiffGIPC→Newton solver 的基本闭环（关节驱动、状态读回、训练收敛）。

**preset**：`direct/cartpole/cartpole_env_cfg.py` 的 `stiffgipc`（effort/位控、单刚体杆+滑轨）。

**该 demo 暴露的 bug + 引擎修改：**

| bug | 根因 | 修改位置 |
|---|---|---|
| 铰接杆冻住不倒 | 重力体力缺旋转项：只给了 `[m·g; 0]`，漏了 `g⊗m_x̄` 积分项 → 关节力矩为零 | 引擎 `compute_trimesh_body_force`（完整积分；M 在梯度里抵消，effort 不受影响）。见记忆 `project_stiffgipc_gravity_torque_root_cause` |
| 软体冻住 / CUDA 越界 | OBJ `f v//vn` 双斜杠面未解析 → 未初始化面索引 | `load_mesh.cpp` 加 `f %d//%d` 分支，或去 normal。见 `stiff-physics-obj-vvn-parser-bug` |
| 多体 Kappa 串扰 | 全局单 Kappa | 见 `project_stiffgipc_integration_gotchas` |

---

## ant — 四足运动（✅ 能走，收敛 fix 后）

StiffGIPC 的**弱项**（浮动基座 + RL 探索需大量并行 env，而单场景 IPC 只能 ~32 env）。攻关很久，最终**靠引擎收敛阈值绝对化打通**。

**preset**：`direct/ant/ant_env_cfg.py` 的 `stiffgipc`
（`use_effort_control=True`, `skip_all_collision=False`, `newton_tol=0.2`, `absolute_dhat=1e-3`,
`friction_rate=3.0`, `revolute_driving_strength_ratio=0.0`(力控), `collision_mesh_segments=8`, `torso_skip_ground_collision=True`）。

**该 demo 暴露的 bug + 引擎/solver 修改（按发现顺序）：**

| # | bug（现象） | 根因 | 修改位置 |
|---|---|---|---|
| 1 | 躯干冻在 z=0.5 不动 | FREE 关节(浮动基座) joint_q 从不读回 → IL3 lazy eval_fk 把躯干 snap 回 spawn | solver `_readback_free_base`（用 eval_ik 从 body_q 同步 joint_q） |
| 2 | 角速度为零/错 | 浮动基座角速度没算 | solver `_sg_body_linvel` kernel（四元数有限差分） |
| 3 | 关节限位没生效 | 引擎只存不用 | 引擎隐式限位 penalty（revolute+prismatic）。见 `project_stiffgipc_joint_limit_penalty` |
| 4 | 髋拿到踝的限位 | 限位是 JOINT_DOF 索引；FREE 基座 7坐标≠6DOF 差 1 | solver 用 `lo[qd0]` 而非 `lo[q0]` |
| 5 | Python 显式限位惩罚把 Kappa 炸到 2.3e9 | 显式惩罚不稳 | 改用引擎隐式能量（默认关 Python 版） |
| 6 | **2-env 走、32-env（训练数量）站** | **收敛阈值 `distToOpt < newton_tol·√(bboxDiagSize2·dt²)` 用真实全场景 bbox → env 多→阈值松→求解糙→走不出** | **引擎收敛阈值绝对化**：新增 `eff_bboxDiagSize2` 成员，收敛判据(GIPC.cu:12415)改用它。见 `project_stiffgipc_convergence_threshold_env_independent` ⭐ |
| 7 | reward~410 看似高其实在站 | alive+up+heading(+1.1/步) > 站立惩罚(−0.5/步) → 站着净赚 | 任务级（非引擎）：站立惩罚整形 + entropy=0.01；但**真正解药是 #6 的引擎修复，不是奖励** |

**关键教训**：①测位移**必须用训练 env 数**，2-env 不代表（旧引擎下 2走/32站）；②"32-env 站"是引擎 env 依赖 bug，不是训练失败 —— 修复后同一 model_199 无需重训直接走（32只里 23 只走）。

**质量/力（排除过的非因素）**：ant 总质量 ~0.9kg（=USD/PhysX 质量），关节力矩 ±7.5 N·m/关节（action_scale 0.5×joint_gears 15）。

---

## shadow_hand — 手内操控（preset 已备，待 smoke-test）

StiffGIPC 的**强项主场**：固定基座 + 接触密集（IPC 价值所在）。**位置控制**（`set_joint_position_target`，区别于 ant/cartpole 的力控）。

**preset**：`direct/shadow_hand/shadow_hand_env_cfg.py` 的 `stiffgipc`
（`use_effort_control=False`, `revolute_driving_strength_ratio=1000`(驱动到位置目标), `skip_all_collision=False`, `collision_mesh_segments=12`）。
另外给 scene/robot/object/event 的 PresetCfg 加了 `stiffgipc = newton_mjwarp` 别名（StiffGIPC 是 Newton 后端，需用 Newton 变体的 USD/actuator，而非 physx default）。

**smoke-test 通过（2026-06-30）**：`Isaac-Repose-Cube-Shadow-Direct-v0 physics=stiffgipc --num_envs 2 --max_iterations 1`（il3 kitless，纯刚体）→ 完整跑通 iteration 0：env 建立、StiffGIPC 步进（**Newton 迭代 ~10-30，收敛良好不顶满**）、rollout、PPO 更新、Mean reward −6.94（未训练正常）、episode length 8。preset（physics=stiffgipc + scene/robot/object/event 的 newton_mjwarp 别名）解析正确，无报错。

**正式训练后的完整发现（2026-07-01）：**

任务 = repose/reorient：把手里 cube 转到目标朝向（浮空 cube = `goal_marker` 目标标记，非 bug），转对 +250（`reach_goal_bonus`）。原版 **8192 env** 训练（`num_envs=8192`, `num_steps_per_env=16`）。

**该 demo 暴露的 bug + 引擎/solver 修复（按发现顺序）：**

| # | bug（现象） | 根因 | 修复 |
|---|---|---|---|
| 1 | 未加 slew cap → 手扭曲/关节崩 | preset 漏了 `max_revolute_step_per_frame`（默认 1.0=lockup 值）| 加 `max_revolute_step_per_frame=0.1`（见 [[large_rotation_slew_cap]]）|
| 2 | **手指整条从手掌脱落** | Shadow Hand 有**无碰撞网格的指根 knuckle 中间 link** → StiffGIPC 的 ABD body 就是网格 → 无网格 body 被跳过 → 24 关节只建了 16 个约束 → 链断 | ⭐**mesh-less link → collision-disabled ABD proxy**（solver：给无网格 link 建极小 box ABD body + 排除其碰撞）。约束 16→24，脱离 516mm→127mm。见 [[project_stiffgipc_meshless_link_proxy]] |
| 3 | 附着弱（曾疑）| kappa=sr×质量，手指克级→kappa~5 | `joint_strength_ratio` 1000→1e5（但#2才是脱落主因）|
| 4 | fixed tendon 未处理 | Shadow Hand 远端 2 关节靠固定肌腱耦合，StiffGIPC solver+引擎都不支持 tendon | **未解决**（远端关节失耦/软趴，非脱落）。作为后续 engine 增强单独立项 |

**碰撞/速度优化：** `collision_mesh_segments` 12→6（顶点 134→~50/link）、`disable_articulation_ground_collision=True`（臂↔地全剔）、solver 按 bbox 尺寸**排除前臂**（最大 body，每 env 1 个）。

**关键定量结论（架构性，改不了）：**
- **引擎单步 ~21ms（16env），瓶颈不是它** —— headless 也只 1.46 帧/s，~660ms/帧是 **IPC 求解本身 GPU-bound**（全局 Newton+PCG+CCD），不是 Python 读回/foldshirt/渲染。
- **吞吐量恒定**：16env=23.4 / 64env=23.0 env-steps/s。**单场景全局求解 → env×4→每步×4慢→吞吐持平**（对比 PhysX replicated env×4→吞吐×4）。ABD 每 body 12-DOF，与顶点无关 → 简化网格只省碰撞/CCD，不改吞吐。
- **S4 per-env mask**：opt-in `STIFFGIPC_S4_MASK`，16env 下能启用不 OOM，但**没提速**（precursor 半成品，实际没剔除已收敛 env 的算力；全局收敛仍按最难 env）。
- **RL 学不出**：reward iter0 −4 → iter8 **−59（下降）**，探索严重不足（64 env × 0.3fps 差 8192 env 几个数量级）。**和 ant 同病：单场景 IPC → env 天花板 → 探索型 RL 先天吃亏。**

**结论**：脱落已修（proxy），手在 StiffGIPC 里能连着抓；但 **reorient 策略训不出是架构性的**（env 数不够）。StiffGIPC 的价值在**接触/软体精度**，不在大规模探索 RL。

---

## lift_franka_soft — 软体抓取（StiffGIPC 原始目标）

Franka Panda 臂抓举一个软长条（0.3×0.05×0.05，young=5e5 软橡胶，黄色）。StiffGIPC 统一处理刚性臂(ABD)+软块(FEM)+无穿透接触。
路径：`manager_based/manipulation/lift_franka_soft/`。

**当前测试目标：机械臂大角度旋转时的"压扁"问题**（ABD 软关节 + 软正交势的刚度失衡 → link 形变）。

诊断开关（测 reset/teleport 用，非正常训练）：
- `STIFFGIPC_FREEZE_ARM=1`：机械臂锁死不动
- `STIFFGIPC_FORCE_BLOCK_DEMO=1`：每 2s 强制把软块 teleport 抬高+左右交替再掉落
- `REPLAY_NO_RESET=1`：禁 env reset（不卡顿）
- checkpoint：`logs/rsl_rl/franka_deformable/2026-06-29_01-50-46/model_149.pt`（会 engage/压块的策略）

**大角度旋转/压扁问题（理论背景）**：ABD 用软正交势 V_⊥=κ‖AᵀA−I‖²（不硬约束刚性）+ 软关节 penalty（K_joint=sr·(m_p+m_c)·dt²，质量加权）。
- ⚠️ 纠正一个常见误解：正交势**不惩罚旋转**（V_⊥ 对刚性旋转 A=R·A₀ 恒定=0），大角度旋转本身免费。
- 压扁真正机理 = **刚度失衡**：当关节/接触 penalty 局部比正交势硬时，优化器宁可把 A 压歪（剪切/压扁）也不刚性满足约束。sr 大(1000)+质量大 → K_joint 可能超过正交 κ → 压扁。
- 修复梯度（轻→重）：**A. 刚度配平**（κ_正交≫K_joint，调 Young's / 给 sr 封顶，推荐先试）→ **B. 硬关节等式约束**（论文 Sec5.3）→ **C. 真 SE(3) 刚体+curved CCD**（否定 ABD 初衷，不建议）。

**已测结论（2026-06-30）**：

1. **单 link 不压扁**：策略正常操作 + slew cap=0.1 下，每个臂 link 的形状（协方差主轴 extent，旋转/平移不变）偏离初始 **0.0–1.3%**（刚体）。把 cap 抬到 1.0 + 大角度旋转后，单 link 仍 **0.0%** —— dump 用 `engine.get_vertices()`（真实 affine 顶点，非刚性化），所以这是真的：**link 本身不压扁**。
2. **变形在关节，不在 link**：cap=1.0 + 随机 action 探索时，相邻 link 质心距离变 **22%**、整臂 bbox 涨大 → **关节处脱开/整臂扭曲**。15 帧即 **permanent lockup**（Newton 顶满）。
3. **放开迭代不解决（根本性不收敛）**：`newton_iter_cap` 150→**2000（13×）**，每步 Newton 仍顶到 **~1990** 不收敛。**不是 cap 太低，是线性化失效。**

**根因（与 SE(3)/ABD 理论闭环）**：单帧大转角 → ①关节 penalty `0.5K·sin²(θ−θ_tgt)` 在 >90° 处梯度指反方向（非凸）；②ABD 在 12-DOF 仿射空间走**直线 Newton 步**，对 SE(3) 曲流形上的大旋转是错模型 → line search 反复拒绝 → 迭代再多也收不敛。PCG（内层）解每个 SPD 子问题没问题，但外层 Newton 方向本身错。

**解法（已用）**：`max_revolute_step_per_frame=0.1`（每帧小 Δθ → 落在仿射线性化有效区 → ~17 迭代收敛）。这是物理正解，不是权宜。备选：更强关节 κ（治标）、硬关节约束、真 SE(3) 刚体+curved CCD（推翻 ABD，不建议）。

**复现命令**（临时把 `max_revolute_step_per_frame` 抬到 1.0）：
```
STIFFGIPC_ROOT=... STIFFGIPC_DUMP=1 STIFFGIPC_DUMP_DIR=... \
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Lift-Soft-Franka-v0 physics=stiffgipc --num_envs 1 --headless
```
诊断开关 `STIFFGIPC_ROTATE_TEST="ord:offset"`（solver，驱动第 ord 个 revolute 关节转到 init+offset）用于受控单关节大角度测试。
