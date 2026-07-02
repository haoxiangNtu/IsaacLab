# StiffGIPC 统一更新计划(v0.6.5 改动 → 统一到 v0.6.6)

---
## ★ 最终裁决(v0.6.6 基线,tag 内容级核验 + 讨论确认)★
> 结论:v0.6.5 的引擎改动**绝大部分已在 v0.6.6 里**。真正要加到 v0.6.6 的引擎改动只有 **5 项**;其余是冗余(已有)或诊断(删)。

### 引擎(Stiff-GIPC)—— 必要集(5 项)
| 改动 | v0.6.6 现状 | 裁决 | 状态 |
|---|---|---|---|
| ① 收敛绝对化(gradVanish→eff_bboxDiagSize2)| 用原始 bbox | 🔧 **必要** | 已改 build_312 |
| ④ 关节限位 penalty(单边弹簧,**非 barrier**)| 无 | 🔧 **必要** | 已改+调优(ratio=20000,限位0.5→挡在0.524)|
| ⑦ ground d=0 → 条件式(`dist2==0.0?1e-12:dist2`)| 有但是 clamp(会冻死)| 🔧 **必要** | 已改 build_312 |
| teleport fem_offset(块 reset)| 未提交 | 🔧 **必要** | 已在工作区 |
| **absolute_dhat 壳透传(engine.py)** | tag 里 0 次 → absolute_dhat 死 | 🔧 **必要**(不透传则 ①② 失效;solver 经 engine.py `__init__` 传)| 已在工作区 |

### 引擎 —— 不必要(v0.6.6 已具备,**不重复移植**)
dhat 绝对化(eff 引擎逻辑)、重力体力旋转项(compute_trimesh_body_force segment 3/6/9)、slew cap、collision API(exclusion/groups/ground-skip)、per-body 密度/惯量 setter、对应 bindings —— 均已在 v0.6.6 tag(内容级确认)。

### 引擎 —— 删/不进
`dump_state_raw`(诊断,删)、S4 per-env mask(半成品,不进)。

### Newton solver —— 裁决
| 改动 | 裁决 |
|---|---|
| Kabsch/浮动基座读回、qd0 修复、_add_articulation_self_collision_exclusions、_drive_joints、_apply_resets、converter segments、mesh-less proxy | ✅ **必要** |
| **跨env排除(_apply_env_collision_groups)** | ✅ **必要:默认排除**(不是条件性)。ABD 已用 per-body-pair add_collision_exclusion 排除;**FEM 见下方专项方案** |
| **gripper-only / 前臂排除** | 🔧 **重构:移出 solver → 到任务 cfg**;solver 只留通用 add_collision_exclusion 管道 |
| NO_ENV_GROUPS | 🗑️ **删**(禁用跨env排除的调试开关,既然默认要排除就不需要)|
| STIFFGIPC_* 诊断 env-gate(FREEZE_ARM/FORCE_BLOCK_DEMO/LAND_DUMP/STATE_DUMP/RESET_*/ROTATE_TEST/DIAG/DUMP)| 🗑️ **删** |
| PY_JOINT_LIMITS | 🗑️ **退役完成**(env-gate 已删;④ 上引擎后遗留的死代码 `_rev_limits/_pris_limits`+`STIFFGIPC_LIMIT_KE/KD`+`lim_ke/lim_kd` **只写不读**,已一并清除,零风险 no-op)|
| S4 mask | 🗑️ **不进(半成品)** |

### IsaacLab —— 裁决(**均采用现有方案**)
lift 场景+rewards+mdp、stiffgipc_manager_cfg 字段、PPO 超参、cartpole/ant/shadow preset ✅ 必要;ant/shadow 实验参数定稿后同步;文档可选。**新增:gripper-only 排除规格从 solver 移到此处(任务 cfg 指定排除对)**。

---
### ★ 实现状态核验(代码级,全部完成)★
| 类别 | 结论 |
|---|---|
| 引擎必要 5 项 | ✅ 全部在 06me + 已重建 build_312;backend 6/6 通过 |
| 跨env排除 | ✅ **默认排除**,per-vertex env 方案(见下方 FEM subscene 小节);引擎+solver+2 个自断言测试全 PASS |
| gripper-only | ✅ 已移任务层(`object_collision_link_filter`)|
| NO_ENV_GROUPS / S4 / 诊断 env-gate / PY_JOINT_LIMITS(含死码)| ✅ 全部清零(grep=0)|
| absolute_dhat 壳透传 | ✅ 必要且在位(桥:tag 的 engine.py 0 次 → 不透传则引擎 eff 永远休眠)|
| IsaacLab 任务本体 | ✅ **lift 冒烟通过(2026-07-02)**:`physics=stiffgipc --num_envs 4 --max_iterations 2` 全跑通 —— per-vertex env isolation 生效(2716 verts/4 envs, FEM=yes)、dhat ABSOLUTE(eff=9)、0 触顶、0 NaN、reward 1.96→4.12。shadow 冒烟此前已过(见 DEMO_TRACKING)。前置:newton editable 加了 5 处 warp<1.14 兼容守卫(graph_coloring + 3 sensors + selection,isaacsim6/il3 是 warp 1.13)|

> 部署提醒:newton solver 默认 `STIFFGIPC_ROOT=".../Stiff-GIPC-v065"`(旧 checkout,无 `set_vertex_env_ids`)。IsaacLab 正式用到本轮改动前需**从 06me 重新发布/安装 wheel**,或运行时设 `STIFFGIPC_ROOT=/home/ps/Downloads/Stiff-GIPC-06me`。

---
### ★ 发布流程实测(2026-07-02,按 STIFF_PHYSICS_RELEASE_HANDBOOK 流程)★
| 步骤 | 状态 |
|---|---|
| 版本准备 | ✅ pyproject 0.6.6→**0.6.7**;CHANGELOG 起草 [0.6.7] 条目(5 必要项 + per-vertex env 隔离 + OBJ 解析修复)+ 补记缺失的 [0.6.6] 条目 |
| cp312 wheel 构建 | ✅ `stiff_physics-0.6.7-cp312-cp312-linux_x86_64.whl`(30MB,dist_rc/,scikit-build-core 全量编译)|
| cp312 fresh-venv 安装验证 | ✅ 干净 venv 离线安装 → **安装态 gate PASS**(确认 site-packages 解析、collide 严格无穿 dy=0.100/pen=-2e-4、isolate 穿过 dy≈0)|
| cp311 wheel 构建(env_isaaclab 目标)| ✅ `stiff_physics-0.6.7-cp311-cp311-linux_x86_64.whl`(30MB)+ fresh py3.11 venv 安装态 gate **PASS**(与 cp312 数值一致)|
| lift 冒烟(IsaacLab 端)| ✅(见上)|

> dist_rc/ 是**工作树构建的过程测试产物**。正式发布仍须按 handbook §4.2:用户 commit → tag v0.6.7 → 从 tag worktree 重建 wheel → 公开仓 gh release(作者必须 haoxiang002)→ handbook §5.1 公开仓 examples 全量 fresh-install 测试。
> 附带:newton editable 加了 5 处 warp<1.14 兼容守卫(graph_coloring + sensor_frame_transform/imu/contact + utils/selection),isaacsim6/il3 的 warp 1.13 也能跑 dev newton。

#### 用户 Runbook(commit/tag/发布 —— 由用户执行)
**① 引擎仓 06me commit + tag**(作者必须 haoxiang002,先 `git config user.name/user.email` 核对):
- 待提交(本轮,12 文件 + 3 新增):`GIPC.cu/.cuh`、`abd_driving_joint.h`、`setup_abd_system_gradient_and_hessian.cu`、`abd_system_parms.h`、`mlbvh.cu/.cuh`、`sim_engine.cu/.h`、`bindings/pystiffgipc.cu`、`stiff_physics/engine.py`、`pyproject.toml`(0.6.7)、`CHANGELOG.md`、新增 `examples/test_env_isolation.py`
- ⚠️ 工作区还有**非本轮**的未跟踪文件(`demo_dataset_softbody.py`、`tools_stl2msh.py`、`Assets/sorted_mesh/*` 等)——是否入库你来定,别混进 0.6.7 commit
```bash
cd /home/ps/Downloads/Stiff-GIPC-06me
git add <上面清单>  && git commit -m "release: v0.6.7 (unification)"  # 按需拆多个 commit
git tag -a v0.6.7 -m "Release v0.6.7" && git push origin <branch> v0.6.7
# 从 tag 干净重建正式 wheel(worktree 隔离):
git worktree add /tmp/build_v0.6.7 v0.6.7 && cd /tmp/build_v0.6.7
/home/ps/Downloads/Stiff-GIPC/.venv/bin/python -m pip wheel . --no-build-isolation -w /home/ps/Downloads/Stiff-GIPC-06me/dist/   # cp312
/home/ps/miniconda3/envs/env_isaaclab/bin/python -m pip wheel . --no-build-isolation -w /home/ps/Downloads/Stiff-GIPC-06me/dist/ # cp311
cd - && git worktree remove /tmp/build_v0.6.7
```
**② 公开仓发布**(handbook §4.2/§5.1;release notes 必须带私有 tag+commit hash;发布前公开仓 examples 全量 fresh-install 测试)
**③ newton 仓 commit**:`solver_stiffgipc.py`、`converter.py`、`graph_coloring.py`、`sensors/sensor_{frame_transform,imu,contact}.py`、`utils/selection.py`、`geometry/types.py`(此前已改)+ 新增 `scripts/test_stiffgipc_env_isolation.py`
**④ IsaacLab-3.0 commit**:16 个 M 文件(manager cfg、lift 任务、cartpole/ant/shadow preset、kit apps、docs)
**⑤ 部署**:`env_isaaclab` / il3 安装新 wheel 后,newton solver 的 `STIFFGIPC_ROOT` 默认路径(指 v065)即可退役——wheel 安装模式优先于 dev 路径。

### ★ FEM 多env 隔离专项方案(采用 UIPC SubsceneTabular 式,替换旧"方案A:per-env FEM 分体")★
**问题**:StiffGIPC 的 FEM 是"一个合并 collision body"(所有 env 共用一个 fem body id),per-body-pair 的 `add_collision_exclusion` 无法区分 env;现状退回空间分隔(env_spacing=2.5m),不鲁棒。用户要求:**FEM 也要按 env 排除,不靠空间分隔**。

**对比参考(rbs-uipc)**:UIPC 用 `SubsceneTabular` —— 每几何体贴 subscene 标签、O(env²) enable 矩阵(env内开/跨env关/env↔ground开),**FEM+ABD 统一、不靠空间分隔**。参考实现:`rbs-uipc-cmp-public/apps/tests/sim_case/42_abd_fem_subscene.cpp`(ABD+FEM 子场景隔离带断言测试)。

**决定:StiffGIPC 引入类 SubsceneTabular 的"per-env 接触组"机制(优于旧方案A):**
- 给**每个几何体(FEM 粒子 + ABD body)贴 per-env 组标签**
- O(env²) enable 矩阵:**env内碰撞开、跨env关、env↔ground 开**
- **只用于接触排除**(broad/narrow phase 跳过跨env对),**与 block-diagonal 求解解耦**(避免现有 `set_body_groups` 的 OOM)
- 一次解决 FEM+ABD 全部跨env隔离,不靠空间分隔,O(env²)

**为什么弃用旧方案A(per-env FEM 分体加载)**:方案A 只解决 FEM、且要连带改 FEM 读回/reset(_apply_resets/teleport/Kabsch 的"合并块"假设),风险大;subscene 式一次覆盖 FEM+ABD 且与 UIPC 对齐。

**附:④ 关节限位设计已被 UIPC 佐证** —— UIPC `AffineBodyRevoluteJointLimit.apply_to(joint, lower, upper, strength)` 与我们移植的 ④ penalty(lower/upper/limit_stiffness)同设计,证明用 penalty(非 barrier)正确。

**排期**:该 subscene 式机制是中等引擎工程(per-element 标签 + O(env²) 排除矩阵 + 解耦求解),放在 solver 清理轮**之后**的一个专门小轮做 + 用多env lift reset 测试验证(块 per-env 正确 reset、跨env 不互碰、不靠间距)。

---

#### ✅ 已实现 + 验证(FEM subscene 轮,per-VERTEX env 方案)
最终落地用 **per-vertex env id**(而非 O(env²) 矩阵):因 FEM 是一个合并 body,只有"每顶点"标签能区分 env;广相直接比对两端点 env,跨env(都≥0且不同)跳过。比 O(env²) 矩阵更简、O(1)/对、FEM+ABD 统一、与求解解耦(仅接触过滤)。

**引擎(Stiff-GIPC-06me)**
- `mlbvh.cu`:`__device__ const int* g_vertex_env_id` 全局 + `_same_env(vA,vB)`(null/任一<0/相等 → 允许)+ host `mlbvh_set_vertex_env_id()`;**8 个广相查询点**(`_selfQuery_vf/_vf_ccd`×2 PT + `_selfQuery_ee/_ee_ccd`×2 EE)各加 `&& _same_env(...)`。
- `mlbvh.cuh`:声明 `mlbvh_set_vertex_env_id`。
- `sim_engine.{cu,h}`:`set_vertex_env_ids(vector<int>)` setter(grow-only device 数组 + `cudaMemcpyToSymbol`)+ `#include mlbvh.cuh`。
- `bindings/pystiffgipc.cu` + `stiff_physics/engine.py`:暴露 `set_vertex_env_ids`。
- 已重建 `build_312`(仅原有 warning)。

**Solver(newton)**
- 删除 pre-finalize 的 O(body²) `_apply_env_collision_groups`(FEM 时旧代码直接 bail→靠间距)。
- 新增 post-finalize `_apply_env_vertex_isolation`:ABD 顶点 env=`b//bpe`、FEM 顶点 env=`i//ppe`,建全顶点 env 数组 → `set_vertex_env_ids`。env内自碰撞/gripper/ground 排除仍走 body-matrix(不变)。
- num_envs 来源:IsaacLab 显式设 `model.num_envs`(native 脚本需手设;<2 则不启用)。

**测试(均自断言 PASS)**
- `examples/test_env_isolation.py`(引擎级):两 FEM 立方,`collide` dy=0.100 & 穿透=−0.0002(严格无穿),`isolate` dy≈0 & 重叠=0.100(穿过)。
- `scripts/test_stiffgipc_env_isolation.py`(solver 级):env1 立方落向 env0,ON z_sep=0(穿过)、OFF(num_envs=1)z_sep=0.041(堆叠)。
- backend 6/6 在 06me 引擎全 PASS(基线无回归)。

**注意**:newton solver 默认 `STIFFGIPC_ROOT=".../Stiff-GIPC-v065"`(旧 checkout,无 `set_vertex_env_ids`)。跑本轮需 `STIFFGIPC_ROOT=/home/ps/Downloads/Stiff-GIPC-06me`;正式部署走安装的 wheel(需从 06me 重新发布 wheel 后 IsaacLab 才用到本改动)。

---

## 目标与原则
- **统一目标分支**:stiff-physics **v0.6.6** 之上开 `unified/v066`(+ newton fork + IsaacLab fork 对应分支)。
- **必要性驱动**:每个改动都配一个"必要性测试"——**去掉它测试必失败,加上才通过**。通不过必要性测试的改动 = 不移植。
- **ground d=0 守卫用条件式**(`dist2==0.0 ? 1e-12 : dist2`),**不能照搬 v0.6.5 的 `fmax(dist2,1e-12)` clamp**(本 session 已证明 clamp 导致 alpha→0 冻死)。
- 诊断 / 半成品(S4 mask、STIFFGIPC_* env-gate)不进主线。
- 引擎提交按 **haoxiang002** 作者身份。

## 版本现状
- `Stiff-GIPC-v065`(tag v0.6.5)= 大改动清单所在(13 文件 ~993 行)。
- `Stiff-GIPC-06me`(v0.6.6)= 当前工作区,已含本 session 的 ground d=0 条件式修复 + teleport fem_offset + absolute_dhat 打通。
- 公开发布 = stiff-physics v0.6.6 wheels。

---

## Phase 0 — 准备(不改功能)
- [ ] P0.1 在 v0.6.6 上开 `unified/v066` 分支(三个 repo)。
- [ ] P0.2 精确 diff:`git diff v0.6.5-base..Stiff-GIPC-v065` 逐 hunk 导出;与 v0.6.6 baseline 对照,标出冲突点(尤其 GIPC.cu 的 ground 守卫)。
- [ ] P0.3 建测试 harness `test_stiffgipc_necessity.sh`:一键跑下面所有"必要性测试 + 回归",每项输出 PASS/FAIL。
- **门槛**:harness 能在 v0.6.6 baseline 上跑通(基线通过项记录下来)。

## Phase 1 — 引擎改动(逐条 port + 必要性测试)
每条:先在 unified 上 **不加改动**跑必要性测试(应 FAIL)→ port 该改动 → 再跑(应 PASS)。

| # | 改动 | 必要性测试 | 判据(无→有) |
|---|---|---|---|
| E1 | 收敛阈值绝对化 (eff_bboxDiagSize2) | 同策略跑 num_envs=2/9/32 | 无:行为随 env 数变(2 走/32 站);有:一致 |
| E2 | dhat 绝对化 | num_envs=1/16/64 测接触力/穿透 | 无:随 bbox 变、跨 env 耦合;有:一致 |
| E3 | 重力体力旋转项 | 自由铰接杆重力下自由摆 | 无:不倒/冻;有:倒到 ≈ -π |
| E4 | 关节限位 penalty (rev+pris) | 驱动关节超限 | 无:越界;有:被挡在限位内 |
| E5 | **slew cap** | 单帧命令大关节跳变 | 无:Newton 触顶(2000 iter 也不收敛);有:收敛 |
| E6 | mesh-less proxy API (exclusion/groups/inertia/density setter) + solver proxy | gripper-only 碰撞 + articulation 自碰撞排除 | 无:全臂碰/自碰撞冻死;有:仅夹爪碰、无自碰撞冻 |
| E7 | **ground d=0 守卫(条件式)** | 软块落地 + 夹爪压块 | 无守卫:NaN 冻;v0.6.5 clamp:alpha→0 冻(本 session 已证);条件式:0 触顶且 dist 恒>0 ✅ |
| E8 | per-body 密度/惯量 | 质量/惯量读回 vs 设定 | 无:错;有:对(**仅当有任务用到才 port**)|

- **注**:E7 是唯一"照搬会出错"的——必须用条件式版本,并用三方对比测试(无/clamp/条件式)证明。

## Phase 2 — Newton solver(port + 测试)
| # | 改动 | 必要性测试 | 判据 |
|---|---|---|---|
| S1 | Kabsch/浮动基座读回 + 角速度 | 驱动后读回位姿 vs 命令 | 无:漂移/错;有:一致 |
| S2 | 关节映射 qd0 修复 | 自由基座(7 coord≠6 dof)关节读回 | 无:错位;有:对 |
| S3 | 跨 env 碰撞排除 | 多 env,env 间不应互碰 | 无:跨 env 穿透;有:隔离 |
| S4 | 前臂/非夹爪排除 | 仅夹爪与软块碰 | 无:前臂顶块;有:仅夹爪 |
| S5 | converter: collision_mesh_segments / mesh_volume | mesh-less proxy 依赖 | proxy 生成正确 |
| — | (不 port) S4 per-env mask 半成品 | — | — |

## Phase 3 — IsaacLab 任务 preset
| # | 改动 | 必要性测试 |
|---|---|---|
| L1 | cartpole stiffgipc preset | cartpole 训练冒烟(能学)|
| L2 | ant preset + entropy(**站立惩罚等临时调参需定稿**)| ant 走路 |
| L3 | shadow_hand preset + newton 变体别名(**segments/kappa 实验值需定稿**)| shadow 冒烟 |
| L4 | lift_franka_soft 场景 + 软体奖励 mdp | lift 训练:reward 上升 + 0 触顶 |
| L5 | stiffgipc_manager_cfg 选项透传(segments/torso_skip/absolute_dhat/slew)| 各 preset 能读到 |

## Phase 4 — 回归套件
- [ ] cartpole / ant / shadow_hand / lift 各自 6/6 或训练冒烟全绿。
- [ ] 多 env(2/9/32)一致性(E1/E2 复测)。
- [ ] 无触顶回归:lift 训练 100 迭代触顶率 ≈ 0。

## Phase 5 — 清理与定稿
- [ ] 删除引擎诊断:`dump_state_raw`(sim_engine.cu/.h/bindings)。
- [ ] 删除 solver 诊断 env-gate:DIAG/DUMP/FBD_NOTELE/FORCE_BLOCK_DEMO/FREEZE_ARM/LAND_DUMP/RESET_DBG/RESET_DEMO/ROTATE_TEST/STATE_DUMP。
- [ ] 定稿实验参数(ant 惩罚、shadow segments/kappa)。
- [ ] 保留但明确的功能开关:`NO_ENV_GROUPS`、`PY_JOINT_LIMITS`(文档化)。
- [ ] 提交:引擎按 haoxiang002 身份;推到 umbrella `haoxiangNtu/stiffgipc-isaaclab` + stiff-physics(v0.6.6)+ newton fork + IsaacLab fork。

## 关键决策点(需你确认)
1. 统一目标确认为 v0.6.6?(而非再等 v0.6.7)
2. E8(per-body 密度/惯量)、S3/S4(排除)是否所有任务都需要,还是任务特化?
3. shadow/ant 的实验参数由谁定稿?
4. "必要性测试失败即不移植"的原则是否严格执行(可能删掉一些看似有用但没测试支撑的改动)?
