# 动态心理人群模型 sis_cf_v1

版本1.0，2026-10-13复查修订，开发分支 `feature/dynamic-psychology-v1`。
实施规格见 [DYNAMIC_PSYCHOLOGY_PLAN.md](DYNAMIC_PSYCHOLOGY_PLAN.md)。

## 数学与状态语义

正常运动复用当前项目multi_sfm regular。每人c/f∈[0,1]可以共存，是现象学激活量，
不是互斥概率。动作REGULAR/CURIOUS/SCARED独立于心理状态，REGULAR可带vigilant警戒。

```text
lambda_q = alpha_q*direct_q + beta_q*sum_j(w_ij*cue_jq), q=C/F
dc/dt = (1-c)*lambda_C - mu_C*c - kappa*f*c
df/dt = (1-f)*lambda_F - mu_F*f
cue_C = expressivity_C*c; cue_F = expressivity_F*f
```

采用用户选择的连续外显线索，不按模式置零，是细微心理外显的仿真假设，不是情绪识别。
边j→i表示i观察j；距离核`max(0,1-d/spread_range)^2`乘有向FOV，
再除以`max(1,raw_row_sum)`，每行权重和≤1，接触数量效应会饱和。

全员使用步前快照，0.025秒仿真步长。冻结lambda，令dC=muC+kappa*f_old、dF=muF，
用`z_new=z_old+(lambda/(lambda+d)-z_old)*(-expm1(-(lambda+d)*dt))`更新。
它是冻结子问题精确更新，整体耦合仍是一阶近似；明显越界报错。

默认alpha=0.60/1.00、beta=0.06/0.12、mu=0.12/0.25、表达系数0.8/0.8、kappa=0。
默认beta<mu提供无外源时的保守衰减基础。增强beta=1.20/2.40，含环网络可能持续激活。
kappa=1作为独立消融。全部参数是工程调试值，尚无真人标定。

## 感知、记忆和行为

风险使用净距、相对接近速度、有限时域最近接近，不以机器人绝对速度代替威胁。
直接fear=`1-product(1-V*risk)`；直接curiosity=`(1-direct_fear)*(1-product(1-V*novelty))`。
新颖性只在可见时指数衰减，短暂消失不重置。

机器人视觉默认8米/200°/每侧10°过渡；传播默认3米及相同角度形式。
yaw来自权威运动，零速保留；没有遮挡、听觉或独立头部朝向。运动转向导致失去视野时
不得读取隐藏位置继续追踪。

个人记忆保存最后观察的位置/速度/半径/时间。预测最多0.75秒，TTL2秒，置信度按0.8秒
时间常数衰减，低于0.2失效。记忆不额外产生直接刺激，心理本身保留过去影响。
传播不携带机器人ID/坐标。无来源高恐惧只能REGULAR+vigilant；无兴趣对象不能凭空接近。

优先级：紧急风险 → 有来源恐惧 → 低风险好奇 → 原任务。
好奇阈值0.60/0.35，恐惧0.65/0.35，警戒0.08/0.04；好奇还要求fear≤0.25、直接风险≤0.25。
紧急风险0.85可抢占，不强行改fear。非紧急模式最短保持0.5秒，风险/目标失效不受其阻挡。
目标保持1秒、优势差0.15、平分按稳定ID，更紧迫威胁可抢占。

CURIOUS生成有界比例接近/后退速度，最大0.35/0.20m/s；观察距离与容差在配置中冻结。
停留带用于验收，不在边缘突变切零。SCARED综合所有已知威胁远离，上限0.8m/s；
方向抵消时采用主要威胁，达到目标净距后减少运动。警戒期望速度系数`1-0.4*f`。

特殊行为只替换目标驱动力，仍叠加原墙体、人际、人机社会项及near。

好奇接近还进行局部可行性检查：观察5米内、位于人际FOV中的邻居，按候选期望速度与
邻居当前速度预测2秒内最近净距。若预计不足0.35米，则保留心理激活、退出定向接近，
先产生零期望速度并等待2秒后重试，日志标记approach_blocked/approach_wait。
紧急恐惧仍可抢占等待。邻人也有独立的短时观测记忆，采用同一TTL/衰减/预测上限；
记忆不能读取隐藏邻人的实时位置，也不参与心理传播。

2026-10-13实际故障复查发现：只检查好奇接近不足以保护返回原路线的交叉运动。
因此对特殊行为、等待、警戒以及返回原任务期间增加局部期望速度筛选。
对于已观察/有效记忆的邻人，令n为邻人指向自己的单位向量、g为净距，要求
`n·v_desired >= n·v_neighbor - (g-0.35)/2`，速度模长≤1。
在这些线性约束与速度圆内选择离原期望速度最近的点；不可行时选有限候选中最小最大
违反量的有界速度，并明确记录`avoidance_feasible=false`，不能宣称获得硬防碰撞保证。
允许避让覆盖原等待零速度，心理激活、目标知识和任务暂停语义不变。

`returning_to_task`与`return_goal`保存中断时的原队首目标；返回目标后或原队列正常推进后
清除，兼容循环/往返队列。无心理干预、无需返回且不警戒的REGULAR仍走原数值路径。
筛选只改目标驱动力，不修改人际社会力或机器人near，也不进行位置投影。

near、实体半径、硬速度1m/s及加速度3m/s²不随心理变化；零期望速度仍允许受限减速和物理排斥。
普通中性分支保留原LightSFM数值路径，覆盖分支使用半隐式Euler，每步只积分一次。

CURIOUS/SCARED暂停原任务队列。恢复REGULAR按实际位置、速度继续loop/ping_pong，
不瞬移、不消费临时目标，每步最多推进一个原目标。

## 配置和启动

场景使用`psychology_model: sis_cf_v1`及`psychology_config`，路径相对场景目录。
配置schema_version=1、occlusion_model=none_open_space，未知键/非法值明确失败。
config/psychology提供默认、direct_only、fear_only、enhanced和inhibition五组完整配置。
新心理场景使用独立六人路线，旧多目标路线与普通动力学保留。
本轮完整Isaac几何与耐久验收针对默认sis_cf_v1场景。增强配置验证了固定链传播机制，
不共用默认场景的30分钟几何结论；改参数或路线后应重新验收。

```bash
cd /home/lpc/workspace/arena5_multi_ws
# 首次或源码更新后
./scripts/build.sh
./scripts/validate_offline.sh
# 仅启动，机器人目标人工发送
./scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml
# 自动发送有限轮双机Nav2任务
./scripts/run_multirobot.sh config/scenarios/two_robots_multi_sfm_6_multigoal_psychology_nav2.yaml auto_demo:=true
```

启动脚本自行加载环境，可视化变量沿用command.txt。自动任务文件为psychology_crossings.yaml，
等待健康、十次显示更新及Nav2 action就绪；只发送机器人目标，不编排行人心理或模式。
失败取消本轮目标并保存原因；完成后场景继续运行；Ctrl+C退出。

## 接口、事务和证据

noop保留v1服务；新心理使用`/multirobot/hunav/compute_agents_v2`及ComputeMultiAgentsV2。
BehaviorIntent恰好覆盖全体行人，枚举REGULAR=0/CURIOUS=1/SCARED=2，带警戒、目标标志/ID、
二维map期望速度覆盖及allow_task_progress。目标ID只作检查，核心不能重新查询真值生成认知方向。
`Agent.behavior.type`保留兼容值1，实际模式以新意图和心理诊断为权威。
v2响应增加力分项及限幅标志，保留原时间、身份与epoch/step契约。

服务无状态，机器人不积分。prepare_step产生候选并回滚内部快照；响应、任务、数值和快照
先校验再共同提交。失败/迟到/超时不发布候选事件；仿真时间驱动心理，墙钟驱动运行保护。
时钟回退锁存停车，重启场景恢复。

心理话题schema_version=2在提交后发布；noop保留schema_version=1。
运行目录保存psychology_<epoch>.jsonl：新旧c/f、直接/传播输入、步前瞬时导数分项、
有向边、行为原因、目标来源/年龄、意图、回归目标、邻人记忆、避让可行性、任务队列、
力分项及限幅；同时记录权威行人位置/速度和本次服务输入机器人位置/速度，便于复现。
日志流式写入，不在内存中无限积累；磁盘日志会持续增长，应按实验保留周期归档。
瞬时导数×dt与指数更新增量分别解释。psychology_demo.json只证明机器人任务执行。

```bash
source scripts/env.sh
ROS_DOMAIN_ID=84 python scripts/validate_psychology_cpu.py --output-dir evidence/runs/psychology/cpu
```

CPU工具启动私有真实ROS核心，不接触Isaac或底盘。固定链让B/C直接刺激严格为零，
验证间接警戒和增强第二跳；单人先进行注明的冻结观察再释放运动，心理值仍全部由模型生成。
六人比较相同机器人输入下的noop与心理闭环，报告净距、限幅、任务和模式。
额外重放失败Nav2实验的机器人观测轨迹：按真实last_seen_ns插值，缺失区间最长7.225秒，
最大位置变化速度约0.26002m/s。这是可复现回归输入，不能冒充完整真值里程计或实际Nav2验收。

已通过7包重构建、116项Python、29项C++；五类真实ROS故障注入及CPU闭环结果见当前证据。
完整快照中的旧静态核心库与新v1核心在10组输入、1/6人、每组1200步上进行
24,000步逐字段精确比较，位置、朝向、速度、原队列和逐机器人作用完全一致。
修正后一机两人Isaac实际检查120.077秒墙钟通过；双机六人1800.050秒墙钟、
957.425秒仿真通过。六人场景最小人机净距0.290389米、人际净距0.386764米，
最大显示误差0.037520米，零失败；三轮六个实际Nav2目标全部成功。
验收使用GPU3、CPU亲和性28-55,84-111，关闭流媒体/Foxglove，保留0.6秒观测超时。
运行条件、失败迭代与最终源码摘要见证据清单；这些结果仅对应已测配置和运行条件。

新心理场景的路线验收必须识别任务暂停，不强套原无心理周期门槛。
GPU验收须先一机两人再双机六人，显示误差≤0.25米、人机净距≥0.10米，无人际/墙体重叠，
六人运行30分钟墙钟，报告仿真时长。CPU、虚拟显示服务不能替代实际Character Graph证据。

原SFM没有行人硬碰撞约束，多人接近与交叉布局必须逐场景验证。当前不包含全局逃生规划、
遮挡、真实情绪识别、群组或训练系统。开发结果与保留的失败试验见
[动态心理证据](../evidence/dynamic_psychology_20261012/README.md)。
