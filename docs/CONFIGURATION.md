# 输入契约与人工决策（0.2.0）

本文说明实际实现的字段。需求基准保持在 `BUILD_SPEC.md`，不通过修改需求来掩盖尚未验证的能力。程序默认值可由 `configure` 生成；完整实例见 `examples/dual_domain/constraints.yaml`。

## CSV

UTF-8 或 UTF-8 BOM，空行及以 `#` 开头的注释行允许。

隔离域：

```csv
designator,domain_id
H1,MECHANICAL
J1,HV
U1,LV
ISO1,BRIDGE_HV_LV
```

功能簇：

```csv
designator,cluster_id
H1,mechanical
J1,power_input
U1,controller
ISO1,isolation
```

两份 CSV 都必须完整覆盖选中 PCB 的所有位号，一件一行。通用可选列为 `component_uuid`、`note`、`cluster_name`；功能簇 CSV 还允许 `parent_cluster_id`。UUID 如提供必须与位号匹配。重复表头、数据行列数不一致、未知位号、重复位号、缺失标签及未知列均报错。每个 `domain_id` 必须在 `rules.domains` 声明。簇 ID 是字符串，不要求连续编号。

若使用上层功能组，功能簇 CSV 可写为：

```csv
designator,cluster_id,parent_cluster_id
U1,L1-control,L2-power
R1,L1-control,L2-power
C1,L1-decoupling,L2-power
```

同一 L1 的所有成员必须声明同一非空父组。父组参与簇间紧凑度目标并在报告中保留，不是电气隔离域，不另外生成保证互不重叠的 L2 硬区域。真实数字电源板的 L2 按此方式保留；多个父组共享网络是允许的。跨电气域拆簇时若生成的 ID 与已有簇名碰撞，会报错而不是合并。

固定件和接口保留原有逻辑簇成员关系，但不作为可自由移动的矩形簇成员；它们作为簇的附着目标/外部锚点。跨 HV/LV 的普通簇默认报错；仅显式 `clusters.cross_domain_policy=split_by_domain` 才拆为物理簇，并记录拆分。`bridge_template` 策略不会自动把普通跨域簇转换为桥接件，桥接必须声明真实器件模板。

## 必须由设计者给出的内容

| 字段 | 含义和约束 |
|---|---|
| `input.project` | 源 `.epro2` |
| `input.board_id` | 多板工程必须明确选择；单板可自动选中 |
| `input.voltage_domains` / `functional_clusters` | 两份完整 CSV |
| `input.adapter` | 原生 v3 使用 `easyeda_pro_v3`；保留合成/旧参考输入名称 `reference_v3` |
| `input.source_unit` | `mil`、`mm`、`0.01inch` 或 `0.01mm`；未知单位拒绝 |
| `rules.id` / `basis` | 规则版本，以及设计规则来源/适用条件说明 |
| `rules.mechanical_gap_mm` | 本体之间最小机械间距 |
| `rules.default_pad_clearance_mm` | 不同器件间普通异网导体的最小铜间距 |
| `rules.domains` | 电气、机械、桥接标签及类型 |
| `rules.isolation_pairs` | 两电气域之间的铜间距、爬电目标、布线预留 |
| `geometry.body_overrides` | 位号或封装 ID 对应的显式本体几何；也可启用下面的源本体读取策略 |
| `rules.component_rules` | 孔、接口、固定件、区域限制等真实机械意图 |
| `rules.bridge_templates` | 每个桥接件的真实 pad 分域、隔离轴和基准 |
| `board.estimated.max_width_mm` / `max_height_mm` | 无板框时允许搜索的最大尺寸 |

规则合并顺序是内建研究默认值 < YAML < 显式 CLI 覆盖。最终配置和哈希写入运行目录，未知字段报错。通用配置不会自动补齐安全/机械设计参数；随包真实案例的经验值经过本次用户授权，写入案例 YAML 的 `rules.basis`，可以修改，不代表适用于其他板。源锁定仅在明确 `unlock_source_locked` 中列出时才允许解锁。

## 坐标与几何

所有规则几何都采用物理 **mm**。全局坐标相对于源 CANVAS 原点，X 向右、Y 向上；角度为逆时针度。局部几何相对于器件真实参考点，不能默认对称居中。适配器沿用 X 右/Y 下和反号角映射；`source_unit` 只缩放源坐标，不缩放规则里的 mm。CANVAS 的 `unit` 是显示单位，须用原生格式或已知封装尺寸确认数值比例。本次真实板所有源角度均为 0°，最终配置保留 0°，未宣称原生非零角度已通过编辑器验收。

`body_overrides` 的键优先匹配位号，然后匹配封装 ID。值可以是 `[xmin,ymin,xmax,ymax]`，也可以是有序凸多边形顶点列表；自交、退化或非法坐标会拒绝。不会将任意丝印当本体。缺少几何时默认报错；只有把位号/封装显式加入 `geometry.approved_pad_envelopes`，才接受焊盘包络加 `pad_envelope_margin_mm` 的保守代理，并记录代理来源。没有焊盘且没有支持的源本体轮廓的器件仍须显式提供本体。

0.2.0 新增两个源几何选项：

```yaml
input:
  adapter: easyeda_pro_v3
  source_unit: mil  # 必须有尺寸或格式依据
geometry:
  source_body_policy: source_courtyard_or_assembly
  polygon_path_frame: conservative_union
```

`source_body_policy` 默认为 `explicit_only`。选择 `source_courtyard_or_assembly` 后，可以读取明确的 COURTYARD/ASSEMBLY 或层定义为 COMPONENT_SHAPE 的闭合几何，生成包含图线宽度的保守 AABB；后者只是源实物外形代理，不是经过制造验证的完整 courtyard。显式 `body_overrides` 优先级更高。曲线、本体与铜的源记录保持原样。

`polygon_path_frame` 默认为 `reject`，遇到 POLYGON 焊盘须明确选择：`footprint` 把路径视为封装坐标；`pad_local` 按 pad 局部坐标旋转/平移；`conservative_union` 使用两种解释的共同外包凸包。本次真实板采用最后一种，避免低估但可能增大占位；不能称为已证明的原生坐标解释。

铜外形支持 RECT、CIRCLE/ELLIPSE、OVAL/ROUND 和上述显式策略下的 POLYGON。椭圆/长圆使用外包几何；孔单独解析，`padOffsetX/Y` 作用于孔，不平移铜盘。圆孔、槽孔与其偏移参与完整占位和独立检查。稳定焊盘 ID、同号的不同物理 PAD，以及封装内 VIA 都保留；网络按 PCB 实例 PAD_NET 的完整身份映射，不能只读封装 `netName`。未支持的特殊层焊盘仍拒绝。

固定板框必须是单个轴对齐矩形。当前拒绝凹板、多环及内孔，而不是把它们替换为包围盒。真实板内禁区可以用 `rules.obstacles` 显式表示，但这不等同于支持原生内孔的读写。

## 器件运动规则

| `mode` | 行为 |
|---|---|
| `free` | A/B 中按允许角度离散选向、连续平移 |
| `fixed_pose` | 保持源位置、源角度和源安装面；`pose` 仅支持 `source` |
| `edge_slide` / `edge_choice` | 按声明板边、沿边区间和朝向采样机械候选，B/C 冻结选定结果 |
| `corner` | 按明确的板角和完整占位内缩距离预布局机械件，B/C 冻结 |
| `edge_band` | 保持源角度，将完整占位放到指定板边的内缩位置；不推断插拔方向 |
| `region_bounded` | 完整占位限制在指定矩形区域；桥接器件由桥接模板定义隔离界面 |

未显式给 `allowed_angles` 时只保留源角度，包含非 90° 原角度；不会擅自扩大角度集合。允许角度例：`[0,90,180,270]`。

机械域的无铜孔件需要明确 `conductive_class: insulating`；金属安装件使用真实电气域 ID。导电本体无论是否同时带焊盘都参与铜间距检查。机械件使用 `fixed_pose` 保持源位置，或使用显式 `corner` 模板重新预布局；未明确导电类别会报错。

0.2.0 板角/边缘带示例：

```yaml
rules:
  component_rules:
    H1:
      mode: corner
      allowed_corners: [bottom_left]
      corner_inset_mm: 1.5
      conductive_class: MAIN
      allowed_angles: [0]
    J1:
      mode: edge_band
      allowed_edges: [right]
      edge_offset_mm: 1.5
      allowed_angles: [0]
```

`corner` 的四种角为 `bottom_left`、`bottom_right`、`top_left`、`top_right`。`corner_inset_mm` 是旋转后完整占位包络距相邻两条板边的距离，不是孔中心距边。`edge_band` 的非负 `edge_offset_mm` 是完整占位距指定板边的**向内**距离，与下述有插拔矢量的边接口模式符号语义不同；该模式必须允许并保持源角度，完整占位不得越板。二者都保留源锁定规则，并将选中预布局姿态冻结到后续阶段；估算板尺寸变化时重新预布局。

有明确插拔基准和朝向的接口使用 `edge_slide` / `edge_choice`，规则需要全部给出：

```yaml
J1:
  mode: edge_slide
  allowed_angles: [0]
  allowed_edges: [left]
  edge_segment_mm: [8, 32]
  local_mating_point_mm: [-2, 0]
  local_outward_vector: [-1, 0]
  edge_offset_mm: 0
  allow_body_overhang: false
  insertion_keepout: [-2, -3, 2, 3]
```

以上仍是合成示意，必须替换为真实几何。边段距离从板框左下角沿边量起：左右边用 Y，上下边用 X。`edge_offset_mm` 的正值沿该边外法向。局部插接向量旋转后必须朝外。固定接口也可同时声明这些边规则，此时既不能移动也必须满足位置/方向要求。

`allow_body_overhang=true` 仅允许本体外伸；焊盘仍必须在板内并满足隔离域约束。`insertion_keepout` / `keepout_geometry` 为局部矩形，不允许其他器件本体、焊盘或固定障碍侵入。

## 桥接模板

```yaml
rules:
  domains:
    HV: {kind: electrical}
    LV: {kind: electrical}
    BRIDGE_HV_LV: {kind: bridge, between: [HV, LV]}
  component_rules:
    ISO1:
      mode: region_bounded
      bridge_template: iso
      allowed_angles: [0, 90, 180, 270]
  bridge_templates:
    iso:
      between: [HV, LV]
      pad_domains: {'1': HV, '2': LV}
      axis_local: [1, 0]
      barrier_point_mm: [0, 0]
```

模板必须覆盖器件的全部真实焊盘号，两侧均需有 pad；同号多个物理 PAD 通过各自稳定 ID 保留，模板按编号指定相同电气域。`axis_local` 指向 `between` 的第一个域到第二个域。旋转后的 `barrier_point_mm` 必须位于隔离带中线；模板不能让桥接件免于铜间距检查。铜间距检查包含同一桥接器件内部跨域 pad。

`creepage_mm` 被记录为目标，当前不把二维直线距离冒充沿表面爬电路径，报告始终为 `NOT_EVALUATED`。主域区域之间预留 `copper_clearance_mm + routing_reserve_mm`；求解阶段另加数值裕量。

## 板框与优化配置

`board.mode=auto`：源中有板框就固定，无板框就估算。`fixed` 必须有板框。`estimated` 覆盖已有板框时必须显式 `replace_existing_outline=true`；多记录板框当前不能在 estimated 模式替换导出，会明确拒绝。

估算模式以占位面积/利用率给出尺寸初值，在最大宽高以内搜索并尝试缩板。尺寸变更后重新生成沿边接口位置。`fixed_pose` 固定孔的源位置始终不动；`corner` 机械件按当前候选板角预布局。找不到布局时不扩大固定板框，也不修改间距规则。

| 字段 | 默认/含义 |
|---|---|
| `continuous_backend` | 仅 `numpy_analytic_float64`，逐项差分检查 |
| `projection_backend` | 仅 `osqp`，检查状态和约束残差 |
| `optimizer` | `projected_adam` 或 `projected_gd` |
| `priority_policy` | `auto`：固定板先 HPWL，估算板先面积；也可明确选择文档列出的策略 |
| `higher_priority_relative_tolerance` | 默认 0；容差相对于保留的高层参考值，不能逐轮累积放宽 |
| `iterations_a/b/c` | 每次候选内的最大梯度迭代数，须为正整数 |
| `max_topology_candidates` | 初始 B 搜索预算；估算缩板和 A′ 另有有限次数，仍受总时间上限约束 |
| `max_ab_feedback_rounds` | A′/B 反馈最大轮数，0 显式关闭 |
| `enable_stage_c` | 默认 true；false 用于 A+B 消融实验 |
| `stage_c.trust_radius_mm` | 相对 C 输入的欧氏位移上限；内部用内接方盒保守投影 |
| `stage_c.cluster_area_growth_ratio` | 相对 C 输入簇完整宽×高面积的增长上限 |
| `time_budget_s` | 优化阶段总时间预算；是软截止，当前一个几何检查/QP 调用结束后检查 |
| `preplace_samples` / `beam_width` | 沿边/桥接采样密度及机械候选 beam 宽度 |
| `output_grid_mm` | 输出取整偏好，取整后重新验证和评分；有风险则保留未取整合法结果 |

`allow_layer_change`、`stage_c.allow_rotation`、`stage_c.allow_cross_cluster_move` 必须为 false。硬簇包含不可关闭。数值裕量不表示电气间距，也不能替代规则。

## 引脚与障碍规则

```yaml
rules:
  obstacles:
    - {id: enclosure, bbox_mm: [20, 10, 23, 15], layers: [top]}
  regions:
    controller_area: [30, 5, 55, 35]
  distance_constraints:
    - {id: decoupling, a: 'U1:1', b: 'C1:1', hard: true, max_mm: 4}
    - {id: preference, a: 'U1:2', b: 'R1:1', hard: false, target_mm: 3, weight: 1}
  order_constraints:
    - {id: signal_order, a: 'U1:1', b: 'R1:1', axis: x, min_separation_mm: 0.5, hard: true}
```

这些同样只是字段示例。硬距离在初始化和后续投影中都参与可行化，不会为了得到初解而放宽最大距离。硬距离基于真实 pad 中心的欧氏距离，顺序是 `b[axis]-a[axis] >= min_separation_mm`。软距离的超限平方进入电气意图目标。`net_weights` 的键必须为实际网络 ID。求解采用保守占位与独立检查，有合法解不代表一定能在当前预算找到。


## 真实数字电源板默认与导出

真实例子见 `examples/real_power_board/constraints.yaml`。机械间隙 0.30 mm、异网铜间隙 0.25 mm 为用户允许的研究默认，理由写在 `rules.basis`；源本体使用上面的读取策略，M3 8×8 mm 和裸测试点 0.7×0.7 mm 是额外明确假设，不能视为精确厂家装配轮廓。MAIN 是本次共地布局域，MECH 是导电机械件分类，L2 用于父功能组；未凭空生成高低压隔离对。所有真实器件保留源 0°角度，预布局只改变孔/接口位置。

```yaml
export:
  emit_epro2: true
  require_model_feasible: true
  overwrite_source: false
  native_open_validation: pending
```

启用 `emit_epro2` 后，`run` 成功时生成 `board_placed.epro2` 和 `board_placed.export.json`。关闭时可以另行执行 `export --run 运行目录 --out 新文件.epro2`。已有目标工程或 `.export.json` 均拒绝覆盖。

导出保留原始日志和非日志 ZIP 成员，只向目标 PCB 追加新事件；组件的可见 ATTR 坐标与角度跟随其姿态变换，避免位号留在旧位置。新增板框采用原生 OUTLINE 层闭合折线。写回后核对完整网络、稳定身份、几何、位姿、板框和约束；这个读回检查不等于原生编辑器验收，`native_open_validation` 仍是 PENDING。

同一封装内部、同一电气域的普通焊盘关系按源库几何保留，不套用全局 `default_pad_clearance_mm`；这部分需要封装库或原生 DRC 验证。同一桥接封装内部的跨域铜间距仍检查。
