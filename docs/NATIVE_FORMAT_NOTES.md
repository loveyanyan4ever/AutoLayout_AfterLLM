# 原生格式支持依据与边界（0.2.0）

核查对象：`ProPrj_数字电源_功率板_2026-09-23.epro2`，SHA-256 `4b98e301328db25bb9a1841967888c26dfa1265824b7d7d4046f535118f8b7b7`。下列统计来自修复前的独立只读核查；源输入未修改。0.2.0 以这些证据实现适配，源格式未明确的部分仍通过显式策略限制。

## 1. 空负载是可以有依据支持的删除记录

[官方 v3 日志格式](https://prodocs.lceda.cn/cn/format/index/)说明：同文档、类型和 ID 取较大的 `ticket`；相同 ticket 时取文档头 `client` 较小者；内层空字符串表示删除。删除记录保留在日志中。文档删除另有 `DELETE_DOC/isDelete` 标识，不能与单图元删除混淆。

实际文件使用空文本 `|||`，而非 JSON 字符串 `||""`。二者均为空负载，但解析时必须区分普通 JSON null、损坏 JSON、非空非对象负载，不能把所有解析异常吞成删除。最高版本是墓碑时，该对象不进入有效对象集合；历史原始字节仍保留。

本文件共 13,510 行、308 个文档、13,202 条原子记录。同文档/类型/ID 没有重复版本，故其文件表现为保留墓碑的快照：

| 文档 | 类型 | 空负载数 |
|---|---|---:|
| SCH_PAGE | LINE | 5 |
| PCB | NET | 19 |
| PCB | PAD_NET | 1,106 |
| PCB | RULE_SELECTOR | 86 |
| 合计 | | 1,216 |

原程序对第一条空负载执行 `json.loads('')`，属于未适配删除语义导致的失败，并非源文件损坏证据。全局解析必须支持上述语义，才可到达 PCB 优化。

## 2. 真实 PCB 没有活动板框与活动独立布线

目标 PCB UUID：`2a21e239da271e7b`，标题 `PCB2`。有效组件 103 个、位号唯一 103 个，顶层 103 个，原生锁定状态全部 false。

本 PCB 的原始记录类型穷举如下；有效状态同时去掉空负载墓碑：

| 类型 | 原始数 | 有效数 |
|---|---:|---:|
| META / CANVAS / ACTIVE_LAYER | 各 1 | 各 1 |
| PRIMITIVE | 37 | 37 |
| LAYER / LAYER_PHYS | 89 / 13 | 89 / 13 |
| RULE_TEMPLATE / RULE | 1 / 15 | 1 / 15 |
| SILK_OPTS / PREFERENCE / PANELIZE | 2 / 1 / 1 | 2 / 1 / 1 |
| NET | 67 | 48 |
| PAD_NET | 1,375 | 269 |
| POURED | 274 | 274 |
| RULE_SELECTOR | 86 | 0 |
| LAYER_FILL / ELE_PLACEHOLDER | 1 / 5 | 1 / 5 |
| COMPONENT / ATTR | 103 / 299 | 103 / 299 |

`POLY`、`LINE`、`ARC`、`REGION`、`FILL`、`POUR`、独立 `VIA` 均为零，连历史记录也没有。`LAYER` 中确实定义了 11 号 `OUTLINE` 层，但没有该层上的几何；`PANELIZE.on=false`；唯一 `LAYER_FILL.fill=[]`。37 个 `PRIMITIVE` 都是显示/拾取配置。结合官方 PCB 图元模型，可认定这个输入快照没有活动 PCB 板框，并按无板框模式估算；不能把旧覆铜路径的外包框当成实际板框。

板框生成应在实际 `OUTLINE` 层写普通闭合原生折线，避免只依赖合成测试定义的 `polyType=BOARD_OUTLINE` 字面值。图层编号应从输入的层类型确定。官方层定义见[图层管理器](https://prodocs.easyeda.com/cn/pcb/tools-layer-manager/)和[当前图层枚举](https://prodocs.easyeda.com/en/api/reference/pro-api.epcb_layerid.html)。

## 3. 274 条 POURED 是无目标的派生结果

[官方形状图元规范](https://prodocs.easyeda.com/cn/format/pcb/shape/)定义 POURED 为覆铜结果，关联一个 POUR 边框。[当前 API](https://prodocs.easyeda.com/cn/api/reference/pro-api.ipcb_primitivepoured.html)也提供查询其覆铜边框 ID 的接口。

实际文件采用较新的表示：外层 ID 为 `["POURED","目标 UUID"]` 的 JSON 字符串，正文仅有 `pourFill`。274 个目标 UUID 在整个工程所有文档的图元 ID 中均不存在，也没有任何 POUR 源图元。因此可将这些结果识别为孤儿派生缓存，保留原始日志文本，在布局模型中排除并记录诊断。

这一规则必须严格限定：存在活动 POUR、无法解码关联目标、存在目标对象但语义不明时，不得静默忽略。不能据本文件特征把所有工程中的 POURED 当作无用数据。尚未实际调用原生编辑器，因而不声称已验证其 GUI 打开、缓存自动刷新或原生 DRC 行为。

## 4. 封装、单位与几何证据

### 4.1 封装引用需允许 Device 回退

103 个器件实际引用 **25 个封装**。10 个器件未直接提供有效 Footprint 属性，需通过 `Device → DEVICE.META.attributes.Footprint` 获取：`+12V`、`+24V`、`3V3`、`GND`、`GND1`、`GND2`、`GND3`、`LED2`、`LED3`、`VCAP1`。回退必须验证文档类型和有效引用，不猜测相似名称。之前仅统计直接 Footprint 属性得到的 23 个封装是不完整统计。

25 个有效封装共 82 个不同 PAD：RECT 47、ELLIPSE 18、OVAL 16、POLYGON 1；按器件实例展开时数量另计。机械封装中多个 PAD 共用编号 `1`，稳定图元 ID 不同；必须保留各个物理焊盘，不能因同号合并或拒绝。`PAD_NET` 的稳定焊盘 ID 必须用于映射和身份校验。

### 4.2 source_unit=mil 由尺寸交叉检查确认

[官方 CANVAS 说明](https://prodocs.easyeda.com/cn/format/pcb/common/)明确：`unit` 是显示单位，不改变数据单位。所以 `CANVAS.unit=mil` 只能作为线索，不足以单独确定数值比例。

本文件中 C0603 封装 `9f3bab99b2f00a99` 的 COMPONENT_SHAPE 路径范围为 `x=±31.496、y=±15.748`；按 mil 换算是 `1.5999968 × 0.7999984 mm`，与其 1608 名义尺寸一致。M3 封装中心焊盘直径 `255.9055 mil≈6.5 mm`，孔尺寸 `125.2×125 mil≈3.18008×3.175 mm`。这些独立尺寸共同支持此输入使用 `source_unit=mil`，保留显式配置，不根据显示单位自动改数据比例。

### 4.3 本体层、工艺填充与封装内过孔

实际层定义与[当前官方枚举](https://prodocs.easyeda.com/en/api/reference/pro-api.epcb_layerid.html)一致：48=COMPONENT_SHAPE，49=COMPONENT_MARKING，50=PIN_SOLDERING，51=PIN_FLOATING。官方图层说明把 COMPONENT_SHAPE 定义为元件实物外形；可将其闭合几何的保守包络用作本体代理，但不能称作经过验证的完整 courtyard。

25 个有效封装有 23 条 layer48 POLY。163 个 FILL 全部位于非铜工艺/文档层：49层23条、50层78条、13层20条、3层14条、7层6条、51层22条，所有 `netName` 为空。没有需要建模为信号铜的封装 FILL。允许非铜层元数据时仍须保存源记录；遇到其他文件的信号层 FILL，必须另作几何建模或拒绝，不能沿用当前结论。

活动器件引用的 `ccbcb0622ce78c46` 封装包含 4 条 VIA，ID 为 `e14`～`e17`，`viaDiameter=24`、`holeDiameter=12`，封装字段 `netName=''`。进一步核对 PCB 实例的 PAD_NET 后，确认这四个 VIA 分别通过完整稳定身份映射到 GND（原日志第 13164～13167 行），必须加入对应器件的 GND 引脚集合，不能只根据封装空网名认定无网络。最终目标板为 265 个 PAD 加 4 个封装 VIA，完整对应 269 个有效 PAD_NET。工程中另一个未选用库封装也有 4 个 VIA，不应将其加入目标板统计。

### 4.4 padOffset 与多边形坐标的边界

[官方焊盘说明](https://prodocs.easyeda.com/cn/format/pcb/pad_via/)和[当前焊盘 API](https://prodocs.easyeda.com/cn/api/reference/pro-api.ipcb_primitivepad.html)均将 `padOffsetX/Y` 定义为孔偏移，将 `relativeAngle` 定义为孔相对焊盘的角度。不能用这些值平移铜焊盘；无孔 SMD 中残留小偏移不会改变铜外形。真实孔包含 ROUND 和 SLOT，孔形状与铜形状需分开处理。

Q1～Q4 复用的唯一 POLYGON PAD 路径外包框中心约 `(-0.18,42.6475)`，而 PAD center 为 `(-0.18,42.65)`，padAngle=0，这支持路径已经位于封装坐标系的解释。但公开旧格式说明与当前字段形式不完全一致，[当前外形 API](https://prodocs.easyeda.com/cn/api/reference/pro-api.tpcb_primitivepadshape.html)也未明确解释路径原点，因此该推断不能冒充已验证格式事实。建议以显式坐标系选项或覆盖绝对/相对解释的保守包络处理，并在评估报告标注；不修改原始 PAD 记录。

## 5. 写回与验收建议

采用追加事件模式：保持原始 ZIP 成员、封装、原理图、网表与原日志字节，新增事件仅属于目标 PCB 文档。姿态事件复制原始 COMPONENT 正文，只更新受支持的位置/角度字段；组件的 ATTR 世界坐标与角度追加跟随变换事件；同一板内新增 ticket 取原最高值以上递增。新板框使用已存在的 OUTLINE 层，闭合数值路径。

导出后应独立核对源日志作为字节子序列完整保留、非日志成员 SHA-256 相同、所有有效组件 UUID/位号、稳定焊盘身份和网络集合不变、姿态与板框读回一致、约束重新计算通过。解析器自身读回是结构与算法验收，不等于原生编辑器打开验证。只有真实执行过原生编辑器导入/另存或原生 DRC 后，才能将对应状态由 PENDING 改为 PASSED。

随包真实输入位于 `examples/real_power_board/`，可执行以下入口复查适配与配置：

```bash
python -m pcb_hierplace inspect --project examples/real_power_board/source.epro2
python -m pcb_hierplace validate --config examples/real_power_board/constraints.yaml
```

上面是项目入口校验；独立归档统计和自动布局验收结果见 [TEST_REPORT.md](TEST_REPORT.md)。本次真实板全部器件保留源 0°角度，尚未通过原生编辑器验证非零角度写回。原生打开、另存和 DRC 均不得由自身读回结果替代。
