# pcb_hierplace 0.2.0

板级 PCB 分层自动布局研究工程：读取立创 EDA `.epro2`、功能簇 CSV 和电气域 CSV，先完成机械孔/接口等预布局，再执行 **A 簇内优化 → B 簇间 floorplan 优化 → C 整体微调**，通过独立约束检查后导出新 `.epro2`。

连续内核是 NumPy float64 解析梯度、projected Adam/GD 与 OSQP 投影；启发式负责离散方向、机械候选和 floorplan 拓扑。0.2.0 修复了上一轮审查的 R01～R12，并适配上传的数字电源板原生记录。具体修复见 [CHANGELOG_0_2.md](docs/CHANGELOG_0_2.md)，测试结果及真实运行指标见 [TEST_REPORT.md](docs/TEST_REPORT.md)。

需求原文保持在 [BUILD_SPEC.md](docs/BUILD_SPEC.md)。本项目不承诺任意 epro2 版本、任意约束组合都能求得合法布局；未支持的输入会明确报错。

## Ubuntu 安装

在解压后的项目根目录执行，使用 Python 3.11 或更新的兼容版本；已验证的解释器与依赖版本见验证报告。无需 GPU、PyTorch 或商业优化器。

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
```

若 Ubuntu 提示缺少 venv 模块，先安装对应 Python 版本的 `venv` 系统包。以后打开新终端，进入工程目录并再次执行 `source .venv/bin/activate`。也可使用安装后的 `pcb-hierplace` 命令，参数与 `python -m pcb_hierplace` 相同。

## 先运行一个合成示例

输出目录必须不存在或为空。以下命令显式启用自动导出：

```bash
python -m pcb_hierplace demo --out examples/local_demo
python -m pcb_hierplace inspect --project examples/local_demo/board.epro2
python -m pcb_hierplace validate --config examples/local_demo/constraints.yaml
python -m pcb_hierplace run --config examples/local_demo/constraints.yaml --out runs/local_demo --set export.emit_epro2=true
python -m pcb_hierplace evaluate --run runs/local_demo
```

成功时最终文件是 `runs/local_demo/board_placed.epro2`，相邻的 `board_placed.export.json` 记录读回与保留检查。合成示例的规则只用于测试。

随包的 `examples/dual_domain` 是固定矩形双域示例；`examples/estimated` 无源板框、需要估算并写入板框；`examples/conflict` 故意包含不可满足的区域限制，应报失败，不应靠扩大固定板框或放宽间距通过。

## 运行上传的数字电源板

`examples/real_power_board/` 包含原始工程副本、规范化 CSV、规则及数据证据。主输入为 `source.epro2`、`functional_clusters.csv`、`voltage_domains.csv`、`constraints.yaml`。

```bash
python -m pcb_hierplace inspect --project examples/real_power_board/source.epro2
python -m pcb_hierplace validate --config examples/real_power_board/constraints.yaml
python -m pcb_hierplace run --config examples/real_power_board/constraints.yaml --out runs/real_power_board
python -m pcb_hierplace evaluate --run runs/real_power_board
```

真实配置已设 `export.emit_epro2: true`，成功后直接得到 `runs/real_power_board/board_placed.epro2`。如果某个配置关闭了自动导出，或需要另一份导出文件，使用一个新的目标路径：

```bash
python -m pcb_hierplace export --run runs/real_power_board --out runs/real_power_board/placed_copy.epro2
```

这块板共有 103 个器件、25 个实际封装，原生连接为 265 个 PAD 加 4 个封装内 VIA；四个 VIA 均通过完整 PAD_NET 身份映射到 GND。10 个器件通过 Device 文档间接引用封装。适配器保留这些对象，不删件凑合运行。

L1 的 13 个簇用作局部功能簇；L2 的 6 个分组按用户确认保留为共地系统的上层功能组，以 `parent_cluster_id` 输入，参与父组紧凑度目标，**不作为六个互相隔离的电气域**。电子件使用 MAIN，四个导电机械件使用 MECH 并归属 MAIN 导电类别。

此输入没有活动板框，采用估算模式。用户允许重新预布局孔和接口：四个孔分配到四角，接口分配到边缘带。本次真实板全部器件保持源 0°角度，只优化位置；真实原生非零角度转换尚未通过编辑器验收。机械间隙 0.30 mm、器件间异网铜间隙 0.25 mm 是本次可配置的研究默认，来源和适用限制写在 `rules.basis`；它们不是安规认证值。M3 装配包络和裸测试点接近包络也有单独证据及假设说明。

## 多板选择与自己的输入

先列出工程内的 PCB：

```bash
python -m pcb_hierplace inspect --project my_project.epro2
```

将输出的准确 `board_id` 写入 YAML 的 `input.board_id`，也可在运行时覆盖：

```bash
python -m pcb_hierplace run --config constraints.yaml --out runs/selected_board --set 'input.board_id=实际板UUID'
```

多板未指定 ID 会报错；CSV 必须完整覆盖所选板。导出只新增所选板的事件，其他板和共享库对象保留。不同板使用不同的运行目录和匹配的 CSV。

新工程可以先生成配置模板：

```bash
python -m pcb_hierplace configure --project my_project.epro2 --out constraints.yaml --interactive
```

`--interactive` 需要真实终端；自动化中去掉此参数即可生成模板并列出未填决策，不会等待 stdin。所有相对输入路径均相对于 YAML 所在目录，`--set` 接受 YAML/JSON 值。例如：

```bash
python -m pcb_hierplace validate --config constraints.yaml --set input.source_unit=mil
python -m pcb_hierplace run --config constraints.yaml --out runs/seed2 --set optimization.seed=2 --set export.emit_epro2=true
```

单位必须有数据依据；CANVAS 的显示单位不能单独证明原生坐标单位。字段、CSV、孔和接口规则见 [CONFIGURATION.md](docs/CONFIGURATION.md)，原生格式依据见 [NATIVE_FORMAT_NOTES.md](docs/NATIVE_FORMAT_NOTES.md)。

## 结果如何看

| 文件 | 用途 |
|---|---|
| `board_placed.epro2` | 通过检查后自动导出的布局工程，自动导出关闭时不会生成 |
| `board_placed.export.json` | 源记录/非日志成员保留、连接与几何读回检查、输出哈希 |
| `report.json` | 最终合法性、源布局与 A/B/C 相关指标、耗时和终止原因 |
| `normalized_design.json` / `assignments.json` | 实际解析的器件、焊盘/孔、本体来源、网络以及 L1/L2/电气域映射 |
| `placements.json` | 最终器件位姿及域/簇区域，单位 mm、Y 向上、角度逆时针 |
| `layout.svg` | 源布局和最终布局可视对照 |
| `run_manifest.json` / `effective_config.json` | 种子、依赖版本、输入/规则/结果哈希及实际配置 |
| `inputs/` | 本次使用的源 epro2 和两份 CSV，支持搬移整个运行目录后复查 |
| `stageA/`、`stageB.json`、`pipeline.log` | 簇候选、拓扑与梯度优化记录 |
| `input_diagnostics.json` | 输入诊断或失败详情；失败时先看 `code` 和 `message` |

可使用随包的 Shapely/GEOS 独立检查脚本复核最终输出（需要安装 `.[test]`）：

```bash
python scripts/independent_evaluation.py --run runs/real_power_board --epro2 runs/real_power_board/board_placed.epro2 --out runs/real_power_board/independent_evaluation.json
```

该脚本共享文件解码器，但距离、相交、包含及原始日志检查不调用优化器内部验收函数。

随包也提供 `scripts/plot_verified_layout.py`，可安装可选的 `matplotlib`、`Pillow` 后，运行 `python scripts/plot_verified_layout.py --run runs/real_power_board --out runs/real_power_board/plots` 生成按实际几何绘制的对照图。绘图依赖不是布局运行所必需的。

先看 `report.json` 中的 `status=FEASIBLE` 和 `validation.model_feasible=true`，再比较 HPWL、板面积、簇/父组范围面积和阶段变化。HPWL 是网络半周长估计，不是布线完成后的真实线长。源散放布局可能出板或重叠，不能仅用相对源 HPWL 的下降百分比证明质量。机械预布局也不等于全板合法初解。

`evaluate` 会重新校验保存的输入哈希、结果和几何；非法结果返回非零退出码。导出目标 `.epro2` 与同名 `.export.json` 都必须是新文件，程序拒绝覆盖源文件和既有评估旁文件。

## 验证范围

支持单顶面、矩形板框、1～2 个主电气域、凸本体或明确来源的保守本体包络、矩形/圆形/椭圆/长圆和显式坐标策略的多边形焊盘、圆孔/槽孔、封装内过孔、固定件、板角机械件、沿边接口、桥接模板、禁区和引脚距离约束。规则仍需反映实际设计。

`source_body_policy: source_courtyard_or_assembly` 可从明确的原生本体层生成包含线宽的保守 AABB，不把普通丝印自动当作本体。`polygon_path_frame: conservative_union` 包络多边形路径的绝对/相对两种解释；它避免低估，也可能占用过多空间。源封装几何原样保留。

活动独立布线/覆铜、无法解释的几何、底面器件、凹板框、多环内孔和特殊层焊盘等仍会拒绝。只对可确认无目标的孤儿 POURED 缓存排除布局建模，原文保留。未完成布线、热/EM 分析和沿表面爬电计算；本项目的二维检查不等于制造验收。

**原生编辑器验收为 PENDING。** 自身解析器读回通过不等于已在立创 EDA 打开、另存或完成原生 DRC。用户实际打开并另存输出文件后，可执行：

```bash
python -m pcb_hierplace verify-native --run runs/real_power_board --project editor_saved.epro2 --opened-and-saved-in-editor
```

只有实际执行过编辑器操作才使用最后一个参数。测试数量、真实板 HPWL 和运行耗时以 [验证报告](docs/TEST_REPORT.md) 及随包实际报告为准。

退出码：0 成功；2 输入/能力/规则问题；3 未找到合法解或评估未通过；4 数值或程序失败。`NO_FEASIBLE_FOUND` 只表示本次候选与预算内没有找到合法解，不表示已证明全局无解。
