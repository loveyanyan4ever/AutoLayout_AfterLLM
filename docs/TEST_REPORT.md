# pcb_hierplace 0.2.0 验证报告

本报告由本次发布测试与实际运行 JSON 生成；证据批次为 2026-09-23，真实运行目录为 `real_run_03`。数值取自完成后的 `report.json`、导出旁文件及 `independent_evaluation.json`，未复用上一版测试报告。

## 回归测试与环境

**153 项通过，失败 0、错误 0、跳过 0；耗时 19.767 秒。** 命令：`python -m pytest -q -p no:cacheprovider --junitxml=pytest_release.xml`，退出码 0。实际测试环境为 `Linux-6.18.44-x86_64-with-glibc2.39`。没有在另一台 Ubuntu 主机另做安装验收，以下 Ubuntu 命令提供复现方法。

| 依赖 | 本次版本 |
| --- | --- |
| Python | 3.12.14 |
| numpy | 2.3.5 |
| scipy | 1.17.0 |
| PyYAML | 6.0.3 |
| osqp | 1.1.3 |
| pytest | 9.1.1 |
| shapely | 2.1.2 |

| 修复范围 | 本版处理 |
| --- | --- |
| R01/R02 | 导电本体即使有 pad 仍检查；跨域 pad 必须显式桥接模板。 |
| R03/R04/R05 | 完整稳定 pad 身份；严格 CSV 列结构；拆簇 ID 冲突拒绝。 |
| R06/R07/R08/R09 | 局部允许角、硬引脚距离可行化、紧凑度语义、自交/非法多边形校验。 |
| R10/R11/R12 | C 冻结结构独立验收；CLI 非法结果/坏 YAML 的错误码；工程与旁文件共同保护。 |
| 真实格式适配 | 墓碑、Device 间接封装、同号物理 pad、封装 VIA、铜/孔偏移分离、原生板框及属性跟随。 |
| 用户确认的模型扩展 | 机械 corner/edge_band 预布局；L2 父功能组目标，未扩为六隔离域。 |

## 正常示例与冲突示例

| 示例 | 结果 | 退出码 | 板尺寸 / mm | 最终 HPWL / mm | 最小本体间距 / mm | 最小跨域铜距 / mm | 独立检查数 | 流水线耗时 / s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 固定板框 | FEASIBLE | 0 | 60.000 × 40.000 | 74.702222 | 0.732556 | 5.800000 | 517 | 17.559 |
| 估算板框 | FEASIBLE | 0 | 58.000 × 38.000 | 74.891792 | 0.202000 | 5.800000 | 517 | 11.679 |
| 区域冲突 | NO_FEASIBLE_FOUND | 3 | 固定边框未扩张 | — | — | — | 不生成正式输出 | 2.126 |

两个正常示例都完成新工程导出及独立复核。固定示例保留源板框，估算示例实际写入新板框。冲突示例返回搜索耗尽，没有生成正式 placements/epro2，不能把搜索失败解释为数学不可行证明。示例源布局含违规，不以源线长下降百分比宣称公平的算法改进。

## 真实数字电源板：输入和决策

完整保留 **103 个器件、269 个物理电气对象、47 个有连接网络、25 个实际封装**。269 个对象包括 265 个 PAD 与 4 个封装 VIA；同编号 PAD 保持各自稳定 ID，不能合并计算数量。原始输入没有活动板框，本次估算矩形尺寸并在导出中实际写入板框。

L1 的 13 个逻辑簇和 L2 的 6 个父功能组均保留。固定/特殊机械实体不作为可自由平移簇成员，因此实际可动簇为 **12** 个。按用户确认，电子件属于共地 MAIN 系统；四个机械件属于 MECH，导电类别为 MAIN。L2 使用 `parent_cluster_id` 参与父组紧凑度目标，不被改成六个互相隔离的电气域，也没有声称六个父组有互斥硬区域。

机械孔按明确 corner 模板预布局，接口按 edge_band 放至板边内侧；所有真实器件保持源 0°方向，仅优化坐标。机械孔/接口在后续 B/C 冻结，不将边缘带条件描述为已验证的三维插拔方向。

| 配置项 | 本次值 |
| --- | --- |
| 机械本体间距 / mm | 0.3 |
| 器件间异网铜间距 / mm | 0.25 |
| 几何检查容差 / mm | 1e-06 |
| 求解裕量 / mm | 0.003 |
| 研究预算 / s | 900 |
| 每次 A / B / C 最大迭代 | 40 / 40 / 40 |
| A′/B 最大反馈轮数 | 1 |
| C 信赖半径 / mm | 0.5 |
| C 簇面积增长上限 | 0.05 |
| 种子 | 0 |

规则来源原文：

> User-authorized research defaults: L2 is common-ground functional hierarchy; no certified HV/LV isolation partition supplied. Source layer48 bodies; M3 8x8mm assembly envelope and bare testpoint 0.7x0.7mm access envelope are explicit assumptions. Mechanical gap0.30mm and different-net copper gap0.25mm are research placement defaults, not a standards certification. Corners/edge bands are heuristic placement; connector source angle0 preserved. All real-board components retain their original source angle (0 deg); native editor validation of nonzero rotation mapping has not been performed. This run optimizes positions while preserving source orientations.

上述间距及 M3 8×8 mm 装配包络、裸测试点 0.7×0.7 mm 接近包络是本次明确采用的研究默认。几何依据、保守包络和其他限制见 `NATIVE_FORMAT_NOTES.md`、真实示例配置及证据文件；不能解释成产品安规数值已获认证。

## 真实输出与独立验证

| 指标 | 结果 |
| --- | --- |
| 模型结果 | FEASIBLE |
| 独立几何/日志检查 | PASSED；64735 项检查；0 项违规 |
| 板尺寸 / mm | 84.395837 × 63.296878 |
| 板面积 / mm² | 5341.992941 |
| 最终 HPWL / mm | 1462.180184 |
| 最小本体间距 / mm | 0.303000 |
| 已检查导体对的最小距离 / mm | 0.303000 |
| 最小孔避让距离 / mm | 1.632435 |
| 跨隔离域铜距离 | 不适用：真实板配置只有 MAIN 一个电气域 |
| epro2 读回最大位置误差 / mm | 0.000000000013 |
| 改变位置的器件数 | 103 |
| 流水线记录耗时 / s | 467.949 |
| 总终止原因 | completed |
| 输出网格量化是否采用 | False |
| 原生编辑器打开/另存 | PENDING / NOT_PERFORMED |

流水线的 `elapsed_s` 在写回/独立复核前记录，不代表下载、打包和最终独立验证的总耗时。总终止原因 `completed` 与得到合法布局是两个概念：预算耗尽时保留已找到的合法结果，不能声称每一阶段都完成配置的最大迭代数。


预算对照记录：`real_run_02` 的状态为 `FEASIBLE`，总终止原因为 `budget`，配置预算 600.0 秒，C 实际迭代数为 0。此前合法结果不能证明 C 已执行；本交付采用 `real_run_03`，生成报告前已断言 C 迭代数大于 0。前次运行的预算消耗和阶段耗时保留于其 JSON，不作为本交付质量指标。

## 真实运行的阶段证据

| 阶段 | 耗时 / s | 实际执行证据 |
| --- | --- | --- |
| P0 输入 | 3.671 | 数据解析与规则编译，无梯度迭代 |
| P1 机械预布局 | 0.342 | 机械候选与域区域生成，无梯度迭代 |
| A 簇内 | 113.634 | 见留存候选表；不能据此还原所有被淘汰候选的总步数 |
| 初始 B 簇间 | 42.413 | 拓扑尝试计数 15；2 条记录，步数 40,40；终止 completed×2 |
| 板框缩小搜索 | 116.959 | 流水线仅保存该阶段耗时，未保存逐次求解步数 |
| A′/B 反馈 | 29.598 | 1 轮；1 条记录，步数 40；终止 completed×1 |
| C 微调 | 159.896 | 选解结果 improved；1 条记录，步数 40；终止 completed×1 |

A 阶段仅统计最终写入 `stageA/*.json` 的候选；记录的 `iterations` 包含被拒绝的试探步，不能等同于有效更新次数。A′ 行列出回流 B 的已存记录，局部 A′ 搜索未逐项归档。

| 局部簇 | 留存形状数 | 留存候选的迭代/终止 |
| --- | --- | --- |
| L1-5 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-3 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-1 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-11 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-4 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-12 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-9 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-6 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-8 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-2 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |
| L1-7 | 3 | 2 条记录，步数 40,40；终止 completed×2 |
| L1-10 | 3 | 3 条记录，步数 40,40,40；终止 completed×3 |

| 反馈轮（从 0 开始） | 是否选得改善结果 | 回流 B 记录 |
| --- | --- | --- |
| 0 | True | 1 条记录，步数 40；终止 completed×1 |

## 验证范围与尚未完成的验收

1. 内部验证覆盖所配置的二维几何、板内、机械、域/簇区域、导体净距和冻结状态；独立脚本使用 Shapely/GEOS 重新做距离、相交、包含及原始日志检查，没有调用优化器的内部验收函数。它仍共享原生文件解码器，不能当成完全独立的 EDA 格式实现。
2. 同一封装内部、同一电气域的普通焊盘关系按库内几何处理，不套全局 `default_pad_clearance_mm`；同封装跨隔离域的桥接 pad 仍检查。本次 0.25 mm 是器件间异网铜距规则，并非原生全板 DRC 通过声明。
3. 圆/椭圆/长圆以及歧义多边形采用报告声明的保守外包模型。检查通过指该已解码模型和配置通过，不等于测量恢复全部真实制造几何。
4. 原始日志、非日志 ZIP 成员、对象身份和稳定引脚网络已核对；源工程哈希保持一致，输出另存。源码测试通过及程序读回不能替代立创 EDA 打开/另存、原生 DRC、实际布线或装配验证。
5. 原生编辑器验收 **PENDING**；爬电路径、安规签署、三维机械/插拔、热与电磁验证未完成。HPWL 是线长代理，不是已布通或实际走线长度证明。本次真实文件未验证非零原生旋转映射。

## Ubuntu 复现

在项目根目录使用 Python 3.11+ 创建并启用虚拟环境，执行 `python -m pip install -e '.[test]'`。随后运行四个命令；输出目录必须为空或不存在，配置已启用自动 epro2 导出：

```bash
python -m pcb_hierplace inspect --project examples/real_power_board/source.epro2
python -m pcb_hierplace validate --config examples/real_power_board/constraints.yaml
python -m pcb_hierplace run --config examples/real_power_board/constraints.yaml --out runs/real_power_board
python -m pcb_hierplace evaluate --run runs/real_power_board
```

独立复核（安装测试依赖后）：

```bash
python scripts/independent_evaluation.py --run runs/real_power_board --epro2 runs/real_power_board/board_placed.epro2 --out runs/real_power_board/independent_evaluation.json
```

随包已有真实运行副本位于 `sample_runs/real_power_board/`，可直接将复核命令中的 `--run` 改为此目录，`--epro2` 改为该目录的实际 epro2 文件。重新运行会受到计算机速度和总时间预算影响，未承诺位姿逐位一致。

## 文件身份

- 本次交付工程：`digital_power_board_placed_final.epro2`。
- 输入 SHA-256：`4b98e301328db25bb9a1841967888c26dfa1265824b7d7d4046f535118f8b7b7`。
- 输出 SHA-256：`ca4ae99419d3ba4f4f4f9a2f2909e3d69c2766b73b7196e23ae467bf12da3efa`。
- 证据：`pytest_release_summary.json`、`normal_final_verification.json`、各运行的 `report.json`、`run_manifest.json`、`independent_evaluation.json` 与 `.export.json`。详细算法改动见 `CHANGELOG_0_2.md`。
