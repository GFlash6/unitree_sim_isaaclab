# 开放词汇语义导航调研总结

> **重点专题：WeDetect / ObjEmbed 与 DiMoS**
> 适用场景：单层仓库、静态物体、单机器人、RGB-D 连续建图、开放词汇物体查询、Nav2 导航
> 当前工程栈：OVO + HMSG + FAST-LIVO2 + Nav2
> 整理日期：2026-08-19

## 1. 执行摘要

本调研的核心问题是：如何把自然语言中的物体目标，稳定地转换为 Nav2 可规划、可到达、朝向合理的导航位姿。调研覆盖语义地图表示、连续多帧融合、开放词汇检索、导航目标生成、坐标对齐、地图生命周期和工程生态。

综合结论如下：

1. **主干表示应采用物体实例级开放词汇地图**，而不是稠密逐点特征地图。OVO 的物体点云 + ID + 语义向量与 GoTo-Object 任务直接匹配，内存和检索成本也更低。
2. **语义地图不应直接返回物体质心作为导航点**。应新增独立的 `semantic_goal_generator`，实现“环形候选点 → costmap 过滤 → `ComputePathToPose` 试探 → 最短可行路径”。这是整个系统最有工程增量的部分。
3. **当前 v1 不宜替换 OVO + SigLIP + Nav2 主栈**。先打通坐标链、查询服务、可达目标生成与拒绝/探索闭环，比过早迁移整体框架更重要。
4. **WeDetect 是 v2 感知前端升级的第一候选**，尤其适合中文类名、仓储长尾物体和“检测即检索”的物体地图范式。但由于 GPL-v3、bbox 不等于 mask、特征空间切换需重建地图，当前更适合先做离线评测和小规模 PoC。
5. **DiMoS 是与当前工程同栈度最高的对照系统，但不应整体迁移**。它已打通 FAST-LIVO2、语义记忆、frontier 探索和 `navigate_with_text`，证明了系统分层思路；但其自研 A* + 局部状态机执行栈弱于 Nav2，且缺少几何可达性防御和量化成功率。

## 2. 推荐系统架构

```text
自然语言指令（例：“去黄色塑料箱”）
  → 文本编码与开放词汇检索
  → FOUND / AMBIGUOUS / NOT_FOUND
  → 物体实例（ID、点云、包围盒、质心、置信度）
  → 环形候选导航位姿（r ≥ 1.0 m，默认 8 方向，朝向物体）
  → costmap 自由空间过滤（建议 cost < 128）
  → Nav2 ComputePathToPose 逐个验证
  → 选择路径最短的可行点
  → NavigateToPose 执行
  → 到达后可选视觉再确认
```

建图侧以 FAST-LIVO2 的 `camera_init` 世界系作为 Nav2 `map` 系；语义点云、占据栅格和物体实例都从同一位姿链生成，避免二次配准。当前采用“在线建图、任务前整体加载、任务中只读”；语义地图与 Nav2 静态地图绑定同一版本号，原子写入和切换。

HMSG 建议裁剪为：

- 保留 Object 层，作为语义检索主体。
- Room 层降级为人工区域层，用“拣货区/存储区/月台”等仓库术语消解同类实例。
- 删除单层场景无用的 Floor 层和为房间投票服务的 View 层。
- 新增 Place/GVD 可达拓扑层，作为物体几何与可导航点之间的中介。

## 3. 总体技术判断

### 3.1 语义地图表示


| 表示              | 优点                                      | 局限                                           | 结论                             |
| ----------------- | ----------------------------------------- | ---------------------------------------------- | -------------------------------- |
| 稠密逐点/栅格特征 | 支持材质、部件和地面等细粒度查询          | 内存大，无稳定实例概念，不利于区分多个同类物体 | 不作为仓库 GoTo-Object 主干      |
| 物体实例地图      | 任务匹配直接，内存小，易存储 ID/几何/语义 | 依赖实例分割、跨帧关联和重复合并质量           | **当前主干，OVO 为最适合的起点** |
| 层级场景图        | 适合大场景、长任务和 LLM 规划             | 多楼层/自动房间分割在单层仓库中收益低          | 只保留轻量区域层和可达拓扑层     |

物体实例地图的优势不仅是内存。HOV-SG 的同基准数据表明，物体/段级表示相比稠密栅格可节省约 75% 内存；OpenMask3D 也表明实例导向特征在长尾类别上显著优于逐点特征。对“去某个箱子/托盘/工作台”这类任务，保存每个点的 512 维语义特征并非必需。

### 3.2 多帧融合和坐标链

- RGB、Depth 与 Pose 必须按图像时刻对齐，位姿应在缓存中对 `t_img` 做 SE(3) 插值，不得直接取“最新位姿”。
- optical frame 与 ROS body frame 的转换应通过 TF 树和标定外参完成，避免手写旋转。
- OVO 的 3D 投影式关联是合理起点；不需要为每个物体单独维护视频 tracker。
- 深度数据建议截断在 4–6 m，物体点云入库前进行 DBSCAN/SOR 去噪、2.5–5 cm 体素下采样和重复投影抑制。
- 观测次数过滤建议为 3–5 帧，以滤除偶发误检和坏视角。
- FAST-LIVO2 无内置回环是高风险项。v1 可用坐标漂移监控和 OVO 式离线重复实例合并兜底；v2 再考虑 PGO。

### 3.3 开放词汇查询

建议查询协议不是无条件 Top-1，而是返回三态：

- `FOUND`：Top-1 相似度达标，且与 Top-2 有足够分差。
- `AMBIGUOUS`：候选得分接近，返回 top-3 由上层追问或用区域词消解。
- `NOT_FOUND`：最高相似度低于阈值，禁止将“最像但不正确”的物体当作目标。

v1 可以使用 SigLIP + prompt ensemble + top-3，查询阈值从 0.20–0.25 起步、Top-1/Top-2 margin 从 0.05 起步，实际值必须用仓库数据校准。中文查询暂用离线中译英；若中文一致性或长尾类召回不达标，再触发 WeDetect PoC。

### 3.4 导航目标生成

物体质心可能在物体内部、货架上、墙后或凹形结构的空腔中，因此不能直接作为导航目标。建议流程为：

1. 在物体点云的可达高度带中估计水平轮廓。
2. 以物体为中心生成半径不小于 1.0 m 的 8 方向环形候选点。
3. 每个候选的 yaw 指向物体。
4. 用 costmap 去掉障碍、致命障碍和未知区域，并考虑机器人半径与膨胀层。
5. 对候选点调用 `ComputePathToPose`，选择可规划且路径最短者。
6. 若当前实例全部失败，尝试 top-2/top-3 语义实例；仍失败则返回 `OBJECT_UNREACHABLE`，不发送必然失败的目标。

### 3.5 超视野目标探索

对 `NOT_FOUND` 不应强行选取低分物体，而应进入“拒绝 → frontier 探索 → 增量观测 → 重查询”闭环。v1 可使用 m-explore-ros2 的暂停/恢复机制；每到达一个新 frontier 就暂停探索，更新语义地图并重查询。v2 再加入类别共现先验和信息增益排序。

## 4. WeDetect / ObjEmbed 专题

### 4.1 定位与家族

[WeDetect](https://github.com/WeChatCV/WeDetect) 由腾讯微信视觉团队提出，论文为 *WeDetect: Fast Open-Vocabulary Object Detection as Retrieval*（arXiv:2512.12309，调研材料记录为 CVPR 2026 接收）。它将开放词汇检测改写为检索问题：文本 embedding 与图像区域特征在共享空间直接点积，删去跨模态融合层。因此视觉特征与当前查询无关，可预先缓存，与物体语义地图的检索形式高度同构。


| 成员                                             | 能力                                     | 在本工程中的候选角色                   |
| ------------------------------------------------ | ---------------------------------------- | -------------------------------------- |
| WeDetect                                         | 中文类名开放词汇检测                     | 替换 2D 检测与区域特征提取             |
| WeDetect-Uni                                     | 无词表 proposal + 类别相关 box embedding | 建图时预提取物体向量，查询时直接点积   |
| WeDetect-Ref                                     | Qwen3-VL 基座的复杂指代理解              | v3 处理“拣货区最左边的黄箱子”        |
| WeDetect-Anything                                | 无 prompt 自动检测所有物体               | 无预设词表的 proposal 来源             |
| [ObjEmbed](https://github.com/WeChatCV/ObjEmbed) | 中英双语物体 embedding + IoU embedding   | Apache-2.0 替代路径，更适合离线/半在线 |

### 4.2 架构与训练特性

WeDetect 的文本塔为 XLM-RoBERTa，视觉侧为专门预训练的 ConvNeXt CLIP 变体，配合 YOLOv6 风格的 CSPRepBiFPAN neck 和 YOLO-World 对比头。识别直接由类名 embedding 与图像网格/区域特征点积完成。

其训练数据强调长尾和多粒度：大规模候选框经 objectness 检测、SAM 高亮后，由视觉语言模型生成层级标签，如“animal / dog / a yellow dog”。这与仓库查询的粒度变化——“箱子 / 黄色箱子 / 黄色塑料箱”——直接对应，是它相比普通 CLIP/SigLIP 区域特征的重要优势。

### 4.3 调研材料中的量化结果


| 模型                    | 参数量 |           FPS | LVIS AP | LVIS 稀有类 AP | COCO AP |
| ----------------------- | -----: | ------------: | ------: | -------------: | ------: |
| WeDetect-Tiny           |    33M |          62.5 |    37.4 |           33.3 |    44.9 |
| WeDetect-Base           |   176M |          35.1 |    47.3 |           43.5 |    52.1 |
| WeDetect-Large          |   490M | 6.0（1280²） |    55.0 |           51.1 |    54.5 |
| Grounding-DINO (Swin-T) |   172M |           6.0 |    27.4 |           18.1 |    48.4 |
| YOLO-World-L            |    48M |          54.6 |    35.4 |           27.6 |    44.9 |

对仓储场景最有意义的不是通用 COCO AP，而是：

- 长尾/稀有类能力强，更有利于托盘、周转箱、液压托盘车等非日常物体。
- Base/Tiny 档具备机载实时潜力，并已有 ONNX 导出路径。
- XLM-R 文本塔支持中文类名直接查询，减少中译英带来的词义偏差。

WeDetect-Uni 更具有结构上的价值：它用 objectness prompt 产生无词表候选框，但保留类别相关的 box embedding。调研材料记录 WeDetect-Base-Uni/Large-Uni 在物体检索中的 COCO F1 分别为 82.5/83.6，明显高于同表中 OpenAI CLIP 的 46.4；工作阈值均为 0.20，与本工程从 0.20–0.25 起步的查询阈值建议相互印证。但不同模型/数据集的相似度不能直接移植，最终仍需仓库数据校准。

### 4.4 它能替换什么，不能替换什么

```text
现有 OVO 前端：RGB → SAM/SAM2.1 mask → CLIP/SigLIP 区域特征
WeDetect 候选：RGB → WeDetect-Uni/Anything bbox + box embedding
                             └→ 可选 SAM box-prompt 补 mask
```


| 环节                        | WeDetect 的替换能力                       | 判断                                                                           |
| --------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------ |
| 2D 候选框                   | 可替换 SAM 的 proposal 角色               | 质量和长尾召回有潜在优势                                                       |
| 2D 语义特征                 | box embedding 可替换 CLIP/SigLIP 区域特征 | 检测与检索共用特征，利于缓存                                                   |
| 中文查询                    | 可直接支持                                | 可绕过离线翻译                                                                 |
| 像素级 mask                 | **不能直接替换**                          | 仅 bbox 时，反投影点云易混入背景；需 box-prompt SAM 补 mask 或重新设计几何过滤 |
| 3D 投影、跨帧关联、实例地图 | 不能替换                                  | 仍由 OVO/自研地图层负责                                                        |
| Nav2 目标生成               | 不能替换                                  | 它是 2D 感知前端，不处理可达性                                                 |

最大的工程误区是把 WeDetect 当成整个 OVO 或语义导航系统的替代品。它实际上只替换感知前端的候选生成和特征提取；坐标投影、多帧融合、物体实例管理和可达导航位姿生成仍需保留。

### 4.5 集成路线与推荐


| 路线           | 做法                                                             | 价值                                   | 代价/风险                                        | 建议                   |
| -------------- | ---------------------------------------------------------------- | -------------------------------------- | ------------------------------------------------ | ---------------------- |
| A：v1 维持     | SAM2.1 + SigLIP，中文离线翻译                                    | 零迁移，OVO 兼容性最好                 | 中文一致性受翻译和特征空间影响                   | **当前主路线**         |
| E：离线评测    | WeDetect-Large 批量给建图图像打中文多粒度标签，人工抽检后制作 GT | 快速建立实验 4 基准，同时预演前端替换  | 运行时无侵入；但许可边界仍需法务确认             | **可立即开始**         |
| D：v2 前端替换 | WeDetect-Uni bbox + embedding，必要时接 SAM box-prompt mask      | 中文原生、长尾强、检索性能高           | GPL-v3；特征空间切换后旧地图不兼容；可能仍需 SAM | **指标不达标时做 PoC** |
| ObjEmbed 备选  | Apache-2.0 的 2B/4B 物体 embedding 模型                          | 许可更友好，支持中英检索和定位质量预测 | 模型重、速度仅适合离线/半在线                    | 商用许可敏感时评估     |

**WeDetect PoC 触发条件**：

- 中文全称/简称/近义词指向同一实例的一致性低于 90%。
- 托盘、周转箱、液压托盘车等长尾类的 Recall@K 显著低于常见类。
- 颜色+材质+类别等多粒度查询经校准仍经常排名翻转。
- 离线 WeDetect 对相同数据的评测明显优于当前 SigLIP 基线。

### 4.6 WeDetect-Ref 与复杂指代

WeDetect-Ref 将 WeDetect-Uni 的 top-100 proposals 压缩为物体 token，用 Qwen3-VL 2B/4B 做候选二分类，而非自回归生成。调研材料记录其 RefCOCO 系平均成绩为 93.2（4B）、速度 5.3 FPS，并支持中英文。它适合将复杂表达拆成“区域/关系约束 + 候选物体检索”，但应放在 v3，不应影响 v1 名词短语查询与可达性闭环的交付。

### 4.7 WeDetect 结论

WeDetect 的战略价值在于：它不是为中文查询外挂一个翻译器，而是把中文文本编码、2D 检测和物体检索统一在同一特征空间。这正是物体实例语义地图所需要的前端形式。然而当前最优策略仍是“先测，后换”：先用它制作 GT 和对比基准，只有当实验证明现有前端成为主要瓶颈时，才切换运行时链路。

## 5. DiMoS 专题

### 5.1 项目定位

[DiMoS](https://github.com/dimensionalOS/dimos)（Dimensional OS）是面向物理空间 Agent 的纯 Python 框架，以 Module / Stream / Blueprint 组织功能，ROS 仅是可选传输层。调研材料记录的项目状态为 Pre-Release Beta v0.0.13，主仓库 Apache-2.0；FAST-LIO2/FAST-LIVO2 原生模块为 GPL-2.0，继承 HKU-MARS 上游许可约束。它的主要能力包括：

- Navigation & Mapping：FAST-LIO2/FAST-LIVO2、体素地图、costmap、A* 与 frontier 探索。
- Perception：2D 检测、3D 投影、VLM 和音频模块。
- Spatial Memory：CLIP 向量检索、命名位置、观测持久化和时空 RAG。
- Agent：MCP、LangGraph 与 `@skill` 能力编排。
- 硬件：Unitree Go2 稳定，G1 为 beta，另有部分机械臂和无人机连接。

它没有论文或导航成功率 benchmark，因此更适合作为工程实现参照，不应被当成已经量化验证的生产解决方案。

### 5.2 完整数据流

```text
FastLio2 / FastLivo
  ├→ odometry / point cloud
  ├→ VoxelGridMapper（Open3D VoxelBlockGrid，CUDA）
  │    └→ CostMapper → 2D global_costmap
  └→ PGO 位姿图优化 → pgo_tf

WavefrontFrontierExplorer
  → goal_request
  → ReplanningAStarPlanner
  → LocalPlanner（初始旋转 / 路径跟随 / 终点旋转）

SpatialMemory / memory2
  → query_tagged_location()
  → NavigationSkillContainer
  → @skill navigate_with_text()
  → goal_request
```

### 5.3 与当前工程的逐环节对照


| 环节     | 当前工程                               | DiMoS                                                           | 判断与借鉴                                       |
| -------- | -------------------------------------- | --------------------------------------------------------------- | ------------------------------------------------ |
| 定位     | FAST-LIVO2（ROS1/桥接）                | `dimos-module-fastlivo`：fake-ROS shim 零修改编译上游，LCM 输出 | ROS2 桥接受阻时的备选实现，但需评估 GPL-2.0      |
| 回环     | FAST-LIVO2 无回环                      | 独立 C++ PGO，输出`pgo_tf`                                      | **v2 最值得借鉴的模块**                          |
| 几何地图 | FAST-LIVO 点云到 PCD/PGM               | GPU 体素地图 + 实时 CostMapper                                  | v1 离线切图足够；column carving 可作动态处理参考 |
| 语义记忆 | OVO 实例点云 + npy 特征 + HMSG         | ChromaDB/SQLite + CLIP + 原始观测                               | 数百实例用 npy 足够，上万实例再考虑向量库        |
| 查询     | SigLIP + top-3 + threshold/margin      | 文本/位置/图像查询，CLIP 余弦                                   | 证实点积检索路线；命名位置可对照 HMSG 人工区域层 |
| 对象跟踪 | OVO 3D 投影式关联                      | 检测 → 3D 投影 → tracker                                      | 机制同源，可借鉴代码组织                         |
| 目标生成 | 拟自研环形候选 + Nav2 路径验证         | `navigate_with_text` 查询记忆/检测后发 `goal_request`           | 架构分层可参考，几何可达性不足                   |
| 探索     | m-explore-ros2 暂停/恢复               | BFS wavefront + 周长过滤 + 信息增益 + timeout                   | 信息增益和超时重评估值得吸收                     |
| 执行     | Nav2 行为树、恢复行为、多 planner 生态 | A* + 局部状态机                                                 | **维持 Nav2，不替换**                            |

### 5.4 `navigate_with_text` 的意义和边界

`NavigationSkillContainer.navigate_with_text(query)` 将 SpatialMemory 语义查找、ObjectTracking 在线检测和导航目标请求组合起来。它修正了“完全没有开源系统打通语义地图到导航目标”的过强表述：

- 学术地图工作如 OVO、ConceptGraphs、HOV-SG 大多停在地图输出或非 Nav2 闭环。
- 工程侧 DiMoS 已在自研执行栈上打通“语义记忆 → 导航目标”。
- 但它的功能主要是 Agent/技能层编排，调研材料未见环形候选点、costmap 预过滤、路径试探、墙体异侧防御或成功率数据。

因此，DiMoS 证明了“查询服务 → 技能编排 → 规划执行”的分层正确，但不能证明目标点在几何上足够可靠。本工程的 `semantic_goal_generator` 仍具有明确的差异化价值。

### 5.5 G1 相关判断

DiMoS 对 Unitree G1 的支持处于 beta，包含 MuJoCo 仿真、G1SimConnection、视频流/相机参数连接与 Pink IK 遥操。若本项目硬件为 G1，这些代码值得作为传感器接入和仿真参考；但 beta 标记和缺少量化数据意味着，不应因为官方演示使用 G1 就直接把它作为生产运行时。

### 5.6 对 DiMoS 的采用建议

**不采用：**

- 不整体迁移到 DiMoS。
- 不用其 A* + 局部状态机替换 Nav2。
- 不在当前数百物体规模下为 ChromaDB 引入额外运维成本。

**优先借鉴：**

1. **PGO 模块**：作为 FAST-LIVO2 无回环的 v2 解法参考，优先研究 `pgo_tf` 如何传播到语义实例和 Nav2 地图。
2. **FAST-LIVO2 fake-ROS shim**：当 ROS1↔ROS2 桥接成为主阻塞时，参考其“上游零修改编译 + LCM 输出”方式，不在 v1 主动重写定位系统。
3. **`navigate_with_text` 分层**：借鉴“语义查询、在线感知确认、目标请求”的技能组合，但底层仍接本工程可达性生成器和 Nav2。
4. **WavefrontFrontierExplorer**：吸收最小 frontier 周长、current/last costmap 信息增益和 goal timeout 重评估机制，改善超视野探索闭环。
5. **命名位置系统**：将 `add_named_location` / `tag_location` 思路映射到 HMSG 的人工区域层，为“拣货区的托盘”提供结构化消解。

### 5.7 DiMoS 结论

DiMoS 应被定位为“持续跟踪的同栈参照系统”，而不是“现在就迁移的目标平台”。它最大的价值是提供了 FAST-LIVO2、PGO、语义记忆、frontier 探索和自然语言导航的同一框架参照；其最大的不足是导航可靠性证据不足，不宜用“演示已打通”替代“几何可达且量化验证”。

## 6. WeDetect 与 DiMoS 的关系

两者处于不同层级，不是相互替代的竞争方案：


| 维度           | WeDetect                                               | DiMoS                                  |
| -------------- | ------------------------------------------------------ | -------------------------------------- |
| 所在层         | 2D 开放词汇感知与物体特征                              | 机器人 Agent/建图/记忆/导航整体框架    |
| 解决的主问题   | 中文和长尾物体“看得见、找得准”                       | 将定位、记忆、探索和自然语言技能串起来 |
| 与 OVO 的关系  | 可替换 OVO 前端的 proposal + embedding，保留 3D 地图层 | 可作 OVO/HMSG/导航分层的工程参照       |
| 与 Nav2 的关系 | 无直接关系                                             | 自研执行栈，不应替换 Nav2              |
| 当前最佳用法   | 离线 GT + 前端 PoC                                     | PGO/桥接/探索/技能编排代码参考         |

理想的未来组合不是二选一，而是：

```text
WeDetect-Uni/Anything（中文、长尾 2D 感知）
  → SAM box-prompt（如需精确 mask）
  → OVO/自研 3D 投影、跨帧关联与实例地图
  → HMSG 物体+人工区域+Place 层
  → 类 DiMoS navigate_with_text 的 Agent 技能编排
  → 本工程 semantic_goal_generator（环形候选+可达验证）
  → Nav2 行为树和恢复机制
```

## 7. 实施路线

### 7.1 阶段划分


| 阶段                    | 工作                                                    | 验收标准                                    |
| ----------------------- | ------------------------------------------------------- | ------------------------------------------- |
| P1：坐标链              | TF 桥接、PCD→PGM、图像时刻位姿插值                     | 同一静态物体在 3+ 观测位置的中心漂移 < 5 cm |
| P2：查询链              | OVO wrapper、实例检索、三态协议、checkpoint 版本化      | 英文查询命中，不存在类正确拒绝              |
| P2.5：WeDetect 离线基准 | 对同一批建图图像打中文多粒度 GT，对比 SigLIP            | 产出中文/长尾类 Recall@K、一致性和错误案例  |
| P3：可达目标            | 环形候选、costmap 过滤、`ComputePathToPose`、top-k 降级 | 可规划率 > 85%，目标位于自由区域比例 > 90%  |
| P4：探索闭环            | m-explore 暂停/恢复、到点增量重查询                     | 移动目标到未建图区后可通过探索找回          |
| V2：按指标触发          | WeDetect 前端 PoC、DiMoS PGO/信息增益借鉴               | 相比 v1 的净收益大于迁移和许可成本          |

### 7.2 四类关键实验

1. **单帧 vs 连续地图**：对比查询相似度、ID 稳定性、中心坐标和重复实例数。
2. **坐标一致性**：从多个机器人位置重复观测同一静态物体；系统性漂移优先排查时间对齐和 optical/body TF。
3. **导航目标可达性**：对比固定 standoff、最近自由格、环形候选、环形+Nav2 试探，重点复现墙边和凹形物体。
4. **开放词汇鲁棒性**：覆盖英文全称/简称、中文、颜色/材质、不存在类；同时跑 SigLIP 和 WeDetect 离线对照。

### 7.3 最低建议指标


| 类别     | 指标                           |           建议目标 |
| -------- | ------------------------------ | -----------------: |
| 检索     | 中英文/近义词 Top-1 实例一致性 |              > 90% |
| 检索     | 不存在类拒绝率                 |               100% |
| 几何     | 同一物体多位置中心漂移         | < 5 cm（健康链路） |
| 目标生成 | 候选位于自由区域               |              > 90% |
| 目标生成 | Nav2 可规划率                  |              > 85% |
| 闭环     | 导航成功率                     |    工程目标 ≥ 75% |
| 闭环     | 停靠距离/朝向误差              | ≤ 1.0 m / ≤ 30° |
| 性能     | 查询到导航启动                 |              < 1 s |

## 8. 风险与决策边界


| 风险                                      | 等级             | 应对                                                     |
| ----------------------------------------- | ---------------- | -------------------------------------------------------- |
| FAST-LIVO2 官方无完整 ROS2 路径           | 高               | 先评估社区 Humble fork/ROS 桥接，受阻再参考 DiMoS shim   |
| 无回环造成长通道漂移                      | 高               | 坐标实验+重建阈值+离线实例合并；v2 评估 DiMoS PGO        |
| 同型号箱体被错误合并                      | 中               | 几何为主、语义为辅，收紧距离与重叠阈值                   |
| 语义检索返回不存在目标的“最像实例”      | 高               | 必须有 threshold + margin + NOT_FOUND，不允许裸 Top-1    |
| WeDetect 仅 bbox 导致点云混入背景         | 中               | 使用 SAM box-prompt 或通过深度/聚类做几何前景分离        |
| WeDetect GPL-v3                           | 中（若进运行时） | 商用前法务评估；可考虑 Apache-2.0 ObjEmbed 或保留 SigLIP |
| DiMoS 展示成熟度被误读为生产可靠性        | 中               | 所有借鉴点必须经本地数据和 Nav2 闭环量化验证             |
| DiMoS FAST-LIO/LIVO 模块 GPL-2.0/上游条款 | 中               | 只读借鉴与代码复用分开处理，复用前法务评估               |

## 9. 最终决策

### 现在做

1. 保持 OVO + HMSG + FAST-LIVO2 + Nav2 主体架构。
2. 优先完成 TF/时间同步、三态检索和 `semantic_goal_generator`。
3. 用 WeDetect-Large 对现有数据离线打标，建立中文、长尾和多粒度查询基准。
4. 跟踪 DiMoS，但只针对 PGO、FAST-LIVO2 shim、frontier 信息增益和 `navigate_with_text` 接口分层做定向研究。

### 达到条件后再做

1. 当实验证明 SigLIP 是中文/长尾召回的主瓶颈时，开始 WeDetect-Uni 前端 PoC。
2. 当长通道的全局漂移超过可接受阈值（建议从 0.3 m 重建触发值评估）时，引入 PGO/回环方案。
3. 当实例数从数百增长到上万、npy 线性检索成为可见瓶颈时，再引入向量库。
4. 当名词短语查询稳定且导航闭环达标后，再用 WeDetect-Ref/ObjEmbed 或 LLM 技能层处理复杂指代。

### 当前不做

1. 不直接迁移到 DiMoS，不替换 Nav2。
2. 不在未有数据对比前把 WeDetect 接入运行时主链。
3. 不为单层静态仓库引入完整多楼层场景图、4D 地图或稠密逐点语义地图。
4. 不用模型展示或仓库 star 数代替实际数据的检索与导航成功率。

## 10. 来源索引

本文是对同目录下 7 份原始调研材料的整合摘要，以总报告和两份补充专题为主：

- `0_开放词汇连续语义地图与Nav2导航目标生成调研报告.md`
- `1_语义地图表示方法.md`
- `2_多帧几何与实例融合.md`
- `3_开放词汇查询与导航目标生成.md`
- `4_GitHub工程生态.md`
- `5_WeDetect与ObjEmbed补充调研.md`
- `6_DiMoS框架补充调研.md`

重点外部入口：

- WeDetect：[https://github.com/WeChatCV/WeDetect](https://github.com/WeChatCV/WeDetect)
- ObjEmbed：[https://github.com/WeChatCV/ObjEmbed](https://github.com/WeChatCV/ObjEmbed)
- DiMoS：[https://github.com/dimensionalOS/dimos](https://github.com/dimensionalOS/dimos)
- DiMoS FAST-LIVO2 模块：[https://github.com/jeff-hykin/dimos-module-fastlivo](https://github.com/jeff-hykin/dimos-module-fastlivo)
- DiMoS FAST-LIO2 模块：[https://github.com/dimensionalOS/dimos-module-fastlio2](https://github.com/dimensionalOS/dimos-module-fastlio2)
- OVO：[https://github.com/tberriel/OVO](https://github.com/tberriel/OVO)
- Nav2：[https://docs.nav2.org](https://docs.nav2.org)

> 注：文中版本、活跃度、许可和性能数据均继承自原始调研材料的记录时点；正式选型或商用前应重新核对上游仓库、论文与许可文件。
>
