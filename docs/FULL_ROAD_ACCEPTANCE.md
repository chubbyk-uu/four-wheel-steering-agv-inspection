# 100×10 m全区域采集验收

**状态：2026-09-14已通过。** 门槛定义见[实施计划第6节](MISSION_IMPLEMENTATION_PLAN.md)，逐项实测见[收尾结果](../results/section6_closure.json)。此页只汇总现行结果，不重复维护原始预算。

- ROI为100×10 m、10条扫描道、1 m间距；20 m短测或100 m窄走廊不能替代全区验收。
- 同一新启动WSL实例连续6次全区任务，覆盖0.5/1.0/2.0/2.778 m/s；两次额定速度及多随机种子，全部ACQUIRED、ESTIMATED_COMPLETE、最终HOLD。
- 私有Mesa修复后，六次稳态实时率约0.9984–0.9994；RSS峰值10.17–10.79 GiB，整卡显存9.57–9.65 GiB，同实例未观察到此前的连续衰减。该结论对应这些配置与时长，不是任意平台保证。
- 100 m紧凑道路资产约2.45 GB；早期164.82 GB是展开纹理加多轮数据的方案预算，不是当前运行所需磁盘。
- 通过的是稳定采集，不是拼接、缺陷检测或自主避障验收。后处理按[当前流程](OFFLINE_PROCESSING.md)推进。

恢复道路用[道路资产指南](ROAD_ASSETS.md)，运行任务用[采集指南](RECTANGLE_CAPTURE.md)，私有图形库部署用[Mesa指南](MESA_SETUP.md)。

## 保留的依据

早期展开纹理预算、压缩方案、GPU按需配方、加载/资源试验均保留在[预算与路线历史](archive/maintenance/FULL_ROAD_BUDGET_HISTORY.md)。历史实测不改写成当前参数；相关结构化数据仍在results中。
