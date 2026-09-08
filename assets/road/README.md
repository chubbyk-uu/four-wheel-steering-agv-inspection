# 混凝土道面资产

当前使用 **ambientCG Concrete047A** 颜色＋NormalGL法线，统一粗糙度0.60。源图暂按整张2.1 m项目尺度映射，非官方实测尺度。源图片转换为线性反射率近似，不是实测BRDF。

## 当前资产与恢复

| 本地目录 | 用途 |
|---|---|
| `source/` | Concrete047A颜色、法线、粗糙度源通道及下载/哈希记录 |
| `generated/` | 三张AI缺陷原图和提示词，保留用于复现 |
| `baked_fullwidth_20m_v1/` | 当前20×10 m全宽道路，含视野余量、共享细沟槽及浅碰撞代理 |
| `baked_branch_crack_v2c/` | 旧/新分叉局部对照及精细碰撞诊断，历史夹具 |
| `baked_branch_collision_v1/` | 同源简化碰撞对照，历史夹具 |

高清源图与烘焙产物被Git忽略，不能仅靠克隆获取；源下载记录、AI原图和生成代码保留。恢复当前道路：

```bash
python3 tools/fetch_ambient_concrete.py
python3 tools/fetch_concrete_pbr.py
python3 tools/generate_streaming_road.py --output assets/road/baked_fullwidth_20m_v1 --length 20 --full-width
```

输出目录必须不存在；已有有效资产直接复用。需要代理时使用下载工具的`--proxy`参数。生成20 m全宽道路约13.34 GB无损磁盘纹素，运行使用有界GPU缓存，不将所有高清数据一次装入显存。精细网格、颜色与法线供OptiX采集，GZ使用同源概览与视觉几何。详见[全宽道路与恢复步骤](../../docs/archive/stage2/STAGE2_FULLWIDTH_ROAD.md)。100 m全宽仍待扩展，历史100 m中央扫描走廊不能冒充全宽。

## 缺陷、标线与尺度

5×5 m板块、8 mm板缝；中央双黄实线各宽15 cm、净距15 cm，纵向板缝位于两黄线之间；两侧白实线中心距路边0.5 m。由`tools/road_markings.py`统一定义，具体布局不声明满足某一地区法规。

AI原图提供裂缝形态，不是现场采集或尺寸真值。不使用程序随机折线代替裂缝形态；从同源图提取轮廓，生成颜色/法线与真实沟槽。细长分叉主干名义宽0.8–2.0 mm，尖端、分叉并集和最终像素宽度需单独测量。20 m道路使用已确认的分叉版本，仍只有少量模板复用，不代表缺陷训练集足够多样。

当前烘焙0.25 mm/纹素、2048核心＋每边2像素邻域边框。GZ/OptiX共享成像几何；稀疏浅沟槽可使用显式有界碰撞代理，不能将较深坑洞、台阶或坡面任意填平。参考[分叉裂缝](../../docs/archive/stage2/STAGE2_BRANCH_CRACK.md)、[碰撞代理](../../docs/issues/COLLISION_PROXY.md)。

## 历史与清理

Brushed/Gravel已停用，保留小型选材报告；其源图和大体积烘焙已删除。`baked_concrete047a_v1`、`baked_pbr_probe_v1`、`baked_shared_road_v1`、`baked_streaming_road_v1`也是历史生成目录，当前本地不再保留。相应文档中的名称是复现输出名，不是现成入口。历史虚线方案也已替换为双黄实线。

完整采集不提交Git，保留与清理范围见[维护审计](../../docs/archive/integration/MAINTENANCE_AUDIT.md)。旧65 kg/16 mm档案及平场仅作历史回归，不能套用当前550 kg/20 mm/v7配置。
