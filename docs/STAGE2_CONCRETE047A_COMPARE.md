# Concrete047A 与 Gravel 同布局比较

> 历史实验记录：本文参数、性能和“当前/默认”描述属于当时版本，不作为新版验收。现行550 kg、20 mm/4K/1.5 m、1 m轨迹间距、11 kHz目标见[当前模型基线](LARGE_AGV_REBUILD.md)。历史命令须按现行配置调整，旧16 mm标定不可用于新版；大体积实验资产可能已清理，复现需重新生成。

选材结论：用户已确认采用Concrete047A，设为烘焙默认材质；Gravel样片仅保留作历史对比。

来源：[ambientCG Concrete 047 A](https://ambientcg.com/view?id=Concrete047A)，CC0。通过用户指定代理下载8K-PNG ZIP（1,033,558,405字节），仅提取8192² Color PNG用于本轮烘焙。下载脚本固定提取目标路径，读取时校验ZIP成员CRC，并归档本地SHA256；未声称与官方发布的摘要比对。完整元数据和摘要在assets/road/source/Concrete047A_api.json、Concrete047A_download.json。

官方API的dimensionX/Y/Z均为0，不能据此确定真实物理尺度。本次项目人为设定整张源图覆盖2.1 m，以便与Gravel版保持相同纹素比例（约0.256 mm/源纹素）。后续如果获得实测尺寸，应调整底纹映射并重烘焙；裂缝宽度、板块尺寸和标线尺寸独立定义，不随底纹尺度改变。

两版均为10×10 m，四块5×5 m板，0.25 mm/烘焙纹素，400个2048²核心图块。使用同一AI缺陷原图、同一位置/旋转种子、同一局部预览坐标；底纹接缝布局依据各自图像匹配计算，不要求相同。保留每版独立目录，不覆盖Gravel样片。裂缝主干名义宽0.8—2.0 mm、长2.2/2.4/2.6 m；不是最终像素横截面宽度的独立验收。

## 复现

```bash
python3 tools/fetch_ambient_concrete.py
python3 tools/bake_concrete_road.py --material Concrete047A --output assets/road/baked_concrete047a_new --length 10 --width 10
python3 tools/check_baked_road.py assets/road/baked_concrete047a_new
python3 tools/preview_baked_road.py assets/road/baked_concrete047a_new
```

通过`--material gravel_concrete_03`可恢复另一版（默认已改为Concrete047A）。源文件、映射尺度、来源网址集中在tools/road_materials.py；原始大文件和烘焙图块被gitignore，源码、下载元数据和小预览可入库。

本轮只比较Color底纹加缺陷的烘焙外观，normal/roughness/displacement未用于渲染。不改变平地碰撞面，尚未接入OptiX纹理，也不是线阵相机拍摄结果。下一步接共享场景纹理和缓存，避免把选材样片误当100×10 m全场性能验收。

比较图可用tools/compare_road_materials.py生成；生成前强制比对两版尺寸、缺陷实例、模板、局部像素坐标及局部标签/支持区域，防止布局变化影响选材。

## 本次结果

Concrete047A烘焙耗时243.90秒。400个图块、760处共享边界全部通过，边框最大差0 DN；归档局部/概览逐像素比对通过，缺陷范围外前后背景最大差0 DN。两版缺陷布局、局部坐标、标签及支持区域一致性检查通过。底纹拼接4项单测通过。

[两版对比](concrete_material_comparison.png)；[Concrete047A全景和前后局部](concrete047a_comparison.png)；[Gravel全景和前后局部](gravel_concrete_comparison.png)。结果报告：results/stage2_concrete047a_bake.json及results/stage2_gravel_bake.json。

清理后保留当前Concrete047A源颜色图与baked_concrete047a_v1；原始ZIP和已淘汰两种底纹的大文件已删除。比较小图和验证报告保留，历史下载摘要用于溯源。重新比较Gravel需先恢复其源图并重新烘焙。
