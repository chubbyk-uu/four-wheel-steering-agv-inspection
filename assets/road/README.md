# 混凝土道面资产

已选定 **Concrete047A** 为默认底纹。通过 tools/fetch_ambient_concrete.py 下载；烘焙器默认使用该材质。

历史对比底纹Gravel：https://polyhaven.com/a/gravel_concrete_03 ，CC0，作者Charlotte Baglioni。官方8K Diffuse PNG，标注宽2.1 m；8K约0.256 mm/纹素。旧Brushed Concrete 03样片已因方向纹理与重复外观问题停用，保留作历史记录。源图按sRGB解码后转换线性反射率近似，并非实测反射率或完整BRDF。

generated/concrete_cracks_ai_v1.png 和 concrete_spalls_ai_v1.png 是本轮imagegen生成的透明底素材，分别为三种长裂缝和三种小面积剥落，不是现场采集或测量真值。裂缝形态来自生图工具，不使用随机折线等程序生成裂缝轨迹。原始PNG保留不修改。

烘焙器从AI裂缝图提取连通主体及中心线，保留走势与分叉，做物理尺寸校准、反射率转换和重采样。主干名义局部直径限制在0.8—2.0 mm，范围来自图像原始粗细变化经截限，不声称是自然裂缝的统计分布。尖端、分叉并集及最终栅格量化不能用名义直径替代独立宽度测量。三个模板纵向长度2.2/2.4/2.6 m；所有模板保留原始图便于审查。

小面积剥落宽25/35/45 mm，首版仅反射外观，几何深度0，不影响轮胎接地。5×5 m板块、8 mm板缝；白色标线宽0.15 m，中心虚线3 m实段/6 m空段，边线位于距两侧1.5 m处。这是可调整的示例布局，不声明符合任何道路标线规范。中心纵缝与中心线部分重合。

烘焙分辨率0.25 mm/纹素，每块2048²有效区、2纹素真实邻域边框；保存Mono8线性反射率PGM和类别标签PNG。源图细节不放大到5 m：使用2.1 m项目映射尺度，以重叠匹配、最小误差切线和窄羽化拼接底纹，不使用固定2.1 m取模平铺。保存布局与混合蒙版以便复现；低分辨率只用于选择接缝，实际烘焙仍采样8K源图。局部内容复用及仅三种缺陷模板仍不代表大地图唯一定位/拼接验收。

大文件本地生成，不提交仓库。metadata记录来源和校验，generated中保留AI原图。输出schema为agv.road.baked.v1，尚未接入OptiX材质/UV采样，不能传给旧cuda_tiles就声称共享场景已完成。

复现：
- python3 tools/fetch_ambient_concrete.py
- python3 tools/bake_concrete_road.py --output assets/road/baked_concrete047a_new --length 10 --width 10
- python3 tools/check_baked_road.py assets/road/baked_concrete047a_new

输出overview.png是从实际原生瓦片面积平均得到的4 mm/纹素彩色概览；crack_256mm_closeup.png与crack_256mm_before.png是同坐标的256×256 mm加缺陷前后对照，也从瓦片生成时的同一缓冲区提取，不另行渲染。manifest保存像素坐标和米制范围。overview_mono.png和局部Mono8图可与落盘PGM独立逐像素比对。

运行 python3 tools/preview_baked_road.py assets/road/baked_concrete047a_new 得到带位置与尺度说明的comparison.png。四块5×5 m板形成10×10 m样片；全场目标仍100×10 m或更宽。所有预览均为烘焙纹理，不是相机照片。

已选定Concrete047A，见[比较记录](../../../docs/STAGE2_CONCRETE047A_COMPARE.md)。官方物理尺寸未知，暂按2.1 m项目映射；沿用已通过检查的样片布局，后续接入OptiX纹理采样。

目录清理后，仅保留Concrete047A颜色源图、当前baked_concrete047a_v1、AI原图及来源元数据。旧Brushed/Gravel高分辨率源图与烘焙目录、已解压ZIP已删除，小型历史报告和选材预览保留。下载器会校验已有颜色图并直接复用，不因ZIP清理重新下载。需要恢复Gravel时先运行 `python3 tools/fetch_concrete_source.py --asset gravel_concrete_03`，再按历史记录烘焙。
