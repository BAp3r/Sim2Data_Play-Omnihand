# 官网参考配色与装配显示

此轮使用官网图例指导材质颜色，未取得或声称使用原厂贴图网格。观察 ID 为 `OBS-M4-20260922-ROOT-APPEARANCE-01`；主会话实际查看了以下两项来源：

- [AIRBOT Play 官网产品图](https://docs.discover-robotics.com/assets/docs-center/play.png)：深色底座/关节机壳，长臂上有白色侧板。
- [OmniHand 官方宣传册](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260522/1779437338739932.pdf)：来自 [O10 下载页](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10)，PDF 第1页（印刷页码22）显示浅色手指/上部掌壳、深色掌根和金属色圆形细节。

配色不确认用户实物的同一型号修订。光照下的官网照片不能给出精确反射率、粗糙度、金属度或全部材料分界。原始参考图、PDF及其SHA256只存私有证据目录，公开仓库只包含来源URL和近似材质配置。

## 可直接使用的接口

`configs/appearance.reference.json` 定义近似线性RGB、roughness、metallic和明确的面选择规则。`build_commissioning_preview.py --appearance configs/appearance.reference.json` 将颜色写入 USD PreviewSurface；长臂侧面和掌根通过 `UsdGeomSubset` 绑定第二种材质。所有顶点、面和物理碰撞保持原样，不为颜色重建或裁切几何。

长臂侧板使用局部面法向近似选择，掌根使用局部Z阈值近似分区；这些边界是设计选择，不是从原厂CAD恢复的材料区域。自定义转接件和D405采用通用金属色显示，尚未确认实物表面处理；不生成厂商logo或假纹理。

`blender_review_scene.py --material-mode reference` 保留USD中的机器人参考材质；默认 `diagnostic` 模式仍可将转接件/D405标成橙色/蓝色用于识别。Thor桌的OmniPBR MDL无法原样映射到Cycles，因此两种模式均使用明确记录的灰色审阅材质，源资产不改。

`--symmetry-views` 添加双腕正面和俯视正交审阅图，`--wrist-closeups` 添加左右腕近景；这些是额外审阅相机，不属于训练的三路RGB。比较左右安装时优先看同一张正面图，不以两张不同透视近景的视觉位置代替坐标检查。

## 左右身份与相机滚转

左手来自 `OmniHandleft3.urdf` / `l_palm`，右手来自独立的 `OmniHandright4.urdf` / `R_palm`。profile逐侧绑定URDF SHA256，组合工具拒绝把左手文件误用于右侧，并将实际源哈希及根link写入私有组合身份报告。

两台D405使用同一真实源模型，禁止负尺度镜像。当前synthetic候选通过右相机绕光轴180°的刚体旋转和螺孔基准平移补偿，同时使名义外壳中心与彩色光心对称。右侧原始相机图像坐标也随实体滚转，不做隐式像素翻转；右侧螺孔位于相反朝向，支架和标定必须另行解决。详细几何复核见 `WRIST_SYMMETRY_REVIEW.md`。

颜色、静态对称和可视化通过不代表机械承力、PhysX碰撞、任务覆盖或生产采集通过。
