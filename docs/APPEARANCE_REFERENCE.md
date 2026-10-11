# 实录与官网参考配色、装配显示

当前外观采用用户新实录截图和此前官网图例，未取得或声称使用原厂贴图网格。M5 观察 ID 为 `OBS-M5-20260922-ASTRA-PHOTO-APPEARANCE-01`；GPT-6 Astra xhigh 实际查看实录、旧预览及新 Blender 输出。实录支持银白色手背/掌壳、黑色机械臂主体和连续浅色侧板；它不支持旧预览按掌根 Z 阈值涂黑的分界。详情见 [M5 专项复核](PHOTO_APPEARANCE_REVIEW.md)。

此前官网观察 ID 为 `OBS-M4-20260922-ROOT-APPEARANCE-01`；主会话实际查看了以下两项来源：

- [AIRBOT Play 官网产品图](https://docs.discover-robotics.com/assets/docs-center/play.png)：深色底座/关节机壳，长臂上有白色侧板。
- [OmniHand 官方宣传册](https://www.agibot.com.cn/file/ueditor/php/upload/file/20260522/1779437338739932.pdf)：来自 [O10 下载页](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10)，PDF 第1页（印刷页码22）显示浅色手指/上部掌壳、深色掌根和金属色圆形细节。

配色不确认用户实物的同一型号修订。光照下的官网照片不能给出精确反射率、粗糙度、金属度或全部材料分界。原始参考图、PDF及其SHA256只存私有证据目录，公开仓库只包含来源URL和近似材质配置。

## 可直接使用的接口

`configs/appearance.reference.json` 定义近似线性 RGB、roughness、metallic、连续材质权重和仅用于显示的法线规则。`build_commissioning_preview.py --appearance configs/appearance.reference.json` 将材质写入 USD PreviewSurface。左右 palm 使用单一银白壳材质，删除旧的掌根黑色截断；不再以逐面阈值绑定黑白 `UsdGeomSubset`。

长臂 link2/link3 使用限角角点法向的连续 smoothstep 权重，通过 face-varying UV 和小型线性色带 PNG 在深浅材质间过渡，避免旧逐面二值分区形成三角形锯齿。色带是明确生成的 synthetic 材质工具，不是原厂纹理。材质边界仍为设计近似，不是从原厂 CAD 恢复的区域。自定义转接件和 D405 继续采用通用金属色，尚未确认实物表面处理。

角点法线按 45° 夹角保留锐边，对临时按位置归组的相邻角点做角度加权。归组只用于计算着色法线，不焊接、重网格、裁切或修改源顶点/面；碰撞数据不消费这些显示法线。M5 实际比较全部 54 个 mesh 的点及面索引，逐项与旧版一致。

色带输出在 USD 同目录 `textures/`，`.usda` 和 flattened `.usdc` 都使用相对资产路径；移动预览时应一起复制纹理和 `preview_report.json`。报告含依赖 SHA256。Blender 渲染脚本先核对文件和哈希，导入失败即停止，并将实际图片 pack 到 `.blend`，防止缺图仍继续产出紫色结果。

`blender_review_scene.py --material-mode reference` 保留USD中的机器人参考材质；默认 `diagnostic` 模式仍可将转接件/D405标成橙色/蓝色用于识别。Thor桌的OmniPBR MDL无法原样映射到Cycles，因此两种模式均使用明确记录的灰色审阅材质，源资产不改。

`--symmetry-views` 添加双腕正面和俯视正交审阅图，`--wrist-closeups` 添加左右腕近景；这些是额外审阅相机，不属于训练的三路RGB。比较左右安装时优先看同一张正面图，不以两张不同透视近景的视觉位置代替坐标检查。

## 左右身份与相机滚转

左手来自 `OmniHandleft3.urdf` / `l_palm`，右手来自独立的 `OmniHandright4.urdf` / `R_palm`。profile逐侧绑定URDF SHA256，组合工具拒绝把左手文件误用于右侧，并将实际源哈希及根link写入私有组合身份报告。

两台 D405 使用同一真实源模型，禁止负尺度镜像。当前 synthetic 候选保留右相机绕光轴 180° 的刚体旋转和螺孔基准补偿，并按用户要求各向外移动 20 mm：mount Y 为左 +31 mm、右 -31 mm。右侧原始光学坐标随实体滚转，不做隐式像素翻转；支架和标定仍待解决。当前复核见 [M5 专项记录](PHOTO_APPEARANCE_REVIEW.md)，M4 推导保留在 [对称性记录](WRIST_SYMMETRY_REVIEW.md)。

颜色、静态对称和可视化通过不代表机械承力、PhysX碰撞、任务覆盖或生产采集通过。
