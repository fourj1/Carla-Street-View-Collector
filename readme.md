# Carla Street View Collector

面向 CARLA 的街景采集工具，提供 GUI + Pygame 控制端，用于批量采集 6 方向全景数据（RGB / 语义分割 / 深度），并支持自动复拍与拼接。

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![CARLA](https://img.shields.io/badge/CARLA-0.9.x-green)
![License](https://img.shields.io/badge/License-MIT-orange)

## 项目特性

- **GUI 管理 CARLA**：启动/关闭服务器、连接状态、同步模式、渲染模式切换。
- **场景控制**：地图切换、天气预设、观测者跟随、速度标签显示。
- **车辆管理**：坐标生成车辆、自动吸附到道路、一键切换自动驾驶、位置瞬移。
- **交通流**：批量生成车辆和行人，并可清除。
- **全景采集**：
  - 6 方向 RGB + 语义分割 + 深度图采集。
  - 自动按距离触发拍摄（默认 10m 间隔）。
  - 全自动采集流程（自动驾驶 -> 采集 -> 复拍 -> 拼接）。
  - 扫描已有数据并自动复拍、拼接。

## 快速开始

### 方式一：直接运行（推荐）
Release 包内置运行环境，解压后运行 `start.bat` 即可打开 GUI。

### 方式二：源码运行
适合二次开发或自定义依赖环境。

```bash
pip install -r requirements.txt
pip install opencv-python
pip install py360convert  # 可选，用于更好的全景拼接
python app/Carla_QtGUI.py
```

## 使用流程（建议）

1. **设置路径**：在 GUI 中选择 `CarlaUE4.exe` 路径；如需 Pygame 采集，请填写本地 `python.exe` 路径。
2. **启动/连接 CARLA**：可在 GUI 中一键启动或手动运行服务端后连接。
3. **生成车辆**：指定坐标或自动吸附到道路生成车辆。
4. **启动采集窗口**：点击 “Pygame 生成车辆” 打开控制窗口。
5. **开始采集**：使用快捷键进行自动/全自动采集与复拍。

## 采集快捷键（Pygame 窗口）

- `Y`：自动拍摄（开源简化版已移除）
- `U`：全自动流程（开源简化版已移除）
- `E`：提前结束自动拍摄并进入复拍（开源简化版已移除）
- `F12`：从已有数据自动复拍并拼接（开源简化版已移除）

如需完整功能，请联系作者：1409151182@qq.com

## 输出结构

默认输出目录为 `panorama_output`，每次拍摄生成一个 `capture_xx_时间戳` 目录：

```
panorama_output/
  capture_001_YYYYMMDD_HHMMSS/
    rgb/
      front.png  back.png  left.png  right.png  top.png  bottom.png
    semseg/
      front.png  back.png  left.png  right.png  top.png  bottom.png
    depth/
      front.png  back.png  left.png  right.png  top.png  bottom.png
    location.txt
    panorama/
      panorama_rgb.png
      panorama_semseg.png
      panorama_depth.png
```

## 配置说明

常用配置保存在 `config.ini`，包括 CARLA 路径、Python 路径、默认生成坐标和备忘录信息。

## 致谢

本项目基于以下开源项目与示例进行二次开发，特此致谢：

- [CARLA Simulator](https://carla.org/) 及其官方示例代码（如 `manual_control.py`、`generate_traffic.py`）
- [CARLA_GUI](https://github.com/9900ff/CARLA_GUI) 的 GUI 设计与交互思路

## 开源协议

本项目采用 [MIT License](LICENSE) 开源协议。
