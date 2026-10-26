# RobStride05：H743 双电机控制与图形化上位机

STM32H743IIT6（Rev.V）+ FreeRTOS + 两台 RobStride 05，共用 CAN 总线。配套中文上位机、OLED 状态显示和 16 颗 WS2812 灯效控制。

## 当前功能

- 两台电机分别准备、使能、正反转调速、减速停止和失能；可同时持续运行。原电机 CAN ID 127，新增电机 ID 126。
- 上位机显示每台速度、位置、力矩、温度、电流、电压、命令回执及诊断，提供曲线与 CSV 导出。当前控制模式为速度模式，位置/力矩是反馈。
- OLED 同时显示两台状态、目标/实际速度，并显示 RTOS 运行统计。
- WS2812 共 16 颗，7 种灯效，支持颜色、亮度、速度、自定义循环列表、排序与重复、关灯/恢复、浏览器保存/读取方案。循环在 MCU 内执行。
- CPU 480 MHz、AHB 240 MHz、APB 120 MHz，LDO/VOS0；CAN 仍为 1 Mbps，OLED/灯带/RTOS 时基已同步适配。遵循原工程约定，ICache 与 DCache 均关闭。

## 目录

| 路径 | 内容 |
|---|---|
| `RobStride05_H743/` | HAL + FreeRTOS 固件、CubeMX 配置、Keil 工程、测试工具与中文说明 |
| `RobStride05_H743/MDK-ARM/Objects/` | 本次实测固件 AXF / HEX；其它编译中间文件不入库 |
| `上位机/RobStrideConsole.exe` | 可直接双击的 Windows 单文件上位机 |
| `上位机/RobStride_Console/` | Python 服务、浏览器前端、测试与打包脚本 |

本仓库只包含 RTOS 工程与配套上位机。文档中旧版单电机、200 MHz 和 HAL 兄弟工程的记录保留用于追溯；当前行为以第 10–12 篇说明为准。

## 使用

1. 打开 [固件说明](RobStride05_H743/README.md) 和 [双电机接入说明](RobStride05_H743/Docs/10_双电机独立控制与实测.md)，核对接线与两台不同 CAN ID。若电机仍为相同出厂 ID，必须逐台隔离配置，不能同时修改。
2. 用 Keil 打开 `RobStride05_H743/MDK-ARM/RobStride05_H743.uvprojx`，编译并下载；配套 AXF/HEX 已随仓库提供。当前满频启动要求 H743 Rev.V 与匹配的供电条件。
3. 退出 Keil Debug，释放探针。双击 `上位机/RobStrideConsole.exe`，浏览器将打开 `http://127.0.0.1:8765`。
4. 选择探针连接，核对两台身份，再在每台卡片中依次准备 → 使能 → 运行。顶部“全部失能”或 Esc 同时请求两台失能。
5. 灯带控制无需使能电机。详见 [自定义灯效说明](RobStride05_H743/Docs/11_WS2812上位机自定义灯效.md)。

上位机通信路径是 **USB 调试探针 → SWD → H743 → CAN → 电机**，无需串口。连接时核对完整 Flash，自动从相邻 AXF 提取内存布局；单独移动 EXE 则使用打包时的内置固件档案。

运行命令持续有效。仅关闭页面不会停止电机；使用界面停止/失能/断开/退出。当前电机版本拒绝电机侧通信超时参数，因此 CAN 断线或 MCU 停机不能保证自动停机，使用时需保留直接切断电源的手段。

## 构建与验证

开发环境：Windows、PowerShell 7、Python 3.11+、Keil MDK ARMCC 5.06u7、STM32H7xx_DFP（本次为 2.7.0）。主机 C 检查另需 GCC；UI 自动化另需 Playwright 和 Edge；打包另需 PyInstaller。

```powershell
# 在仓库根目录执行
python -m pip install -r ./上位机/RobStride_Console/requirements.txt
pwsh -File ./RobStride05_H743/Tools/build.ps1
pwsh -File ./RobStride05_H743/Tools/test_host.ps1
Push-Location ./上位机/RobStride_Console
python -m unittest discover -s tests -v
# 如需重新打包：python -m pip install pyinstaller
pwsh -File ./build-singlefile.ps1
Pop-Location
```

`Tools/build.ps1` 默认 Keil 位于 `C:\Keil_v5\UV4\UV4.exe`；可用 `-Keil` 和 `-Python` 指定路径。CubeMX 再生成后应执行 `Tools/integrate.py`，恢复已验证的启动与 RTOS 集成；修改初始化时也要更新 `Tools/cubemx_baseline`。

本次已完成 0 错误/0 警告编译、实板烧写、满频寄存器与时间基准核对、双电机正反转和同时 ±5 rad/s、单台独立停止、OLED 帧缓冲回读、7 种灯效与自定义列表回归，以及并行运行检查。最终两台均已失能。没有进行负载、极限速度和长期温升测试；逐颗实际发光未做光学验收。

完整时钟表、测试证据与产物 SHA256 见 [480 MHz 性能配置与实板回归](RobStride05_H743/Docs/12_480MHz性能配置与实板回归.md)。

## 第三方代码

保留 STM32 HAL、CMSIS、FreeRTOS 等组件原有版权与许可证；相关声明见 [Docs/Licenses](RobStride05_H743/Docs/Licenses)。上位机依赖及其许可证随各自项目分发。仓库未对全部代码另行授予统一许可证。
