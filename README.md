# a7c_fastboot

面向 Sony ILCE-7C（A7C）的一套最小 fastboot 实现，包含设备侧固件源码、载荷注入链路与配套的主机端命令行工具。

设备侧提供一个 USB fastboot 协议服务：查询版本/产品名，只读访问寄存器与分区表，按分区导出数据，以及在专门的测试构建下把数据原样写回分区。设备启动前，镜像经一块 RP2040 桥接板按 BootROM 的串行启动协议推入并跳转；主机侧负责发现设备、发送命令、把数据落盘并做校验。

> 本文件是总览与导航。各子项目按实际动手顺序拆分：搭注入链路 → 构建载荷 → 注入 → 用主机工具。

## 目录结构

```
a7c_fastboot/
├─ rp2040/                注入链路（RP2040 侧）
│  ├─ README.md           接线、构建、烧录、自检
│  └─ firmware/           UART 桥接固件：.ino / .pio / .pio.h / Makefile
├─ payload/               fastboot 载荷（设备侧）
│  ├─ README.md           构建、镜像格式、源码地图
│  ├─ build.ps1           构建脚本
│  ├─ payload.ld          链接脚本
│  ├─ include/            头文件
│  └─ src/                设备侧源码
├─ tools/                 主机端工具：注入 + fastboot 交互
│  └─ README.md           载荷注入、fastboot 命令、分区导出、`oem darwin`
└─ out/                   构建产物与导出数据（git 忽略）
```

- [rp2040/README.md](rp2040/README.md)：桥接板接线（1.1）、固件构成（1.2）、构建烧录（1.3）、固件自检（1.4）
- [payload/README.md](payload/README.md)：构建命令与产物（2.1）、镜像格式（2.2）、源码地图（2.3）
- [tools/README.md](tools/README.md)：载荷注入（3）、fastboot 主机工具（4）

## 环境要求

### 载荷构建

- Windows + PowerShell 5.1
- ARM 交叉工具链 `arm-none-eabi-gcc`（[payload/build.ps1](payload/build.ps1) 顶部的 `$tc` 指向工具链 `bin` 目录，按本机位置修改）

### 载荷注入

- 一块 RP2040 桥接板，按 [§1.1](rp2040/README.md#11-硬件与接线) 接线
- `make` + `arduino-cli`（仅烧写桥接固件时需要）
- `pyserial`（`a3push.py` 与 `host_control.py` 都依赖它，只打印计划时也需要）
  ```powershell
  pip install pyserial
  ```

### 主机工具

- Python 3.8+，`pyusb`
  ```powershell
  pip install pyusb
  ```
- 设备在系统里需绑定 WinUSB 或 libusbK（可用 Zadig 完成）

## 快速开始

四步跑通整条链路（都在仓库根执行），细节见对应文档。

```powershell
# 1. 烧写桥接固件（仅首次）
cd rp2040\firmware; make setup; make flash PORT=COM4; cd ..\..   # rp2040 §1.3

# 2. 构建载荷
.\payload\build.ps1                               # -> out\payload.a3，payload §2.1

# 3. 注入：相机断电，执行后给相机上电
python tools\a3push.py out\payload.a3 --send --cold --expect 18d1:4ee0   # tools §3.3

# 4. 验证载荷是否在跑
python tools\a7c_fb.py info                       # tools §4.2
```

前提：桥接板已按 [rp2040 §1.1](rp2040/README.md#11-硬件与接线) 接线，相机 USB 已绑定 WinUSB/libusbK（见"环境要求"）。

## 常见问题

| 现象 | 处理 |
|---|---|
| `devices` 能列出，`open()` 报错 | 驱动未绑定到 WinUSB/libusbK，用 Zadig 重新绑定 |
| `dump` 中途设备消失 | 加 `--resume --retries N` 续传，用 `armed` 判断是流被截断还是读超时（[tools §4.3](tools/README.md#43-分区导出与续传)） |
| 注入后相机正常启动、没有枚举 | 波形错过了上电窗口，重试；先用 `--probe` 确认 ROM 当时在解析记录（[tools §3.3](tools/README.md#33-常用命令)） |
| `--cold` 直接中止，提示基线为高 | 相机已上电或采样线被拉高；断电、断开测试板后重试 |
| `darwin` 返回 `FAIL no reply from darwin (table)` | 看 `dw_seq` 是否已打印：已打印说明载荷还活着，问题在 SIO 链路；链路静默时复位设备即可恢复（[tools §4.5](tools/README.md#45-oem-darwin-与长期存活)） |
| `ModuleNotFoundError: No module named 'serial'` | `pip install pyserial` |
| `-o` 指定的路径被拒绝 | 导出/读入文件必须落在仓库目录内（如 `out\`） |
| 写命令被直接拒绝 | 当前镜像不是 `-Write` 构建（[payload §2.1](payload/README.md#21-构建命令与产物)） |

## 约定与限制

- 只针对 ILCE-7C（`18d1:4ee0`）：其他机型的分区表、寄存器与 GPIO/SIO 参数都不同，不能直接套用。
- 写命令只存在于 `-Write` 测试构建（[payload §2.1](payload/README.md#21-构建命令与产物)），常规构建一律拒绝。
- 导出/读入的文件必须落在仓库目录内（如 `out\`），`-o`/`-f` 传仓库外的路径会被拒绝。
- `--slice` 上限、断线重试与续传行为见 [tools §4.3](tools/README.md#43-分区导出与续传)；导出后请自行校验（如 `Get-FileHash`），不要只看命令是否返回 `OKAY`。

## 免责声明

- 本项目仅供**个人学习、研究与互操作性测试**使用，不提供任何形式的担保（包括但不限于适销性、特定用途适用性与不侵权担保）。
- 使用本项目的一切后果由使用者自行承担。涉及设备读写的操作可能导致数据损坏、功能异常或设备无法正常使用，请务必先自行备份，并在你能接受风险的前提下操作。
- 本项目与 Sony 及其他厂商无任何隶属或授权关系，文中涉及的商标与产品名称归各自所有者所有。
- 请遵守你所在地区的法律法规与你所持设备的保修、许可条款；请勿将本项目用于任何未经授权的用途。
- 任何写入类操作只应在**你拥有或已获得明确授权**的设备上进行。
- 如果你不理解上述风险，请不要使用本项目的写入功能。

## 致谢

- 特别感谢 **ILCE-7M4（7M4）相关开源项目**：本项目在设备侧 fastboot 服务的命令集与整体结构上，很大程度参考并对其做了移植与对齐，没有那份公开工作就不会有这里的起点。
  - <https://github.com/DavidBuchanan314/ILCE-7M4-RE>
- 感谢 **sony-a6700-powerful** 项目公开的思路与实现，为本项目提供了有价值的参考。
  - <https://github.com/xyzhangxin/sony-a6700-powerful>
- 感谢 Sony 相机研究社区长期公开分享的分析资料、工具与讨论，本项目的很多思路都来自这些公开资料的启发。
- 感谢 Python 生态中的 `pyusb`、`libusb`，以及 Zadig 等驱动工具的作者与维护者，本项目的主机端交互与设备驱动绑定直接受益于它们。
- 感谢 `arm-none-eabi-gcc` / GNU 工具链的维护者，以及所有把裸机开发经验写成公开文档的人。
- 感谢在讨论区、issue 与邮件中提供过线索、复现与纠错的朋友。

## 范围说明

本仓库覆盖设备侧 fastboot 服务、主机端交互工具、载荷注入脚本与 RP2040 桥接固件源码。注入需要自备 RP2040 板并按 [rp2040 §1.1](rp2040/README.md#11-硬件与接线) 接线；桥接板的选型、焊接与供电不在本文档范围内。
