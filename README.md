# a7c_fastboot

面向 Sony ILCE-7C（A7C）的一套最小 fastboot 实现，包含设备侧固件源码与配套的主机端命令行工具。

设备侧提供一个 USB fastboot 协议服务：查询版本/产品名，只读访问寄存器与分区表，按分区导出数据，以及在专门的测试构建下把数据原样写回分区。主机侧负责发现设备、发送命令、把数据落盘并做校验。

> 本文档只描述各文件的作用与命令用法。

## 目录结构

```
a7c_fastboot/
├─ build.ps1              构建脚本
├─ payload.ld             链接脚本
├─ include/               头文件
├─ src/                   设备侧源码
└─ tools/                 主机端 Python 工具
```

### 设备侧

| 文件 | 作用 |
|---|---|
| `src/start.S` | 入口与栈/段初始化 |
| `src/main.c` | 主流程：外设初始化、命令循环、灯码 |
| `src/usb_phy.c` | USB PHY 配置 |
| `src/dwc3.c` | USB 控制器驱动（设备模式、端点、传输） |
| `src/usb.c` | USB 描述符、控制传输与 bulk 收发 |
| `src/fastboot.c` | fastboot 命令解析与响应（`getvar` / `oem *` / `download` / `flash` / `upload` 等） |
| `src/storage.c` | 存储侧读通路（CMD17）与写入通路（CMD24，默认关闭） |
| `src/part.c` | SDM2 分区表解析 |
| `src/darwin.c` | 厂商诊断命令 `oem darwin` |
| `src/timer.c` | 时基与延时 |
| `src/led.c` | 灯码输出 |
| `src/a3.c` | 镜像头部校验 |
| `src/libc.c` | 最小 libc 替代（字符串/内存） |
| `include/*.h` | 对应的头文件与寄存器定义 |
| `payload.ld` | 链接布局 |

### 主机侧

| 文件 | 作用 |
|---|---|
| `tools/a7c_fb.py` | 主机端命令行工具，本文档的主要对象 |
| `tools/sdm2_parse.py` | 分区表解析与比对（`parse` / `compare`） |
| `tools/a3image.py` | 把 flat 二进制包装为 `.a3` 镜像，并做自校验 |

## 环境要求

- Windows + PowerShell 5.1
- ARM 交叉工具链 `arm-none-eabi-gcc`（`build.ps1` 顶部的 `$tc` 指向工具链 `bin` 目录，按本机位置修改）
- Python 3.8+，`pyusb`
  ```powershell
  pip install pyusb
  ```
- 设备在系统里需绑定 WinUSB 或 libusbK（可用 Zadig 完成）

## 构建

```powershell
.\build.ps1              # 常规构建 -> out\payload.a3
.\build.ps1 -Clean       # 先清空 out\ 再构建
.\build.ps1 -Selftest    # 只做镜像自校验的最小构建 -> out\selftest.a3
.\build.ps1 -Write       # 带写入通路的测试构建 -> out\payload_w.a3
```

产物都在 `out\` 下：`<name>.a3`（可分发镜像）、`<name>.bin`（flat 镜像）、`<name>.elf` / `.map` / `.dis`（调试用）。

- flat 镜像上限 64 KiB，超出会直接构建失败。
- `-Write` 不是默认构建，产物名固定为 `payload_w.a3`，永远不会覆盖 `payload.a3`。
- 另有若干诊断用构建开关，见 `build.ps1` 头部注释。

## 主机工具 `tools/a7c_fb.py`

```powershell
python tools\a7c_fb.py [--timeout 毫秒] <命令> [参数]
```

`--timeout` 为单条命令的 USB 超时，默认值按命令自动选取。

### 只读命令

| 命令 | 作用 |
|---|---|
| `devices` | 只列举 `18d1:4ee0`，不打开设备 |
| `info` | 打印 version / product / serialno / max-download-size |
| `getvar <name>` | 读取一个 fastboot 变量 |
| `peek <addr[:len]>...` | 只读访问寄存器/内存，多个地址在同一会话内完成 |
| `storage` | 读取存储设备标识信息 |
| `partition` | 读取并打印 SDM2 分区表（每槽一行） |
| `dump <name> [off] [length] -o <file>` | 按分区导出数据 |
| `upload <file>` | 把当前已就绪的数据流导出到文件 |
| `armed` | 报告上一次导出停在哪里（用于区分「流被截断」与「读超时」） |
| `darwin [args]` | 发送厂商诊断命令 `oem darwin` |
| `reboot` | 请求设备结束当前会话并重新枚举 |
| `selftest` | 不需要设备的自检，校验分片边界与握手 |

### 写命令（仅对 `-Write` 构建可用）

| 命令 | 作用 |
|---|---|
| `flash <zone> -f <file>` | 把单个扇区写到白名单分区 |
| `flash-part <zone> -f <file>` | 把整个分区分块写回，仅允许整分区 |

- 白名单与分区大小定义在 `src/fastboot.c` 的 `write_zones[]`；主机侧 `ZONE_BYTES` 必须与之手工对齐。
- `flash-part` 按 16 KiB 分块，块序递增；某块失败即停止，之前的块已写入、之后的块未动。
- 未知分区名、长度不是 512 的整数倍、超过分区大小，这三种情况都会在打开设备之前报错退出，不会向设备发送任何命令。
- 写入后设备侧会立刻读回并逐字节比对，`OKAY` 表示读回内容与发送内容完全一致；`FAIL` 会附带 `w_n / w_err / v_bad / err` 便于定位。

## 常用示例

```powershell
# 发现设备与基本信息
python tools\a7c_fb.py devices
python tools\a7c_fb.py info
python tools\a7c_fb.py getvar product

# 分区表
python tools\a7c_fb.py partition
python tools\a7c_fb.py partition > out\parts.txt
python tools\sdm2_parse.py compare <partinf.conf> --log out\parts.txt

# 导出 2 MiB 分区并校验
python tools\a7c_fb.py dump nflasha23 0 0x200000 -o out\nflasha23.bin
Get-FileHash out\nflasha23.bin -Algorithm SHA256

# 中断后续传
python tools\a7c_fb.py dump nflasha23 0 0x200000 -o out\nflasha23.bin --resume --retries 5

# 原样写回（需要 -Write 构建的镜像）
python tools\a7c_fb.py flash-part nflasha23 -f out\nflasha23.bin

# 不需要设备的自检
python tools\a7c_fb.py selftest
```

## `tools/sdm2_parse.py`

```powershell
python tools\sdm2_parse.py expected <partinf.conf> [--csv <file>]
python tools\sdm2_parse.py capture (--raw <dump.bin> | --log <partition.txt>) [--csv <file>]
python tools\sdm2_parse.py compare <partinf.conf> (--raw <dump.bin> | --log <partition.txt>)
python tools\sdm2_parse.py synth   <partinf.conf> --out <table.bin> [--magic 6428|8246]
```

- `expected`：把分区清单 `partinf.conf` 展开成期望表。
- `capture`：把原始表镜像（`--raw`）或 `a7c_fb.py partition` 的重定向文本（`--log`）解析成表。
- `compare`：期望表与实测表逐项比对，输出 `MATCH` / `MISMATCH`，并对魔术数写法等无害差异降级为 `note:`。
- `synth`：按清单生成一张合成的 2048 字节表镜像，用于离线自测。

## 约定与限制

- `dump` 每次传输 16 KiB，`--slice` 请保持 ≤ 16384；更长的切片不可靠。
- 导出文件必须落在本工具目录内，`-o` 传目录外的路径会被拒绝。
- 设备打开失败（`open()` 报错但 `devices` 能列出）通常是驱动未绑定到 WinUSB/libusbK。
- 设备在传输异常后可能短暂断开再重新出现，`dump` 自带重试与续传；`--retry-wait` 控制重连等待秒数。
- 写命令只在本仓库的 `-Write` 测试构建下存在，常规构建会直接拒绝。

## 免责声明

- 本项目仅供**个人学习、研究与互操作性测试**使用，不提供任何形式的担保（包括但不限于适销性、特定用途适用性与不侵权担保）。
- 使用本项目的一切后果由使用者自行承担。涉及设备读写的操作可能导致数据损坏、功能异常或设备无法正常使用，请务必先自行备份，并在你能接受风险的前提下操作。
- 本项目与 Sony 及其他厂商无任何隶属或授权关系，文中涉及的商标与产品名称归各自所有者所有。
- 请遵守你所在地区的法律法规与你所持设备的保修、许可条款；请勿将本项目用于任何未经授权的用途。
- 任何写入类操作只应在**你拥有或已获得明确授权**的设备上进行。
- 如果你不理解上述风险，请不要使用本项目的写入功能。

## 致谢

- 特别感谢 **ILCE-7M4（7M4）相关开源项目**：本项目在设备侧 fastboot 服务的命令集与整体结构上，很大程度参考并对其做了移植与对齐，没有那份公开工作就不会有这里的起点。
- 感谢 Sony 相机研究社区长期公开分享的分析资料、工具与讨论，本项目的很多思路都来自这些公开资料的启发。
- 感谢 Python 生态中的 `pyusb`、`libusb`，以及 Zadig 等驱动工具的作者与维护者，本项目的主机端交互与设备驱动绑定直接受益于它们。
- 感谢 `arm-none-eabi-gcc` / GNU 工具链的维护者，以及所有把裸机开发经验写成公开文档的人。
- 感谢在讨论区、issue 与邮件中提供过线索、复现与纠错的朋友。

## 范围说明

本仓库只覆盖设备侧 fastboot 服务的实现与主机端交互工具。如何把构建产物交给设备运行不在本文档范围内。
