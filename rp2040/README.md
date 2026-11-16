# RP2040 桥接固件

注入链路的 RP2040 侧。

桥接板负责三件事：给相机产生 BootROM 能识别的串行启动波形、在正确的上电窗口发送记录、把相机侧 UART 转成 USB 串口。固件只需烧写一次。

## 1.1 硬件与接线

| 信号         | 引脚   | 说明                      |
| ---------- | ---- | ----------------------- |
| `MULTI_TX` | GP28 | 发往相机的数据；冷启动时同一引脚也用作上电采样 |
| USB 串口     | —    | 与主机通信，固定 115200         |

- 桥接板与相机必须共地。

### 相机侧的引脚定义、详细接线方式不在本文档范围内说明。

## 1.2 固件构成

| 文件                                       | 作用                                                                   |
| ---------------------------------------- | -------------------------------------------------------------------- |
| `firmware/a7c_uart_boot.ino`             | Arduino 固件：3 字节命令帧、carrier 波形、BLOB 发送、`CMD_TIMING` 运行时调波形            |
| `firmware/a7c_serialboot.pio` / `.pio.h` | PIO 程序与生成的 C 头文件（拼帧时序）                                               |
| `firmware/Makefile`                      | 用 `arduino-cli` 构建/烧写（默认 FQBN `rp2040:rp2040:waveshare_rp2040_zero`） |

## 1.3 构建与烧录

```powershell
cd rp2040\firmware
make setup                 # 安装 arduino-cli 的 rp2040 核心（仅首次）
make build
make flash PORT=COM4       # 烧写到桥接板
```

- 依赖 `make` + `arduino-cli`；默认板型 `waveshare_rp2040_zero`，换板子改 `Makefile` 里的 `FQBN`。
- 波形参数由主机每次运行时用 `CMD_TIMING` 下发，固件里的编译期默认值不参与实际注入。

## 1.4 固件自检

烧写后用 `tools/host_control.py` 直接验证桥接板，不涉及相机镜像（命令在仓库根执行）：

```powershell
python tools\host_control.py [-p COM4] <命令>
```

| 命令                             | 作用                                          |
| ------------------------------ | ------------------------------------------- |
| `ping`                         | 握手，确认串口与固件都在跑                               |
| `reset`                        | 复位桥接板                                       |
| `carrier on\|off [--hold [秒]]` | 打开/关闭 carrier；`--hold` 持续刷新（不填值 = 到 Ctrl+C） |
| `blob <hex\|@file>`            | 直接发一段原始记录                                   |
| `baud <rate>`                  | 切换相机侧 UART 波特率，`0` = 关                      |
| `uart <hex\|@file>`            | 透传数据（需 `UART_TX` 固件构建）                      |
| `monitor [--duration]`         | 录制相机侧 UART 输出                               |

> `host_control.py push --cold` 是旧版盲发流程，在 A7C 上相邻 `MEM_WRITE` 会让 ROM 返回 `0x2004`。注入请统一用 `a3push.py --cold`，见 [3. 载荷注入](../tools/README.md#3-载荷注入)。

