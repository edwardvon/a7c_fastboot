# fastboot 载荷构建

设备侧 fastboot 服务的源码、链接脚本与构建脚本。构建产物写到仓库根的 `out\`（已 git 忽略），以下命令都在仓库根执行。

## 2.1 构建命令与产物

```powershell
.\payload\build.ps1              # 常规构建 -> out\payload.a3
.\payload\build.ps1 -Clean       # 先清空 out\ 再构建
.\payload\build.ps1 -Selftest    # 只做镜像自校验的最小构建 -> out\selftest.a3
.\payload\build.ps1 -Write       # 带写入通路的测试构建 -> out\payload_w.a3
```

产物都在仓库根的 `out\` 下：`<name>.a3`（可分发镜像）、`<name>.bin`（flat 镜像）、`<name>.elf` / `.map` / `.dis`（调试用）。

- flat 镜像上限 64 KiB，超出会直接构建失败。
- `-Write` 不是默认构建，产物名固定为 `payload_w.a3`，永远不会覆盖 `payload.a3`。
- 另有若干诊断用构建开关，见 `build.ps1` 头部注释。

## 2.2 产物格式与约束

- `.bin` 是裸 flat 镜像；`.a3` 在开头加 32 字节头（magic、入口偏移、镜像大小、payload CRC、头 CRC），紧跟 32 字节清零 guard，之后才是 payload。BootROM 的串行启动只认这种格式。
- guard 不是填充：BootROM 交接时仍用自己的 SP，入口前的 `push` 会覆盖 `+0x1C` 处的头 CRC，这段清零区就是给它的缓冲。
- [a3image.py](../tools/a3image.py) 可单独做包装与校验：

  ```powershell
  python tools\a3image.py <flat.bin> -o <out.a3> --verify
  ```

  它同时被 `a3push.py` 调用，用于发送前的自检与入口地址解析。
- 注入链路只认 `.a3`；`-Selftest` 产物同样可注入，用于验证链路本身（见 [3. 载荷注入](../tools/README.md#3-载荷注入)）。

## 2.3 设备侧源码地图

编进 payload 的就是这些文件：

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
