# 主机端工具

`tools/` 下分两组脚本：载荷注入（`a3push.py`、`host_control.py`、`a3image.py`）与 fastboot 交互（`a7c_fb.py`、`sdm2_parse.py`）。以下命令都在仓库根执行。

## 3. 载荷注入

### 3.1 前置条件与原理

- 相机处于断电状态；桥接板已烧写固件并接好线（见 [RP2040 桥接固件](../rp2040/README.md)）。
- 有可用的 `.a3` 镜像（见 [fastboot 载荷构建](../payload/README.md)）。
- BootROM 只在相机上电后的一小段窗口内监听串口启动，因此要么在正确窗口内发送，要么先把 carrier 波形挂在线上、等上电瞬间再整组发送。
- 冷启动判据来自 GP28 的电平采样（发送与采样复用同一引脚），不是计时器。

### 3.2 注入方式

```powershell
python tools\a3push.py [--send] <镜像.a3> [注入方式] [选项]
python tools\a3push.py --probe --send [--cold]
python tools\a3push.py --entry-test --send [--cold]
```

**默认只打印传输计划**（记录条数、每条记录的类型/长度/CRC、入口地址），不打开串口也不发送；加 `--send` 才真正注入。

| 方式 | 参数 | 说明 |
|---|---|---|
| 冷启动（推荐） | `--cold` | 先武装 carrier（每 20 ms 刷新），GP28 采样到第一个高电平即「相机已上电」，再等 0.5 s 把整组发送一次 |
| 流式 | `--stream <秒>` | 整组循环重发，电池可在任意时刻装入，按时间停；结果看设备灯码 |
| 手动 | `--manual` | 回车后等 `--delay` 秒发送一次 |
| 默认 | 无 | 复位 → 武装 carrier → 0.5 s 后单次发送 |

`--cold` 开始前会先检查基线：若采样已经为高（测试板还接着，或相机已上电在驱动数据线），立即中止，一个字节都不发。

### 3.3 常用命令

```powershell
# 只看计划，不碰硬件
python tools\a3push.py out\payload.a3

# 冷启动注入，并等待相机重新枚举
python tools\a3push.py out\payload.a3 --send --cold --expect 18d1:4ee0

# 电池随时装入：循环发送 60 s
python tools\a3push.py out\payload.a3 --send --stream 60

# 只读诊断：ROM 当时有没有在解析记录
python tools\a3push.py --probe --send --cold

# 只读诊断：组闭合与跳转链路是否成立（不跑载荷代码）
python tools\a3push.py --entry-test --send --cold
```

### 3.4 关键参数

| 参数 | 默认 | 作用 |
|---|---|---|
| `-p/--port` | 自动识别 RP2040 | 桥接板串口 |
| `--chunk` | 4086 | 每条 MEM_WRITE 的数据长度；记录上限 4100 B，不能超过 |
| `--delay` | 0.5 | 上电 / 回车到第一条记录的间隔 |
| `--wait` | 900 | `--cold` 等待相机上电的上限（秒） |
| `--expect` | `18d1:4ee0` | 注入成功后应枚举的 VID:PID；`none` 表示不检查 |
| `--q` / `--slow` / `--fast` | 20833 / 12 / 129 | 波形：位周期（ns）/ 慢前导块长度 / 快元胞长度 |
| `--probe` / `--entry-test` | 关 | 不发镜像，只发固定记录做链路诊断 |
| `--crc-init` | `0xFFFFFFFF` | 诊断用：故意让每条记录的 CRC 失效 |

### 3.5 报文与判读

- 记录组固定为 `[SET_UNLOCK, MEM_WRITE×N, SET_ENTRY, SET_UNLOCK]`：首尾必须是同一类型，ROM 只有读到重复的首条类型才认为组结束；结尾缺 `SET_UNLOCK` 会落到 `0x2004`（1 长 4 短）。
- 每条记录最多 4100 B，默认分片 4086 B 刚好用满记录上限。
- 波形默认值是本机验证过的组合：`slow=12` 把 60 边沿前导块放在 250 µs，落在 ROM 的 225.4–293.6 µs 同步窗口内；`slow=11`（229.2 µs）在硬件上会被判过短。
- 注入成功后相机以 `18d1:4ee0` 枚举，接着用 `a7c_fb.py info` / `darwin` 验证（§4）；若相机正常启动而不是枚举，说明波形错过了窗口，重试即可。

### 3.6 灯码与常见失败

| 灯码 | 含义 |
|---|---|
| 1 长 3 短 | 记录 CRC 或位同步问题 |
| 1 长 4 短 | 组未闭合或记录超时 |
| 1 长 5 短（`--entry-test`） | 组闭合与跳转链路均成立 |

## 4. fastboot 主机工具

### 4.1 调用形式与超时

```powershell
python tools\a7c_fb.py [--timeout 毫秒] <命令> [参数]
```

`--timeout` 为单条命令的 USB 超时，默认值按命令自动选取。

### 4.2 只读命令

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
| `darwin [args]` | 发送厂商诊断命令 `oem darwin`，兼作载荷存活/复位观测点（§4.5） |
| `reboot` | 请求设备结束当前会话并重新枚举 |
| `selftest` | 不需要设备的自检，校验分片边界与握手 |

常用组合：

```powershell
# 发现设备与基本信息
python tools\a7c_fb.py devices
python tools\a7c_fb.py info
python tools\a7c_fb.py getvar product

# 分区表
python tools\a7c_fb.py partition
python tools\a7c_fb.py partition > out\parts.txt

# 不需要设备的自检
python tools\a7c_fb.py selftest
```

### 4.3 分区导出与续传

```powershell
# 导出 2 MiB 分区并校验
python tools\a7c_fb.py dump nflasha23 0 0x200000 -o out\nflasha23.bin
Get-FileHash out\nflasha23.bin -Algorithm SHA256

# 中断后续传
python tools\a7c_fb.py dump nflasha23 0 0x200000 -o out\nflasha23.bin --resume --retries 5
```

- `dump` 按「arm 一片 → upload 一片」循环，每片 16 KiB；`--slice` 必须是 512 的倍数且 ≤ 16384（更长的切片在硬件上会断在第 3、4 片），`--single` 退回一次 arm + 一次 upload 的旧路径。
- `--retries`（默认 3）与 `--retry-wait`（默认 3.0 秒）控制断线重连重试；`--resume` 从已有输出文件的末尾继续，而不是截断。
- 设备在传输异常后可能短暂断开再重新出现，`dump` 自带重试与续传。
- 一次导出异常结束时，用 `armed` 看数据流停在哪里：有进度说明是流被截断，无进度说明是读超时。
- `upload <file>` 不带区间，直接把载荷当前已 arm 的那一段导出（arm 状态在载荷内存里，同一次会话内不必重新注入）。
- 导出/读入的文件必须落在仓库目录内（如 `out\`），`-o`/`-f` 传仓库外的路径会被拒绝。

### 4.4 写命令（仅对 `-Write` 构建可用）

| 命令 | 作用 |
|---|---|
| `flash <zone> -f <file>` | 把单个扇区写到白名单分区 |
| `flash-part <zone> -f <file>` | 把整个分区分块写回，仅允许整分区 |

```powershell
# 原样写回（需要 -Write 构建的镜像）
python tools\a7c_fb.py flash-part nflasha23 -f out\nflasha23.bin
```

- 白名单与分区大小定义在 `../payload/src/fastboot.c` 的 `write_zones[]`；主机侧 `ZONE_BYTES` 必须与之手工对齐。
- `flash-part` 按 16 KiB 分块，块序递增；某块失败即停止，之前的块已写入、之后的块未动。
- 未知分区名、长度不是 512 的整数倍、超过分区大小，这三种情况都会在打开设备之前报错退出，不会向设备发送任何命令。
- 写入后设备侧会立刻读回并逐字节比对，`OKAY` 表示读回内容与发送内容完全一致；`FAIL` 会附带 `w_n / w_err / v_bad / err` 便于定位。
- 写命令只在本仓库的 `-Write` 测试构建下存在，常规构建会直接拒绝。

### 4.5 `oem darwin` 与长期存活

`oem darwin` 的用途不只是诊断：在「把设备留在 fastboot 下长期调试」这个场景里，它同时是**存活与复位的观测点**。

- **只读**：它通过 SIO 通道读取 Darwin（系统电源 MCU）的 RAM，不写 MCU、不碰看门狗寄存器、不改 Backup 分区，因此不构成写入风险。
- **实测现象**：设备停在服务模式、不执行任何命令时约 **245 s** 自动复位（SoC 侧硬件看门狗的静态分析值）；进入 fastboot 后执行过一次 `oem darwin` 的会话越过了同一时间点，之后持续在线，最长观测约 **50 分钟**，其间还有一段 605 s 零命令的纯空闲窗口仍存活。
- **判读**：输出里的 `dw_seq` 是命令入口自增的静态计数器，且在**第一笔读之前**打印——递增说明载荷实例没重启（SoC 没复位），回到 `1` 说明载荷重启过，此前关于「存活时长」的结论作废；两行读回值不变说明 MCU 侧看门狗表还在、没被重配。
- **不要当成喂狗**：「必须定期执行 `oem darwin` 保活」已被纯空闲观测证伪，单次执行之后没有周期性依赖。正确用法是：注入后建一次基线，之后只在怀疑复位时再跑一次。
- **冷帧不算故障**：每次链路空闲后的第一笔交换必定失败、第二笔才成功，属 MCU 的应答节奏；`FAIL no reply from darwin (table)` 时先看 `dw_seq` 有没有打印出来，打印了说明载荷还活着、问题在 SIO 链路或引脚复用。

```powershell
# 存活观测（只读，注入后建一次基线即可，不要定时跑）
python tools\a7c_fb.py darwin
```

实现细节、地址来源与全部实测数据见 [oem darwin 实现与效果](file:///e:/coding/sony/oem_darwin_实现与效果_20261027.md)。

### 4.6 分区表对照 `tools/sdm2_parse.py`

离线核对 SDM2 分区表：把固件的 `partinf.conf` 展开成期望表，再把 `oem partition` 的日志或原始表镜像解析出来，两者逐项比对。

```powershell
python tools\sdm2_parse.py expected <partinf.conf> [--csv FILE]
python tools\sdm2_parse.py capture (--raw <dump.bin> | --log <info.txt>) [--csv FILE]
python tools\sdm2_parse.py compare <partinf.conf> (--raw <dump.bin> | --log <info.txt>)
python tools\sdm2_parse.py synth <partinf.conf> --out <table.bin> [--magic 6428|8246]
python tools\sdm2_parse.py selftest
```

- `expected`：按 `partinf.conf` 生成期望表。
- `capture`：解析原始表镜像（`--raw`，2048 字节 = 4 个 512 字节扇区）或 `a7c_fb.py partition` 的重定向文本（`--log`）。
- `compare`：期望表与实测表逐项比对，输出 `MATCH` / `MISMATCH`，并对魔术数写法等无害差异降级为 `note:`。
- `synth`：按清单生成一张合成的 2048 字节表镜像，用于离线自测；`--magic` 选表头写法（默认 `6428`）。
- `selftest`：带内置清单的离线自检，不需要任何输入文件。
- `start` / `count` 按字节偏移解释，脚本额外打印 `start_lba`。

```powershell
python tools\sdm2_parse.py compare <partinf.conf> --log out\parts.txt
```
