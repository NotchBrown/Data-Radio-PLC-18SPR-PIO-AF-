# 上位机 FSK 支持与默认参数参考（新增）

本文档汇总 FSK 调制新增的配置接口、默认参数（供上位机设计默认数值参考）与验证脚本。
协议基础见 `doc/upperpc.md` / `doc/upperpc_cfg.md`。

## 1. 新增地址总览

| 地址 | 读写 | 名称 | 说明 | 存 EEPROM |
|:---:|:---:|------|------|:---:|
| 0x2F | 读写 | **射频调制** | 0=LoRa 1=FSK；写后固件自动重配 SX1278 | ❌ |
| 0x39~0x3E | 读写 | **FSK 包/节点/同步直写** | 映射见 §2 | ❌ |
| 0x60~0x7F | 读写 | **物理参数直写** | 低 5 位 = SX1278 寄存器号(0x00~0x1F) | ❌ |

> LoRa 高层参数（0x30~0x38，存 EEPROM）在 FSK 模式下**复用**：频率(0x30/0x31)、功率(0x35)、前导(0x36)、同步字(0x37，作 FSK 同步字节)、LNA(0x38)。

## 2. FSK 寄存器直写映射（0x39~0x3E）

| UART3 地址 | SX1278 寄存器 | 名称 |
|:---:|:---:|------|
| 0x39 | 0x30 | PACKETCONFIG1（包格式+CRC+地址过滤）|
| 0x3A | 0x31 | PACKETCONFIG2（包模式）|
| 0x3B | 0x32 | PAYLOADLENGTH（载荷长度）|
| 0x3C | 0x33 | NODEADRS（节点地址）|
| 0x3D | 0x34 | BROADCASTADRS（广播地址）|
| 0x3E | 0x28 | SYNCVALUE1（同步字）|

> ⚠️ 注意：FSK 与 LoRa 寄存器地址不同！0x2B~0x2F 是 LoRa FEI/0x23 是 FSK RXDELAY，勿混淆。

## 3. FSK 物理参数直写（0x60~0x7F → SX1278 0x02~0x1F）

| UART3 | SX1278 | 名称 | 默认值(固件宏) |
|:---:|:---:|------|:---:|
| 0x62/0x63 | 0x02/0x03 | BitRate MSB/LSB | 100kbps |
| 0x64/0x65 | 0x04/0x05 | Fdev MSB/LSB | 50kHz |
| 0x69 | 0x09 | PA_CONFIG | 0x8D (13dBm) |
| 0x6C | 0x0C | LNA | 0x23 |
| 0x6D | 0x0D | RXCONFIG | 0x1E (AFCAUTO+AGCAUTO+TrigPreamble)|
| 0x72 | 0x12 | RXBW | 200kHz (0x09) |
| 0x73 | 0x13 | AFCBW | 200kHz (0x09) |

**寄存器换算公式**（供上位机算值）：
- `BitRate寄存器 = 32000000 / BitRate`
- `Fdev寄存器 = Fdev / 61.035`
- `RxBw寄存器`：查表（100kHz→0x0A，200kHz→0x09，250kHz→0x01）

## 4. 默认可跑参数（供上位机默认数值参考）

### A. LoRa（已验证双向联通 ✅）
| 参数 | 值 |
|---|---|
| 频率 | 470MHz |
| SF | 7 |
| BW | 125 / 500 kHz |
| CR | 5 (4/5) |
| 功率 | 13 dBm |
| 前导 | 8 |
| 同步字 | 0x12 |
| LNA | 0x23 |
| 主从 | 1 主 / 0 从（需一台主一台从）|
| 地址 | 本机/对端（如 A=1, B=2）|
| 调制(0x2F) | 0 |

### B. FSK（推荐，极限留 0.5 余地）
| 参数 | 值 |
|---|---|
| 频率 | 470MHz |
| BitRate | 100kbps（极限 ~200kbps 的 0.5）|
| Fdev | ±50kHz |
| RxBw / AfcBw | 200kHz |
| 功率 | 13 dBm |
| 前导 | 8 |
| 同步字节 | 0xC1（与 0x37 同步字一致）|
| PACKETCONFIG1 | 0x90（可变包 + CRC on）|
| PACKETCONFIG2 | 0x40（包模式）|
| PAYLOADLENGTH | 0xFF（可变包最大）|
| 调制(0x2F) | 1 |

## 5. 全量数据内容指示（CI）

`CI = 0xFE` = DH(0x80)+DL(0x40)+A3(0x20)+A2(0x10)+A1(0x08)+A0(0x04)+WORDLEN(0x02)
即 **16 路 DI + 16 路 DO + 4 路 10bit 模拟量**。

主站帧 ≈ 10B：`[地址][定位头][CI₁][CI₂][DI_H][DI_L][A3..A0 4×10bit=5B]`
从站回传 ≈ 8B：`[地址][定位头][CI][DI_H][DI_L][模拟 5B]`

## 6. 上位机推荐 FSK 配置流程

```
1. 写高层(双板一致): 0x16/0x17 地址, 0x19 主从, 0x30/0x31 频率, 0x35 功率, 0x36 前导, 0x37 同步
2. 写任务0: 0x80=0xFE(CI₁), 0x40=0xFE(CI₂), 0x81=1(ENA), 0x82/0x83=周期
3. 0x1E 保存 (高层+任务表)
4. 0x2F=1 切 FSK (固件自动重配)
5. 0x60~0x7F 直写 FSK 物理参数 (BitRate/Fdev/RxBw/PA/LNA/RxConfig)
6. 0x39~0x3E 直写 FSK 包格式 (PACKETCONFIG1/2/PAYLOADLENGTH/SYNCVALUE1)
7. 拨【模式3】开始收发
```

> FSK 直写参数与 0x2F **不存 EEPROM**：重启后 0x2F 回 0（LoRa），FSK 参数回固件默认宏。上位机每次 FSK 会话需重设。

## 7. 验证脚本

| 脚本 | 调制 | 数据内容 | 默认周期 | 更新率 |
|---|---|---|---|---|
| `demo_lora_full.py` | LoRa | 全量 CI=0xFE | 20ms | 50 帧/s |
| `demo_lora_dl.py` | LoRa | 只 DL CI=0x40 | 15ms | ~66 帧/s |
| `demo_fsk_full.py` | FSK | 全量 CI=0xFE | 5ms | 200 帧/s |
| `demo_fsk_dl.py` | FSK | 只 DL CI=0x40 | 2ms | 500 帧/s |

> 全量 = 16DI+16DO+4路10bit模拟量；只 DL = 16路数字输入低8位(DI0~7)。DL 帧更小故可更快。
> 最大更新率评估：模式2 下 0x20 写 0x0001，回复即测试帧耗时 ms；周期 ≥ 2×耗时 即留 0.5 余地。

```bash
python demo_lora_full.py COM_A COM_B --bw 500
python demo_lora_dl.py   COM_A COM_B
python demo_fsk_full.py  COM_A COM_B
python demo_fsk_dl.py    COM_A COM_B
```
每个脚本：写双板配置 + 保存 + 应用/切 FSK → 提示拨模式3 → 双向通联监测（读 A/B RX 计数 + link + CRC 错）。
