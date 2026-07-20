# 交接文档：F32C 云台 + MaixCAM Pro 视觉打靶（台架版，不装车）

> 写给零上下文的新会话。本文档记录任务背景、硬件现状、已完成工作、踩过的坑和下一步计划。
> 日期：2026-07-20

---

## 1. 任务是什么

复现轮趣（WHEELTEC）"25 电赛 E 题 STM32 版"官方例程的**云台视觉打靶**部分：
MaixCAM Pro 识别 A4 靶纸（黑框白纸）圆心 → 串口回传像素误差 → STM32F103C8T6 做视觉 PD 控制 → USART3 总线驱动两个 F32C 无刷云台电机（yaw + pitch）追踪靶心。

**关键约束：不装车。** 纯台架方案，底盘电机/编码器/红外巡线/IMU 全部不接，只做"视觉 + 云台"联动打靶。

**当前状态：已打通全链路，可以打靶。** 剩余问题是精度和响应速度不够，处于调参阶段。

## 2. 工程结构（打靶实际参与的模块加 ★）

```
├── main.py                        ★ MaixCAM Pro 端：YOLOv5 找黑框→透视变换→算圆心误差→UART1 发 8 字节帧
├── USER/
│   ├── MiniBalance.c              ★ 入口：外设初始化、电机上电配置（失能→加速度→模式→使能）、OLED 主循环
│   ├── control.c/.h               ★ 核心：TIM2 5ms 控制循环、视觉 PD、云台输出；底盘 PI（SEARCH 模式下输出强制 0）
│   ├── bluetooth_usart.c/.h       ★ USART2(PA3) + DMA 定长 8 字节收视觉帧 → g_follow_x/y/area, g_data_ready
├── MiniBalance_HARDWARE/
│   ├── BLDC_GIMBAL/DataScope_DP.c ★ F32C 电机协议（USART3/PB10-PB11, 115200）：7A 地址 功能码 数据 BCC 7B
│   ├── H30_IMU/h30_imu.c          ★ 注意：IMU 本身没接，但 TIM2 5ms 定时器在这个文件里初始化，不能删！
│   ├── IR_TRACK / ENCODER / MOTOR / KEY / OLED   （台架打靶不参与或仅调试显示）
├── F32C无刷电机使用手册-2026-07-15.pdf            （电机协议、上位机用法、零点/保存指令）
└── WHEELTEC_F32C云台实现瞄靶使用手册(2026.06.11).pdf （接线表、参数表、调试心得，第 3/4 章最有用）
```

## 3. 硬件接线现状（已验证可工作）

- **MaixCAM Pro：独立 5V 供电**（⚠️ 不再从 STM32 板取电！之前从 STM32 5V 取电导致 MaixCAM 频繁掉电重启）
- MaixCAM **A19 (UART1_TX) → STM32 PA3 (USART2_RX)**，单向，115200 8N1；两板共地
- 云台驱动板：12V 独立电池；驱动板 RX←PB10(USART3_TX)、驱动板 TX→PB11(USART3_RX)；与 STM32 共地
- **电机 1（地址 1）= 底部带导电滑环电机 = YAW 轴**，速度模式（MODE_SPEED），可无限旋转
- **电机 2（地址 2）= 摇臂电机 = PITCH 轴**，单圈位置模式（MODE_SINGLE_POS_L），视觉模块装在它的摇臂上
- ⚠️ 命名易混：**带滑环的是 yaw 不是 pitch**（滑环就是为了 yaw 无限转不绕线）。曾经认反过。

## 4. 已确认的结论（不要重复验证）

1. **视觉↔主控协议完全匹配**：`7B X_H X_L Y_H Y_L V_H V_L 7D`，坐标 ±32768 偏移编码，area 字段仅作有效标志（1/0）。STM32 端 DMA 定长收帧 + IDLE 中断错位自愈。
2. 控制频率 200Hz（TIM2 5ms），视觉约 25fps —— 每 8 个控制周期一帧新数据，中间沿用旧指令。
3. SEARCH 模式（默认）就是台架打靶模式：底盘 PWM 强制 0、**不使用陀螺仪**（use_compensation=0），闭环 = 视觉误差→PD→电机，摄像头装上云台即闭环成立。
4. yaw 轴**不需要也无法校零**（速度模式没有零点概念）；只有 pitch（电机 2）需要校机械零点。
5. 信号链：main.py 算 err_center → UART 8 字节 → DMA → g_follow_x/y → TIM2 每 5ms `Visual_Gimbal_Process()` → PD（无 I 项）→ yaw 发转速 / pitch 发角度增量累加 → USART3 F32C 协议 → 驱动器内部闭环。

## 5. 已做的代码修改（相对官方例程）

| 位置 | 修改 | 原因 |
|---|---|---|
| `USER/control.c:9` | `Control_Work_Enable = 0` → `1` | 没接 PA5 按键，上电直接启动 |
| `USER/control.c:442` | 注释掉 `H30_IMU_TIM_Callback();` | IMU 没接，I2C 超时等待会把 5ms 中断拖爆 |
| `USER/control.c:193` | `Visual_Gimbal_Output(24.0f, 0.0f)` → `(0.0f, 0.0f)` | 停掉上电 360° 寻靶扫描（会绕线）。用户改为手动预对准 |

`Control_Work_Mode` 保持默认 `CONTROL_MODE_SEARCH`，**不要改成 RUN**（RUN 引入 IMU 补偿和巡线路段逻辑）。

## 6. 踩过的坑（绝对不要再踩）

1. **上电顺序**：必须先给 12V 驱动板上电，再给/复位 STM32。使能等初始化指令只在 main() 发一次，电机后上电会永远失能不动。判断：F32C LED 失能常亮、使能快闪。
2. **电机出厂默认地址都是 2**：yaw 电机需用上位机改成地址 1（`7A 02 0D 01 04 7B`），改时总线上只接这一个电机，改完发保存。
3. **零点设置必须跟保存指令**：上位机设零（`7A 02 0A 72 7B`）只写 RAM，**必须再发 `7A 02 08 70 7B` 保存 EEPROM**，否则断电即丢——曾因此出现"第一次联调角度对、下次上电就乱"。校零流程：先装好视觉模块→手动扶正→设零→保存。
4. **MaixCAM 供电**：峰值电流 >1A，不能从 STM32 板 5V 取电，会反复掉电。独立供电 + 共地 + 单根 TX 线即可。改独立供电后，**旧的 5V 连线必须断开**。
5. **MaixCAM 上电不会自动跑 main.py**：默认停在 launcher 菜单 → 无任何串口输出 → 看似"联动失效"。需打包安装 app 并设开机自启（目录下已有 app.yaml/dist），或每次手动在屏幕点开。判断视觉链路是否通：OLED 第 5 行 `F` 值（≈25 正常，0 = 没数据）。
6. "误差稳定但电机一圈圈转不停"**不是 bug**：摄像头没装上云台时反馈环断开，恒定误差→恒定转速。装上即收敛。
7. pitch 上电会动两下（回 0°→抬到 348°）是正常初始化，`control.c:184` 的 `3480`（=348.0°）是待命俯仰角，可按靶位高度改。

## 7. 当前问题与下一步计划（按顺序做）

**现状：能打靶，但精度不够、响应偏慢。**

### 第一步：校准瞄准中心（精度第一要素，先于一切 PID）
锁定靶后看激光点偏差，把 `control.c:459-460` 的 `VISION_CENTER_X/Y`（SEARCH 分支里每周期强制写为 -2/10）改成实测值，让"误差=0"对应"激光正中靶心"。

### 第二步：整定响应（一次只动一个参数，阶跃测试：突然挪靶 20cm）
1. `KP_VISION_X`（0.7）逐步 ×1.3~1.5 上调，到出现回摆振荡退一档；`KP_VISION_Y`（0.25）同理
2. 过冲回摆 → 加 `KD_VISION_X/Y`（0.02 → 0.05 → 0.1）；KD 过大 = 动作发涩、噪声抖
3. 大幅移动时"匀速慢追"= 限幅瓶颈 → 放大 `VISION_YAW_LIMIT_RPM`(30→50~60)、`VISION_PITCH_LIMIT_RPM`(10)、`VISION_PITCH_STEP_LIMIT_X10`(10)
4. 总增益 `vision_segment_gain`（SEARCH 下 0.4，control.c:457）可整体放大，与调 KP 二选一别叠加
5. 驱动器侧：`BLDC_SetAcc(…,100)`（MiniBalance.c:119-121）可提到 200~300；驱动器内部 PID 仅在"指令合理但执行拖沓"时用上位机调（速度环 P 5~50 / I 5~100 从小往大）
6. 免费延迟优化：main.py `model_dual_buff_mode = True → False`（去掉一帧 ≈40ms 延迟，对闭环比帧率更重要）

### 第三步：精度收尾
- `VISION_DEADZONE`（3.0 像素）= 精度下限，可减到 2.0，抖则配 KD
- 视觉坐标噪声大时在 main.py 对 err_center 做 2~3 帧滑动平均（代价：延迟）

### 后续扩展方向（已定位插入点）
| 功能 | 插入点 |
|---|---|
| 锁定后自动开火 | `Visual_Gimbal_Process` 末尾：`vision_lost_ticks==0 && |err|<死区` 持续 N 帧 → GPIO |
| 移动靶提前量 | cmd_yaw 计算处加误差速度外推 |
| 读电机反馈校验 | USART3 RX 已就绪：`BLDC_ReqFeedback` + `BLDC_Motor1/2` 结构体，现无人调用 |
| 上位机实时调参 | 参考 bluetooth_usart.c 中 `#if 0` 关闭的旧调参协议 |

## 8. 观测/调试手段速查

- **OLED**：第 1 行 `M SEA/RUN` + `E0/E1`（模式/启停）；第 4 行 X/Y（视觉误差）；第 5 行 `F`（视觉帧率，≈25 正常）
- **main.py** 每帧 `print(err_center)`
- SEARCH 死区停转判据：err_x ∈ [CENTER_X−3, CENTER_X+3] 且 err_y 同理 → 两电机应停
- 极性错（云台甩离目标而非追）→ 翻 `control.c:24-25` 的 `VISION_DIR_X/Y` 符号（当前均 -1.0）
- 电机指令有没有发出：示波器看 PB10，复位后应有 115200 波形（7A 02 05 7D 7B…）
