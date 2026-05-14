# 循迹小车 — MSPM0G3507

MSPM0G3507 八路红外传感器 + L298N 电机驱动循迹小车。

## 控制策略

- **频率**: 100Hz（DelayMs 10ms 主循环）
- **偏差**: 八路线性权重求和（X1=-7 ~ X8=+7）
- **直道**: PD控制，KP=5 KD=10，低KP+高KD防摆头
- **拐弯**: 三阶段状态机——锁定(pivot原地转) → 检测(居中退出) → 增强纠偏

## 硬件接线

| 传感器 | MCU引脚 | 电机驱动 | MCU引脚 |
|--------|---------|----------|---------|
| X1 | PB0 | IN1 | PA0 |
| X2 | PB1 | IN2 | PA1 |
| X3 | PB24 | IN3 | PA28 |
| X4 | PB26 | IN4 | PA31 |
| X5 | PB27 | ENA | PA8 (TIMA0 CCP0) |
| X6 | PB18 | ENB | PA9 (TIMA0 CCP1) |
| X7 | PB19 | | |
| X8 | PB20 | | |

## 编译环境

- CCS Theia + TICLANG 编译器
- MSPM0 SDK 2.10.00.04
- SysConfig 引脚配置

## 调参

| 现象 | 调整 |
|------|------|
| 直道摆头 | 降 KP_STRAIGHT 或升 KD_STRAIGHT |
| 直角弯冲出 | 升 MIN_CORNER_LOCK 或升 CORNER_PIVOT_PWR |
| 直角弯转过头 | 降 MIN_CORNER_LOCK |
| 直角弯误触发 | 升 CORNER_DEBOUNCE |
| 转弯后冲出 | 升 POST_CORNER_KP 或升 POST_CORNER_FRAMES |
