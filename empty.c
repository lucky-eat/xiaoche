/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * ============================================================================
 * 循迹小车 — MSPM0G3507 + 八路并行红外传感器 + L298N 电机驱动
 * ============================================================================
 *
 * ============================================================================
 * 一、硬件接线
 * ============================================================================
 *
 * 【电源】
 *   电池正极(7~12V) → L298N +12V 端子
 *   电池负极         → L298N GND + MSPM0 GND (务必共地!)
 *
 * 【八路红外传感器模块 — 10线: VCC/GND/OUT1~OUT8】
 *   模块 VCC  → 3.3V / 5V (按模块规格)
 *   模块 GND  → GND
 *   模块 OUT1 → PB0   (X1, 最左)
 *   模块 OUT2 → PB1   (X2)
 *   模块 OUT3 → PB24  (X3)
 *   模块 OUT4 → PB26  (X4)
 *   模块 OUT5 → PB27  (X5)
 *   模块 OUT6 → PB18  (X6)
 *   模块 OUT7 → PB19  (X7)
 *   模块 OUT8 → PB20  (X8, 最右)
 *
 * 【L298N 电机驱动】
 *   IN1 → PA0  (左电机方向1)    IN3 → PA28 (右电机方向1)
 *   IN2 → PA1  (左电机方向2)    IN4 → PA31 (右电机方向2)
 *   ENA → PA8  (左使能, TIMA0 CCP0 PWM)
 *   ENB → PA9  (右使能, TIMA0 CCP1 PWM)
 *
 *   真值表 (左电机为例):
 *     IN1=1 IN2=0 ENA=PWM → 正转前进
 *     IN1=0 IN2=1 ENA=PWM → 反转后退
 *     IN1=0 IN2=0         → 刹车停止
 *
 * 【调试接口】
 *   PA19 → SWDIO,  PA20 → SWCLK (不可占用)
 *
 * ============================================================================
 * 二、控制策略
 * ============================================================================
 *
 * 传感器分工 (间距密, 线窄只覆盖1~2个探头):
 *
 *      X1    X2    X3    X4    X5    X6    X7    X8
 *    最左                                        最右
 *      |     |     |     |     |     |     |     |
 *      +-----+-----+-----+-----+-----+-----+-----+
 *      ↑                                         ↑
 *   直角弯入口                             直角弯入口
 *      |←─────── X2~X7 直道纠偏 ────────────→|
 *
 * X1/X8 → 直角弯检测 (最外侧, 传感器灵敏一触即发)
 * X2~X7 → 直道姿态纠正 (六路全参与, 低KP+高KD防摆头)
 *
 * 拐弯状态机:
 *   [直道] ──X1或X8触发(1帧)──→ [拐弯]
 *   [拐弯] ──X4/X5居中(1帧)──→ [直道]
 *
 * 直道: 低速(45) + 低KP(5) + 高KD(10) = 温柔纠偏+强力阻尼, 不摆头
 * 拐弯: 低速(30) + 高KP(60) + 丢线锁±7 = 全力PD转向, X4/X5一帧即退
 *
 * ============================================================================
 * 三、调参指南
 * ============================================================================
 *
 * 现象                    | 解决
 * ------------------------|---------------------------
 * 直道来回摆头              | 降 KP_STRAIGHT(-2), 还摆加 KD_STRAIGHT(+2)
 * 直道不居中(偏一边走)      | 升 KP_STRAIGHT(+3)
 * 直角弯拐不过来            | 降 CORNER_SPEED(-10), 或升 KP_CORNER(+20)
 * 直角弯误触发(直道进拐弯)  | 升 CORNER_DEBOUNCE(+1), 或检查传感器是否太灵敏
 * 直角弯过早退出            | 升 CORNER_EXIT_COUNT(+3)
 * 直角弯后冲出去            | 降 STRAIGHT_SPEED, 升 KD_STRAIGHT
 * 轮子转向反了              | 交换 CarMove 里的 ± 号
 * 车不动 / 传感器不认线     | 检查 ACTIVE_LEVEL 是 0 还是 1
 *
 * ============================================================================
 */

#include "ti_msp_dl_config.h"

/* ============================================================================
 * 用户可调参数
 * ============================================================================ */

/* 传感器有效电平: 0=低电平有效, 1=高电平有效 */
#define ACTIVE_LEVEL    1

/* PWM 周期: 32MHz / 32000 = 1kHz, 不动 */
#define PWM_PERIOD      32000

/* ---- 速度 (0 ~ 32000) ---- */
#define STRAIGHT_SPEED  45      /* 直道: 空载低速 */
#define CORNER_SPEED    30      /* 拐弯基准速度, 低速+PD差速 */

/* ---- 直道 PD (温柔纠偏 + 强力阻尼) ---- */
#define KP_STRAIGHT     5.0f    /* 低比例, 不摆头 */
#define KD_STRAIGHT     10.0f   /* 高微分, 踩刹车防过冲 */

/* ---- 拐弯 PD (强力转向) ---- */
#define KP_CORNER       60.0f   /* 高比例, 确保转向 */
#define KD_CORNER       1.0f    /* 轻阻尼, 不拖刹 */

/* ---- 拐弯状态机 ---- */
#define CORNER_DEBOUNCE     1   /* 入口: 1帧即触发 */
#define CORNER_EXIT_COUNT   1   /* 出口: 1帧居中即退出 */

/* ---- 丢线恢复 ---- */
#define LOST_STEP        1      /* 每帧偏差递增量 */
#define LOST_MAX         7      /* 偏差搜索上限 */

/* ============================================================================
 * 引脚宏 — SysConfig 生成, 勿手动修改
 *
 * IR_SENSOR (GPIOB, 8路输入):
 *   X1=PB0  X2=PB1  X3=PB24  X4=PB26
 *   X5=PB27 X6=PB18 X7=PB19 X8=PB20
 *
 * GPIO_MOTOR_DIR (GPIOA, 4路输出):
 *   PIN_5=PA0(IN1)  PIN_6=PA1(IN2)
 *   PIN_7=PA28(IN3) PIN_8=PA31(IN4)
 * ============================================================================ */

#define IR_PORT         IR_SENSOR_PORT
#define IR_PIN_X1       IR_SENSOR_X1_PIN
#define IR_PIN_X2       IR_SENSOR_X2_PIN
#define IR_PIN_X3       IR_SENSOR_X3_PIN
#define IR_PIN_X4       IR_SENSOR_X4_PIN
#define IR_PIN_X5       IR_SENSOR_X5_PIN
#define IR_PIN_X6       IR_SENSOR_X6_PIN
#define IR_PIN_X7       IR_SENSOR_X7_PIN
#define IR_PIN_X8       IR_SENSOR_X8_PIN

#define MOTOR_PORT      GPIO_MOTOR_DIR_PORT
#define MOTOR_IN1       GPIO_MOTOR_DIR_PIN_5_PIN
#define MOTOR_IN2       GPIO_MOTOR_DIR_PIN_6_PIN
#define MOTOR_IN3       GPIO_MOTOR_DIR_PIN_7_PIN
#define MOTOR_IN4       GPIO_MOTOR_DIR_PIN_8_PIN

/* ============================================================================
 * ReadIR — 并行读取八路传感器
 * ============================================================================ */
static void ReadIR(uint8_t ir[8])
{
    ir[0] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X1) != 0) ? 1 : 0;
    ir[1] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X2) != 0) ? 1 : 0;
    ir[2] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X3) != 0) ? 1 : 0;
    ir[3] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X4) != 0) ? 1 : 0;
    ir[4] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X5) != 0) ? 1 : 0;
    ir[5] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X6) != 0) ? 1 : 0;
    ir[6] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X7) != 0) ? 1 : 0;
    ir[7] = (DL_GPIO_readPins(IR_PORT, IR_PIN_X8) != 0) ? 1 : 0;
}

/* ============================================================================
 * CalcError — 线性权重求和
 *
 * X1=-7  X2=-5  X3=-3  X4=-1  X5=+1  X6=+3  X7=+5  X8=+7
 *
 * 负值=线偏左需左转, 正值=线偏右需右转
 * 多路同时触发时权重自动叠加/抵消
 * ============================================================================ */
static int8_t CalcError(uint8_t ir[8])
{
    int8_t err = 0;

    if (ir[0] == ACTIVE_LEVEL) err += -7;
    if (ir[1] == ACTIVE_LEVEL) err += -5;
    if (ir[2] == ACTIVE_LEVEL) err += -3;
    if (ir[3] == ACTIVE_LEVEL) err += -1;
    if (ir[4] == ACTIVE_LEVEL) err +=  1;
    if (ir[5] == ACTIVE_LEVEL) err +=  3;
    if (ir[6] == ACTIVE_LEVEL) err +=  5;
    if (ir[7] == ACTIVE_LEVEL) err +=  7;

    if (err > 7)  err = 7;
    if (err < -7) err = -7;

    return err;
}

/* ============================================================================
 * IsLineLost — 全部传感器看到白底 → 丢线
 * ============================================================================ */
static uint8_t IsLineLost(uint8_t ir[8])
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (ir[i] == ACTIVE_LEVEL) return 0;
    }
    return 1;
}

/* ============================================================================
 * IsCornerEntry — X1或X8检测到黑线 → 直角弯
 * ============================================================================ */
static uint8_t IsCornerEntry(uint8_t ir[8])
{
    return (ir[0] == ACTIVE_LEVEL || ir[7] == ACTIVE_LEVEL);
}

/* ============================================================================
 * IsLineCentered — X4或X5检测到黑线 → 线回到中间
 * ============================================================================ */
static uint8_t IsLineCentered(uint8_t ir[8])
{
    return (ir[3] == ACTIVE_LEVEL || ir[4] == ACTIVE_LEVEL);
}

/* ============================================================================
 * PD_Calc — PD 控制器
 *
 * output = KP * err + KD * (err - err_last)
 *
 * KP/KD 由调用者根据模式传入:
 *   直道: KP_STRAIGHT(10) + KD_STRAIGHT(8) → 温柔+阻尼
 *   拐弯: KP_CORNER(30) + KD_CORNER(3)     → 强力转向
 * ============================================================================ */
static int16_t PD_Calc(int8_t err, float kp, float kd)
{
    static int8_t err_last;
    int16_t output;

    output = (int16_t)(kp * err + kd * (err - err_last));
    err_last = err;

    return output;
}

/* ============================================================================
 * 电机控制
 * ============================================================================ */

static void MotorLeft(int16_t speed)
{
    if (speed > 0) {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_IN1);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_IN2);
    } else if (speed < 0) {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_IN1);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_IN2);
        speed = -speed;
    } else {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_IN1 | MOTOR_IN2);
    }
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    DL_Timer_setCaptureCompareValue(TIMA0, (uint32_t)speed, DL_TIMER_CC_0_INDEX);
}

static void MotorRight(int16_t speed)
{
    if (speed > 0) {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_IN3);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_IN4);
    } else if (speed < 0) {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_IN3);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_IN4);
        speed = -speed;
    } else {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_IN3 | MOTOR_IN4);
    }
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    DL_Timer_setCaptureCompareValue(TIMA0, (uint32_t)speed, DL_TIMER_CC_1_INDEX);
}

/*
 * 实际转弯方向相反时, 交换下面 left/right 的 ± 号。
 */
static void CarMove(int16_t base_speed, int16_t Vz)
{
    int16_t left, right;

    left  = base_speed - Vz;
    right = base_speed + Vz;

    if (left  < 0) left  = 0;
    if (right < 0) right = 0;
    if (left  > PWM_PERIOD) left  = PWM_PERIOD;
    if (right > PWM_PERIOD) right = PWM_PERIOD;

    MotorLeft(left);
    MotorRight(right);
}

/* ============================================================================
 * LineWalk — 主循线逻辑 (每 10ms 调用一次, 100Hz)
 *
 * 拐弯状态机:
 *   [直道] ──X1或X8触发──→ [拐弯]  锁定转向方向, 写死最大差速
 *   [拐弯] ──X4/X5连续N帧居中──→ [直道]
 *
 * 直道: X2~X7全参与, 低KP+高KD, 温柔纠偏防摆头
 * 拐弯: 无PD, 固定最大差速, 死转到底
 * ============================================================================ */
static void LineWalk(void)
{
    uint8_t ir[8];
    int8_t  err;
    int16_t speed, pd_out;

    /* 持久状态 */
    static uint8_t in_corner       = 0;
    static uint8_t corner_enter_cnt = 0;
    static uint8_t corner_exit_cnt  = 0;
    static int8_t  last_err        = 0;
    static uint8_t lost_cnt        = 0;
    static int8_t  corner_dir      = 0;  /* 1=右转, -1=左转 */

    /* 1. 读传感器 */
    ReadIR(ir);

    /* 2. 计算偏差 */
    err = CalcError(ir);

    /* 3. 丢线恢复 */
    if (IsLineLost(ir)) {
        if (in_corner) {
            /* 拐弯中丢线: 沿锁死方向继续, 不等渐进搜索 */
            err = (corner_dir > 0) ? 7 : ((corner_dir < 0) ? -7 : 0);
        } else {
            /* 直道丢线: 沿上次方向渐进搜索 */
            lost_cnt += LOST_STEP;
            if (lost_cnt > LOST_MAX) lost_cnt = LOST_MAX;
            if (last_err > 0)      err =  lost_cnt;
            else if (last_err < 0) err = -lost_cnt;
            else                   err = 0;
        }
    } else {
        lost_cnt = 0;
        last_err = err;
    }

    /* 4. 拐弯状态机 */
    if (!in_corner) {
        if (IsCornerEntry(ir)) {
            corner_enter_cnt++;
            if (corner_enter_cnt >= CORNER_DEBOUNCE) {
                in_corner = 1;
                corner_exit_cnt = 0;
                corner_dir = (err > 0) ? 1 : -1;  /* 锁死转向方向 */
            }
        } else {
            corner_enter_cnt = 0;
        }
    } else {
        if (IsLineCentered(ir)) {
            corner_exit_cnt++;
            if (corner_exit_cnt >= CORNER_EXIT_COUNT) {
                in_corner = 0;
                corner_enter_cnt = 0;
                corner_dir = 0;
            }
        } else {
            corner_exit_cnt = 0;
        }
    }

    /* 5. 速度 + PD差速 */
    if (in_corner) {
        speed  = CORNER_SPEED;
        pd_out = PD_Calc(err, KP_CORNER, KD_CORNER);
    } else {
        speed  = STRAIGHT_SPEED;
        pd_out = PD_Calc(err, KP_STRAIGHT, KD_STRAIGHT);
    }

    /* 6. 限幅后差速驱动 */
    if (pd_out >  speed) pd_out =  speed;
    if (pd_out < -speed) pd_out = -speed;

    CarMove(speed, pd_out);
}

/* ============================================================================
 * PWM_Init — TIMA0 硬件 PWM
 *
 * PA8(CCP0) → 左轮 ENA,  PA9(CCP1) → 右轮 ENB
 * 1kHz, 占空比 0 ~ 32000
 * ============================================================================ */
static void PWM_Init(void)
{
    DL_Timer_reset(TIMA0);
    DL_Timer_enablePower(TIMA0);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM19,
        IOMUX_PINCM19_PF_TIMA0_CCP0);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_8);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM20,
        IOMUX_PINCM20_PF_TIMA0_CCP1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_9);

    DL_Timer_setClockConfig(TIMA0, &(DL_Timer_ClockConfig){
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0
    });

    DL_Timer_initPWMMode(TIMA0, &(DL_Timer_PWMConfig){
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period            = PWM_PERIOD,
        .isTimerWithFourCC = true,
        .startTimer        = DL_TIMER_STOP
    });

    DL_Timer_setCounterControl(TIMA0,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    /* CCP0 左轮 */
    DL_Timer_setCaptureCompareOutCtl(TIMA0,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptCompUpdateMethod(TIMA0,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_0_INDEX);

    /* CCP1 右轮 */
    DL_Timer_setCaptureCompareOutCtl(TIMA0,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptCompUpdateMethod(TIMA0,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMER_CC_1_INDEX);
    DL_Timer_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_1_INDEX);

    DL_Timer_enableClock(TIMA0);
    DL_Timer_setCCPDirection(TIMA0,
        DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_Timer_startCounter(TIMA0);
}

/* ============================================================================
 * DelayMs — 毫秒延时
 * ============================================================================ */
static void DelayMs(uint32_t ms)
{
    delay_cycles(ms * (CPUCLK_FREQ / 1000));
}

/* ============================================================================
 * 主函数 — 100Hz 控制循环
 * ============================================================================ */
int main(void)
{
    SYSCFG_DL_init();
    PWM_Init();
    DelayMs(500);

    DL_GPIO_clearPins(MOTOR_PORT,
        MOTOR_IN1 | MOTOR_IN2 | MOTOR_IN3 | MOTOR_IN4);
    DL_Timer_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(TIMA0, 0, DL_TIMER_CC_1_INDEX);

    while (1) {
        LineWalk();
        DelayMs(10);
    }
}
