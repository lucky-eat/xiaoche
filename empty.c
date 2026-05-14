/*
 * 循迹小车 — MSPM0G3507 + 八路并行红外传感器 + L298N 电机驱动
 *
 * 基于 TI MSPM0 SDK empty 模板，CCS Theia + TICLANG 编译
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
 * 传感器分工:
 *
 *      X1    X2    X3    X4    X5    X6    X7    X8
 *    最左                                        最右
 *      |     |     |     |     |     |     |     |
 *      +-----+-----+-----+-----+-----+-----+-----+
 *      ↑                                         ↑
 *   直角弯入口                             直角弯入口
 *      |←─────── X2~X7 直道纠偏 ────────────→|
 *
 * X1/X8 → 直角弯检测 (最外侧)
 * X2~X7 → 直道姿态纠正 (六路全参与, 低KP+高KD防摆头)
 *
 * 拐弯状态机 (三阶段):
 *
 *   [直道] ──X1或X8触发──→ [拐弯-锁定]
 *                              │ 前 MIN_CORNER_LOCK 帧:
 *                              │  · 内侧轮反转, 外侧轮正转 (原地pivot)
 *                              │  · 完全不检测退出条件
 *                              │  · 不响应传感器
 *                              ↓
 *                          [拐弯-检测]
 *                              │ 到达锁定帧数后:
 *                              │  · 继续pivot转弯
 *                              │  · 检测连续N帧 X4/X5居中
 *                              ↓
 *                          [拐弯-退出]
 *                              │ X4/X5连续居中N帧 → 退出
 *                              │ 进入 POST_CORNER 增强纠偏
 *                              ↓
 *                          [直道]
 *
 * 关键改进:
 *   1. 最小锁定帧数: 确保车体至少转了足够角度才允许退出
 *   2. Pivot原地转: 内侧轮反转, 不再只是"一边停一边走"
 *   3. 退出检测延迟: 锁定结束后才开始检测居中, 避免过早退出
 *   4. 退出后增强PD: 残余偏差用高KP快速纠正, 防止冲出赛道
 *
 * ============================================================================
 * 三、调参指南
 * ============================================================================
 *
 * 现象                    | 解决
 * ------------------------|---------------------------
 * 直道来回摆头              | 降 KP_STRAIGHT(-2), 还摆加 KD_STRAIGHT(+2)
 * 直道不居中(偏一边走)      | 升 KP_STRAIGHT(+3)
 * 直角弯拐不过来(冲出)      | 升 MIN_CORNER_LOCK(+10), 或升 CORNER_PIVOT_PWR(+5)
 * 直角弯转过头(转多了)      | 降 MIN_CORNER_LOCK(-5)
 * 直角弯不退出(原地打转)    | 降 CORNER_MAX_FRAMES(-20), 降 CORNER_EXIT_COUNT(-2)
 * 直角弯误触发(直道进拐弯)  | 升 CORNER_DEBOUNCE(+1)
 * 直角弯后冲出去            | 升 POST_CORNER_KP(+10), 或升 POST_CORNER_FRAMES(+5)
 * 轮子转向反了              | 交换 PivotTurn 里的正反转
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
#define STRAIGHT_SPEED  45      /* 直道: 基础速度 */
#define CORNER_SPEED    35      /* 拐弯外侧轮前进速度 */
#define CORNER_PIVOT_PWR 25     /* 拐弯内侧轮反转速度 (pivot原地转) */

/* ---- 直道 PD (温柔纠偏 + 强力阻尼) ---- */
#define KP_STRAIGHT     5.0f    /* 低比例, 不摆头 */
#define KD_STRAIGHT     10.0f   /* 高微分, 踩刹车防过冲 */

/* ---- 拐弯 (不要PD, 用固定pivot差速) ---- */

/* ---- 拐弯状态机 ---- */
#define CORNER_DEBOUNCE     2   /* 入口: 连续2帧确认, 防误触发 */
#define MIN_CORNER_LOCK    25   /* 锁定帧数: 至少转250ms才允许退出 */
#define CORNER_EXIT_COUNT   6   /* 出口: 连续6帧居中确认 */
#define CORNER_MAX_FRAMES  80   /* 安全超时: 80帧未退出则强制结束 (800ms) */
#define POST_CORNER_FRAMES 12   /* 退出后增强纠偏帧数 */
#define POST_CORNER_KP   20.0f  /* 退出后临时KP, 快速归中线 */

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
 * PD_Calc — PD 控制器 (仅直道和退出后增强阶段使用)
 *
 * 拐弯阶段不调用 PD_Calc, err_last 保持旧值, 切换回 PD 模式时
 * 会产生 derivative 尖峰。在进入需要 PD 的阶段前调用 PD_Reset()。
 * ============================================================================ */
static int8_t pd_err_last;

static int16_t PD_Calc(int8_t err, float kp, float kd)
{
    int16_t output;

    output = (int16_t)(kp * err + kd * (err - pd_err_last));
    pd_err_last = err;

    return output;
}

static void PD_Reset(void)
{
    pd_err_last = 0;
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
 * CarMove — 直道/退出后PD差速驱动
 * left = base - Vz, right = base + Vz
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

/*
 * PivotTurn — 拐弯原地旋转
 * dir > 0: 右转 → 左轮正转 + 右轮反转
 * dir < 0: 左转 → 左轮反转 + 右轮正转
 *
 * 实际方向反了的话交换下面 MotorLeft/MotorRight 的正负号。
 */
static void PivotTurn(int8_t dir)
{
    if (dir > 0) {
        MotorLeft( CORNER_SPEED);
        MotorRight(-CORNER_PIVOT_PWR);
    } else if (dir < 0) {
        MotorLeft(-CORNER_PIVOT_PWR);
        MotorRight( CORNER_SPEED);
    } else {
        /* dir==0 不应发生, 防御: 直走 */
        MotorLeft( CORNER_SPEED);
        MotorRight( CORNER_SPEED);
    }
}

/* ============================================================================
 * LineWalk — 主循线逻辑 (每 10ms 调用一次, 100Hz)
 *
 * 拐弯三阶段状态机:
 *   阶段0 [直道]       → PD纠偏, 检测X1/X8触发
 *   阶段1 [拐弯-锁定]   → Pivot原地转, 不检测退出, 倒数MIN_CORNER_LOCK帧
 *   阶段2 [拐弯-检测]   → 继续Pivot, 检测连续N帧中线居中
 *   阶段3 [退出后增强]  → PD高KP纠偏, 倒数POST_CORNER_FRAMES帧, 回到直道
 * ============================================================================ */
static void LineWalk(void)
{
    uint8_t ir[8];
    int8_t  err;
    int16_t speed, pd_out;

    /* 持久状态 */
    static uint8_t  mode             = 0;   /* 0=直道 1=拐弯锁定 2=拐弯检测 3=退出增强 */
    static uint8_t  corner_enter_cnt = 0;
    static uint8_t  corner_exit_cnt  = 0;
    static uint16_t corner_frame     = 0;   /* 拐弯内帧计数 */
    static uint8_t  post_frame       = 0;   /* 退出增强帧计数 */
    static int8_t   corner_dir       = 0;   /* 1=右转, -1=左转 */
    static int8_t   last_err         = 0;
    static uint8_t  lost_cnt         = 0;

    /* 1. 读传感器 */
    ReadIR(ir);

    /* 2. 计算偏差 */
    err = CalcError(ir);

    /* 3. 丢线恢复 (仅直道生效; 拐弯内由pivot接管) */
    if (mode == 0 && IsLineLost(ir)) {
        lost_cnt += LOST_STEP;
        if (lost_cnt > LOST_MAX) lost_cnt = LOST_MAX;
        if (last_err > 0)      err =  lost_cnt;
        else if (last_err < 0) err = -lost_cnt;
        else                   err = 0;
    }
    if (!IsLineLost(ir)) {
        lost_cnt = 0;
        last_err = err;
    }

    /* ==================================================================
     * 4. 拐弯状态机
     * ================================================================== */

    switch (mode) {

    /* ---- 阶段0: 直道 ---- */
    case 0:
        if (IsCornerEntry(ir)) {
            corner_enter_cnt++;
            if (corner_enter_cnt >= CORNER_DEBOUNCE) {
                /* 进入拐弯-锁定阶段 */
                mode = 1;
                corner_frame = 0;
                corner_exit_cnt = 0;

                /* 根据哪一侧传感器触发决定转向 */
                if (ir[0] == ACTIVE_LEVEL && ir[7] != ACTIVE_LEVEL)
                    corner_dir = -1;  /* X1单触发 → 左转 */
                else if (ir[7] == ACTIVE_LEVEL && ir[0] != ACTIVE_LEVEL)
                    corner_dir =  1;  /* X8单触发 → 右转 */
                else
                    corner_dir = (err >= 0) ? 1 : -1;  /* 兜底: 按偏差符号 */
            }
        } else {
            corner_enter_cnt = 0;
        }
        break;

    /* ---- 阶段1: 拐弯-锁定 (强制pivot, 不检测退出) ---- */
    case 1:
        corner_frame++;
        if (corner_frame >= MIN_CORNER_LOCK) {
            mode = 2;  /* 锁定结束, 进入检测阶段 */
            corner_exit_cnt = 0;
        }
        break;

    /* ---- 阶段2: 拐弯-检测 (继续pivot, 检测居中退出) ---- */
    case 2:
        corner_frame++;
        if (IsLineCentered(ir)) {
            corner_exit_cnt++;
            if (corner_exit_cnt >= CORNER_EXIT_COUNT) {
                mode = 3;
                post_frame = 0;
                PD_Reset();
            }
        } else {
            corner_exit_cnt = 0;
        }
        if (corner_frame >= CORNER_MAX_FRAMES) {
            mode = 3;
            post_frame = 0;
            PD_Reset();
        }
        break;

    /* ---- 阶段3: 退出后增强纠偏 (高KP PD, 快速归中线) ---- */
    case 3:
        post_frame++;
        if (post_frame >= POST_CORNER_FRAMES) {
            mode = 0;
            corner_enter_cnt = 0;
            corner_dir = 0;
        }
        break;
    }

    /* ==================================================================
     * 5. 执行驱动
     * ================================================================== */

    if (mode == 1 || mode == 2) {
        /* 拐弯锁定+检测: 固定pivot旋转 */
        PivotTurn(corner_dir);
    } else if (mode == 3) {
        /* 退出增强: 中速 + 高KP PD */
        speed  = STRAIGHT_SPEED;
        pd_out = PD_Calc(err, POST_CORNER_KP, KD_STRAIGHT);
        if (pd_out >  speed) pd_out =  speed;
        if (pd_out < -speed) pd_out = -speed;
        CarMove(speed, pd_out);
    } else {
        /* 直道: 低速 + 低KP/高KD PD */
        speed  = STRAIGHT_SPEED;
        pd_out = PD_Calc(err, KP_STRAIGHT, KD_STRAIGHT);
        if (pd_out >  speed) pd_out =  speed;
        if (pd_out < -speed) pd_out = -speed;
        CarMove(speed, pd_out);
    }
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
