#pragma once

/* ═════════════ 硬件引脚 ═════════════ */
#define PIN_PWM_U   26   /* M2-PWM-1 */
#define PIN_PWM_V   27   /* M2-PWM-2 */
#define PIN_PWM_W   32   /* M2-PWM-3 */
#define PIN_EN      33   /* M2-EN */
#define PIN_ISENS_U 22   /* M2-ISENS-1 */
#define PIN_ISENS_V 23   /* M2-ISENS-2 */
#define PIN_I2C_SCL 8
#define PIN_I2C_SDA 7

#define ADC_CH_U    ADC_CHANNEL_6   /* GPIO22 */
#define ADC_CH_V    ADC_CHANNEL_7   /* GPIO23 */

/* ═════════════ AS5600 ═════════════ */
#define AS5600_ADDR            0x36
#define AS5600_REG_STATUS      0x0B
#define AS5600_REG_RAW_ANGLE   0x0C
#define AS5600_RES             4096.0f
#define AS5600_ANGLE_MASK      0x0FFF
/* 参照 balancecar 工程 (已验证跑通) 的 I2C 参数:
 *   I2C_BUS_DEFAULT_FREQ_HZ    400000
 *   I2C_BUS_DEFAULT_TIMEOUT_MS 1000
 *   I2C_BUS_DEFAULT_GLITCH_CNT 7
 *   ENCODER_PROBE_RETRY_DEFAULT  10,  gap 50ms
 *   ENCODER_SETTLE_MS_DEFAULT    500  (Wire.begin() 后等 500ms) */
#define I2C_SPEED_HZ           400000
#define I2C_TIMEOUT_MS         1000
#define I2C_GLITCH_CNT         7
#define I2C_PROBE_RETRY        10
#define I2C_PROBE_GAP_MS       50
#define ENCODER_SETTLE_MS      1000   /* 上电等待 */
#define I2C_FRAME_TIMEOUT_MS   5      /* 连续打帧时的短超时, 保证帧密度高 */

/* ═════════════ 电机常数 ═════════════ */

/* 极对数: 这里填一个初值, 上电标定会实测出真实值并自动采用。
 * 12N14P 结构 = 7 极对。 */
#define MOTOR_POLE_PAIRS_INIT   7      /* 灯哥FOC 2208 = 12N14P = 7 极对 */

#define MOTOR_KV_RPM_PER_V      110.0f  /* 2208 = 110KV */
#define MOTOR_KT_NM_PER_A       (9.5493f / MOTOR_KV_RPM_PER_V)

/* ═════════════ 控制参数 ═════════════ */

/* 速度环 PI: 输出为 Vq (调制比)。Kp 单位 调制比/(rad/s) */
#define SPD_KP        0.0040f
#define SPD_KI        0.0060f

/* 电压前馈 (按目标频率给一个基础电压, 速度环在此之上修正) */
#define FF_BOOST      0.080f
#define FF_SLOPE      0.004f

/* Vq 硬限幅 (SPWM 必须 < 0.5) */
#define AMP_MIN       0.010f
#define AMP_MAX       0.400f

/* 电流上限: 电压幅值的天花板, 防止转矩过大烧线圈 */
#define I_LIMIT_DEFAULT_MA   200.0f   /* Iq 目标 (mA) */
#define CUR_LIM_KP    8.0e-5f
#define CUR_LIM_KI    4.0e-4f
#define IAMP_FILT_ALPHA  0.150f   /* dq 电流低通, ~24Hz @1kHz */
/* dq 电流环 PI: 单位 调制比/mA, 调制比/(mA·s) */
/* 速度外环 PI: 输出 = Iq 指令 (mA), 输入 = 电角速度误差 (rad/s) */
#define SPD_KP_MA        27.0f
#define SPD_KI_MA        200.0f
/* 转矩方向手动修正系数。
 * 实测规律: 转矩符号恰好等于标定得到的编码器方向 s_enc_dir。
 * 所以实际用的是 TORQUE_SIGN * s_enc_dir, 这里留 1.0 表示不自作主张;
 * 万一以后换了相序还反, 就把它改成 -1.0。 */
#define TORQUE_SIGN      (1.0f)
/* 电流内环开关:
 *   1 = 速度 PI -> Iq -> dq 电流 PI (需要两路电流采样都正常)
 *   0 = 速度 PI 直接输出电压幅值 (不依赖电流采样, 用于电流采样故障时)
 *       电流读数仅打印监视, 且不做 OCP (读数不可信) */
#define USE_CURRENT_LOOP   1
/* 电压模式速度环: 输出调制比, 输入 rad/s */
#define SPD_V_KP           0.0050f
#define SPD_V_KI           0.0100f
#define CUR_KP_AMP_PER_MA   2.5e-4f
#define CUR_KI_AMP_PER_MA_S 8.0e-4f

/* 过流保护 */
#define OCP_MA        1500.0f  /* 新量程 ±1930mA */
#define OCP_COUNT     150

/* 采样链路 */
/* 电流采样极性。实测: vd=0、vq>0 时测得 iq<0, 符号相反,
 * 说明两路分流器的差分输入方向与约定相反, 这里取 -1 纠正。
 * 若将来改线导致符号又反, 改回 +1 即可。 */
#define SENSE_SIGN    (-1.0f)
#define R_SHUNT_OHM   0.015f   /* R015 = 15mΩ */
#define AMP_GAIN      50.0f
#define ADC_OVERSAMPLE 2      /* 降过采样以提高控制率 */

/* 标定 */
#define OL_AMP        0.120f    /* 开环测试固定调制比 */
#define CAL_AMP       0.200f    /* 标定吸引磁场强度 (小了转子会滞后) */
#define CAL_SETTLE_MS 1500      /* 每个位置等待转子稳定的时间 */

/* 其他 */
#define STEP_MS           1
#define SPEED_LPF_ALPHA   0.05f   /* 编码器差分测速的低通 */

/* ═════════════ 内部常量 ═════════════ */
#define PWM_FREQ_HZ    20000
#define PWM_RES_BITS   LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX   1023.0f
#define PWM_DUTY_MID   511.5f
#define TWO_PI_F       6.28318530718f
#define SQRT3_F        1.7320508f
#define ONE_OVER_SQRT3 0.57735027f
#define ADC_FULL_MV    3100.0f
#define RAIL_MARGIN_MV 200.0f
