% ESP32-P4 M2 闭环仿真。此脚本是 Simulink 模型工作区的数据源。
% 保存修改后重新打开模型，或在 MATLAB 中执行
% reload(get_param('esp32p4_foc','ModelWorkspace'))。
% 控制参数取自 firmware/foc/main/main.c @ c20fad4。

%% 灯哥 DENG FOC 套件 2208 和负载（每相参数，SI 单位）
p_pairs = 7;                     % 固件使用的极对数
% 另一只 DengFOC 配套 2208 的公开辨识结果见 README.md。
Ke_LL_rms_per_krpm = 7.68;       % 线电压 RMS / 机械千转每分
psi_pm = Ke_LL_rms_per_krpm * sqrt(2/3) / (p_pairs * (1000*2*pi/60)); % Wb
R_s = 18.76;                     % Profiler 每相电阻 (ohm)
L_d = 2.58e-3;                   % Profiler 相电感 (H)，采用无凸极近似
L_q = L_d;
J_rotor = 0.5e-6;                % Profiler 界面数值按机械轴使用，需实机核对 (kg*m^2)
B_rotor = 1.07e-6;               % 同一界面给出的黏滞摩擦 (N*m*s/rad)
J_load = 0;                      % 折算到电机轴的附加负载惯量
B_load = 0;                      % 折算到电机轴的附加黏滞摩擦
J_m = J_rotor + J_load;
B_m = B_rotor + B_load;
motor_x0 = [0; 0; 0; 0];         % [id/A; iq/A; 机械角/rad; 机械速度/(rad/s)]
V_dc = 12;                       % 母线电压 (V)

%% 控制任务、三路 PI 和给定限幅
Ts_ctrl = 1e-3;                  % 用定周期近似实机可变的循环入口间隔
ctrl_dt_max_s = 0.05;
f_console_max_Hz = 80;
f_closed_max_Hz = 60;
I_console_max_mA = 1800;
Kp_w = 27;                       % mA/(rad/s)，电角速度
Ki_w = 200;                      % mA/((rad/s)*s)
Kp_i = 2.5e-4;                   % 调制比/mA，两轴采用同一整定
Ki_i = 8.0e-4;                   % 调制比/(mA*s)
id_target_mA = 0;
mod_limit = 0.40;                % 逐轴和圆形限幅
torque_sign = 1;
alpha_i = 0.15;
alpha_w = 0.05;

%% 传感器、安装标定与保护
R_shunt = 0.015;                 % ohm，每相串联分流电阻
sense_gain = 50;                 % INA199A1 增益 V/V
sense_wiring_sign = -1;          % 模拟硬件差分输入的接线方向
sense_sign = -1;                 % 数字换算采用的 SENSE_SIGN
b_u_mV = 1680;                   % 示例静止零点，实机逐相标定
b_v_mV = 1680;
adc_bits = 12;
adc_range_mV = 3100;
unipolar_threshold_mV = 300;
enc_bits = 12;                   % AS5600 RAW ANGLE
s_enc = 1;                       % 编码器安装方向/已标定方向 (+1 或 -1)
theta_off = 0;                   % 对应标定后的电角度零点 (rad)
enc_dt_min_us = 100;             % 仅在开区间内刷新速度
enc_dt_max_us = 20000;
enc_fail_decay = 0.9;
ocp_limit_mA = 1500;
ocp_samples = 150;
ocp_disable_s = 0.020;           % 中点 PWM 到撤使能的延迟

%% 逆变器、仿真步长与试验输入
pwm_Hz = 20000;
pwm_bits = 10;
duty_mid = 0.5;
pwm_switching = 0;               % 0 平均桥；1 逐载波计算每个对象步的导通比例
Ts_plant = 1e-5;                 % 对象 RK4 步长 (s)，Ts_ctrl 应为其整数倍
sim_end_s = 3;
start_Hz = 0;
target_Hz = 8;                   % 有符号电频率 (Hz)
step_speed_s = 0.10;
Imax = 200;                      % 速度 PI 电流上限 (mA)
start_load_Nm = 0;
load_Nm = 0.002;                 % 有符号负载转矩 (N*m)
step_load_s = 1.25;
enc_ok = 1;                      % 正常读角；设 0 模拟通信失败
