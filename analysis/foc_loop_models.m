function model = foc_loop_models(dataFile)
%FOC_LOOP_MODELS Sampled small-signal loops of the aligned drive at rest.
%   M = FOC_LOOP_MODELS() reads the same parameter script as the Simulink
%   model. Supply a different .m parameter script as the optional argument.
%   M.current is the free-rotor q-current loop with the speed loop open.
%   M.speed closes the q-current loop first. M.locked holds the shaft still.
%   In each case open is the return ratio at the controller error junction
%   (positive sign, characteristic equation 1+open=0). closed and measured
%   map the reference to the physical and filtered outputs, respectively.
%   Current is in A, speed is electrical rad/s and the sample time is in s.

if nargin < 1
    root = fileparts(fileparts(mfilename('fullpath')));
    dataFile = fullfile(root,'simulink','esp32p4_foc_data.m');
end
run(dataFile);

steps = round(Ts_ctrl/Ts_plant);
assert(steps >= 1 && abs(steps*Ts_plant-Ts_ctrl) <= 8*eps(Ts_ctrl), ...
    'Ts_ctrl must be a positive integer multiple of Ts_plant.');
dtUs = round(1e6*Ts_ctrl);
assert(enc_ok == 1 && dtUs > enc_dt_min_us && dtUs < enc_dt_max_us, ...
    'The speed model requires successful encoder updates at each sample.');
assert(pwm_switching == 0 && id_target_mA == 0, ...
    'The rest linearization uses averaged PWM and zero d-axis reference.');
assert(~(b_u_mV < unipolar_threshold_mV && b_v_mV < unipolar_threshold_mV), ...
    'The unipolar current clamp has no two-sided linearization at zero.');

levels = 2^pwm_bits;
gainPWM = V_dc*(levels-1)/levels;
kt = 1.5*p_pairs*psi_pm;
% Augmented continuous state [iq; omega_e; theta_e; held modulation].
generator = [-R_s/L_q, -psi_pm/L_q, 0, gainPWM/L_q;
             p_pairs*kt/J_m, -B_m/J_m, 0, 0;
             0, 1, 0, 0;
             0, 0, 0, 0];
transition = rk4_hold(generator,Ts_plant,steps);
phi = transition(1:2,1:2);
gamma = transition(1:2,4);
currentSign = sense_wiring_sign*sense_sign;
piDt = min(ctrl_dt_max_s,Ts_ctrl);
currentKp = 1000*Kp_i;
currentIntegralStep = 1000*Ki_i*piDt;
currentPlant = current_sensor(phi,gamma,alpha_i,currentSign,Ts_ctrl);
[currentOpen,currentClosed] = pi_loop(currentPlant,currentKp,currentIntegralStep);
model.current = channels(currentOpen,currentClosed);

lockedTransition = rk4_hold([-R_s/L_q gainPWM/L_q;0 0],Ts_plant,steps);
lockedPlant = current_sensor(lockedTransition(1,1),lockedTransition(1,2), ...
    alpha_i,currentSign,Ts_ctrl);
[lockedOpen,lockedClosed] = pi_loop(lockedPlant,currentKp,currentIntegralStep);
model.locked = channels(lockedOpen,lockedClosed);

% At k+1 the encoder sees the angle increment accumulated under u[k].
% Folding that interval integral directly into its LPF avoids introducing
% the unobservable absolute-angle state or subtracting two large angles.
speedWeight = alpha_w*s_enc/(dtUs*1e-6);
angleRow = transition(3,1:2);
angleInput = transition(3,4);
A = [currentPlant.A, zeros(3,1);
     speedWeight*angleRow, 0, 1-alpha_w];
B = [currentPlant.B; speedWeight*angleInput];
C = [currentPlant.C, zeros(2,1); 0 1 0 0; 0 0 0 1];
allSensors = ss(A,B,C,zeros(4,1),Ts_ctrl);
[~,innerClosed] = pi_loop(allSensors,currentKp,currentIntegralStep);
direction = torque_sign*s_enc;
speedPlant = innerClosed(3:4,:);
[speedOpen,speedClosed] = pi_loop(speedPlant, ...
    direction*Kp_w/1000,direction*Ki_w*piDt/1000);
model.speed = channels(speedOpen,speedClosed);
model.Ts = Ts_ctrl;
model.currentLimitA = min(Imax,I_console_max_mA)/1000;
model.modLimit = mod_limit;
model.polePairs = p_pairs;
end

function transition = rk4_hold(generator,h,n)
% One held-input controller interval contains n fixed-step RK4 updates.
d = h*generator;
transition = (eye(size(d)) + d + d^2/2 + d^3/6 + d^4/24)^n;
end

function plant = current_sensor(phi,gamma,alpha,polarity,Ts)
% Store the current feedback already available at the start of tick k.
n = size(phi,1);
A = [phi, zeros(n,1); alpha*polarity*phi(1,:), 1-alpha];
B = [gamma; alpha*polarity*gamma(1)];
C = zeros(2,n+1);
C(1,1) = 1;
C(2,end) = 1;
plant = ss(A,B,C,zeros(2,1),Ts);
end

function [forward,closed] = pi_loop(plant,kp,integralStep)
% All plants above are strictly proper. Combine PI and plant once, then
% close output 2 (the measured signal) without duplicating plant states.
% u[k] = kp*e[k] + xi[k], xi[k+1] = xi[k] + integralStep*e[k].
[A,B,C,D,Ts] = ssdata(plant);
Af = [A,B;zeros(1,size(A,2)),1];
Bf = [kp*B;integralStep];
Cf = [C,zeros(size(C,1),1)];
forward = ss(Af,Bf,Cf,D,Ts);
closed = ss(Af-Bf*Cf(2,:),Bf,Cf,D,Ts);
end

function loop = channels(forward,closed)
loop.open = forward(2,:);
loop.closed = closed(1,:);
loop.measured = closed(2,:);
end
