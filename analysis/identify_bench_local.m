function fit = identify_bench_local(outDir)
%IDENTIFY_BENCH_LOCAL Fit one local dq/shaft model from the first session.
%   FIT = IDENTIFY_BENCH_LOCAL() returns the plant overrides expected by
%   simulate_bench_sine(rows,tuning,fit) and saves a one-row parameters.csv.
%   Only the three explicitly named first-session files enter the fit.
%
%   The current-mode controller command is reconstructed at the fundamental,
%   not measured. The fitted resistance is an effective drive coefficient.
%   A local angular restoring term is referenced to each run's initial angle;
%   the code does not identify the physical cogging law or a winding resistance.
root = fileparts(fileparts(mfilename('fullpath')));
dataDir = fullfile(root,'bench-results');
if nargin < 1, outDir = fullfile(dataDir,'local-model'); end
run(fullfile(root,'simulink','esp32p4_foc_data.m'));
initialTuning = readtable(fullfile(dataDir,'tune','parameters.csv'),'TextType','string');
initialTuning = initialTuning(initialTuning.folder==".",:);
assert(height(initialTuning)==1,'Expected one first-session controller configuration.');
Kt = 1.5*p_pairs*psi_pm;

currentFiles = ["current_2Hz_60.csv","current_2Hz_150.csv"];
effectiveR = zeros(size(currentFiles));
for k = 1:numel(currentFiles)
    h = trialPhasors(fullfile(dataDir,currentFiles(k)),"current",enc_bits);
    % The controller uses the PREVIOUS integrator value at a 1-ms nominal
    % control tick. The next tick accumulates Ki*Ts*(reference-feedback).
    piResponse = initialTuning.Kp_i + initialTuning.Ki_i*Ts_ctrl/expm1(1i*h.omega*Ts_ctrl);
    dutyScale = V_dc*(2^pwm_bits-1)/2^pwm_bits;
    zeroOrderHold = -expm1(-1i*h.omega*Ts_ctrl)/(1i*h.omega*Ts_ctrl);
    voltage = dutyScale*zeroOrderHold*piResponse*(h.reference_mA-h.filtered_mA);
    backEmf = psi_pm*p_pairs*1i*h.omega*h.theta_rad;
    % Local q-axis equation at small angles and id near zero:
    % Vq - psi*p*j*w*theta - j*w*Lq*Iq = R_eff*Iq.
    effectiveR(k) = real((voltage-backEmf)/h.iq_A - 1i*h.omega*L_q);
end
% Equal weight per trial in impedance units; the larger-amplitude
% experiment must not dominate simply because its applied current is higher.
fit.effective_R_ohm = mean(effectiveR);

h = trialPhasors(fullfile(dataDir,'speed_1Hz_2.csv'),"speed",enc_bits);
% Kt*i_q = J*theta'' + B_eff*theta' + k_local*(theta-theta0).
% The recorded encoder coordinate is used with the model's aligned positive
% q-axis convention. Only combinations of J/Kt, B/Kt and k/Kt are observable.
currentPerAngle = h.iq_A/h.theta_rad;
fit.restoring_A_per_rad = real(currentPerAngle) + J_m*h.omega^2/Kt;
fit.effective_B_Nms_per_rad = Kt*imag(currentPerAngle)/h.omega;
assert(all(structfun(@(v) isfinite(v) && v>0,fit)), ...
    'Non-positive local coefficients: check polarity, phasors and excitation.');
fprintf('First-session current captures imply %.5f and %.5f effective ohm.\n',effectiveR);
fprintf('1-Hz current/angle = %.5f %+.5fi A/rad; nominal Kt = %.6f Nm/A.\n', ...
    real(currentPerAngle),imag(currentPerAngle),Kt);
if ~isfolder(outDir), mkdir(outDir); end
writetable(struct2table(fit),fullfile(outDir,'parameters.csv'));
end

function h = trialPhasors(filename,mode,encoderBits)
rows = readtable(filename,'TextType','string');
assert(~isempty(rows) && all(rows.mode==mode) && all(rows.board_end=="ok") && ...
    all(rows.expected_samples==height(rows)) && all(rows.limited==0) && ...
    all(rows.seq==(0:height(rows)-1)') && all(diff(rows.t_us)>0), ...
    'The training record must be a complete, unsaturated, ordered capture: %s',filename);
hz = rows.frequency_Hz(1);
cycles = rows.cycles(1);
assert(hz>0 && all(rows.frequency_Hz==hz & rows.cycles==cycles) && cycles>=2);
h.omega = 2*pi*hz;
t = rows.t_us*1e-6;
keep = t*hz >= cycles-2 & t*hz < cycles;
t = t(keep);
assert(numel(t)>10 && (t(end)-t(1))*hz>1.9 && max(diff(t))*hz<0.1, ...
    'Insufficient coverage of the final two cycles: %s',filename);
A = [sin(h.omega*t),cos(h.omega*t),ones(size(t))];
angles = unwrap(rows.angle_code*2*pi/(2^encoderBits));
% A small constant angular drift must not be mistaken for a harmonic.
angleFit = [A,t-mean(t)]\angles(keep);
h.theta_rad = complex(angleFit(1),angleFit(2));
coeff = A\[rows.iq_mA(keep)/1000,rows.reference(keep),rows.iq_filtered_mA(keep)];
phasors = complex(coeff(1,:),coeff(2,:));
h.iq_A = phasors(1);
h.reference_mA = phasors(2); % used in current mode only
h.filtered_mA = phasors(3);
assert(abs(h.iq_A)>eps && abs(h.theta_rad)>eps,'Unresolved harmonic: %s',filename);
end
