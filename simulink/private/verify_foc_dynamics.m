function [summary,trace] = verify_foc_dynamics(model)
% Simulate the saved model in normal mode. Outputs are derived from logsout.
% RPM is signed in the plant shaft coordinate system. Current is in mA.
load_system(model);
[~,name] = fileparts(char(model));
mw = get_param(name,'ModelWorkspace');
Ts = getVariable(mw,'Ts_ctrl');
p = getVariable(mw,'p_pairs');
direction = getVariable(mw,'s_enc');
levels = 2^getVariable(mw,'pwm_bits');
switchHz = getVariable(mw,'pwm_Hz');
Vdc = getVariable(mw,'V_dc');
target = abs(getVariable(mw,'target_Hz'));
loadStep = getVariable(mw,'step_load_s');
loadValue = abs(getVariable(mw,'load_Nm'));
base = Simulink.SimulationInput(name);
base = setModelParameter(base,'SimulationMode','normal','SignalLogging','on','ReturnWorkspaceOutputs','on');
base = setVariable(base,'pwm_switching',0,'Workspace',name);

% The baseline uses the model's configured step times, stop time and plant.
forward = setVariable(base,'target_Hz',target,'Workspace',name);
forward = setVariable(forward,'load_Nm',direction*loadValue,'Workspace',name);
simout = sim(forward);
trace = trace_table(simout.logsout,Ts,p,direction);
summary.nominal = metrics(trace,loadStep,levels);
clear simout

reverse = setVariable(base,'target_Hz',-target,'Workspace',name);
reverse = setVariable(reverse,'load_Nm',-direction*loadValue,'Workspace',name);
simout = sim(reverse);
revTrace = trace_table(simout.logsout,Ts,p,direction);
summary.reverse = metrics(revTrace,loadStep,levels);
clear simout revTrace

% Resolve switching and include a full settled controller hold interval.
% The shortened experiment applies the speed step at t=0 with zero load.
pwmTs = Ts / ceil(Ts/min(2e-6,1/(20*switchHz)));
carrierStop = ceil(.65/Ts)*Ts;
carrier = setModelParameter(forward,'StopTime',sprintf('%.16g',carrierStop));
carrier = setVariable(carrier,'pwm_switching',1,'Workspace',name);
carrier = setVariable(carrier,'Ts_plant',pwmTs,'Workspace',name);
carrier = setVariable(carrier,'step_speed_s',0,'Workspace',name);
carrier = setVariable(carrier,'load_Nm',0,'Workspace',name);
simout = sim(carrier);
summary.pwm = pwm_metrics(simout.logsout,Ts,pwmTs,switchHz,levels,Vdc);
summary.passed = summary.nominal.passed && summary.reverse.passed && summary.pwm.passed;
end

function T = trace_table(logs,Ts,p,direction)
[time,~] = series(logs,'omega_ref',1);
% Select complete control samples at roughly 1 ms. Keep the exact logged times.
stride = max(1,round(.001/Ts));
t = time(1:stride:end);
T = table(t,'VariableNames',{'time_s'});
T.ref_rpm = direction * sample(logs,'omega_ref',t,1)*60/(2*pi*p);
T.measured_rpm = direction * sample(logs,'omega_hat',t,1)*60/(2*pi*p);
T.actual_rpm = sample(logs,'omega_m',t,1)*60/(2*pi);
ref = sample(logs,'idq_ref_mA',t,2); fb = sample(logs,'idq_f_mA',t,2);
actual = 1000*sample(logs,'idq_A',t,2); u = sample(logs,'vdq',t,2);
T.id_ref_mA = ref(:,1); T.iq_ref_mA = ref(:,2);
T.id_f_mA = fb(:,1); T.iq_f_mA = fb(:,2);
T.id_true_mA = actual(:,1); T.iq_true_mA = actual(:,2);
T.vd = u(:,1); T.vq = u(:,2);
T.Te_Nm = sample(logs,'Te_Nm',t,1); T.load_Nm = sample(logs,'load_torque',t,1);
T.counts = sample(logs,'counts',t,3); T.enable = sample(logs,'enable',t,1);
T.trip = sample(logs,'trip',t,1); T.iUVW_A = sample(logs,'iUVW_A',t,3);
T.uv_mA = sample(logs,'uv_mA',t,2); T.vUVW = sample(logs,'vUVW',t,3);
T.theta_m = sample(logs,'theta_m',t,1); T.theta_enc = sample(logs,'theta_enc',t,1);
end

function m = metrics(T,loadStep,levels)
t = T.time_s;
window = t >= t(end)-min(.30,(t(end)-loadStep)/2);
before = t >= loadStep-.1 & t < loadStep;
after = find(t >= loadStep-1e-12);
assert(any(before) && ~isempty(after) && any(window),'Verification needs both pre-load and post-load data.');
m.reference_rpm = mean(T.ref_rpm(window));
m.measured_rpm = mean(T.measured_rpm(window)); m.actual_rpm = mean(T.actual_rpm(window));
eActual = T.actual_rpm-T.ref_rpm; eEst = T.measured_rpm-T.ref_rpm;
m.rms_error_rpm = sqrt(mean(eActual(window).^2));
m.measured_rms_error_rpm = sqrt(mean(eEst(window).^2));
m.preload_rpm = mean(T.actual_rpm(before));
m.applied_load_Nm = mean(T.load_Nm(window))-mean(T.load_Nm(before));
loadDirection = sign(m.applied_load_Nm);
m.load_delta_rpm = max(0,max(loadDirection*(m.preload_rpm-T.actual_rpm(after))));
tol = max(.1,.02*abs(m.reference_rpm));
lastOut = find(abs(eActual(after))>tol,1,'last');
if isempty(lastOut)
    m.load_recovery_s = 0;
elseif lastOut == numel(after)
    m.load_recovery_s = NaN;
else
    m.load_recovery_s = T.time_s(after(lastOut+1))-loadStep;
end
m.mean_iq_mA = mean(T.iq_true_mA(window)); m.mean_iq_feedback_mA = mean(T.iq_f_mA(window));
m.mean_iq_ref_mA = mean(T.iq_ref_mA(window)); m.mean_id_mA = mean(T.id_true_mA(window));
m.no_trip = all(T.trip == 0) && all(T.enable ~= 0);
m.counts_in_bounds = count_bounds(T.counts,levels);
m.passed = m.no_trip && m.counts_in_bounds && isfinite(m.load_recovery_s) ...
    && m.rms_error_rpm < tol && m.measured_rms_error_rpm < tol ...
    && abs(m.reference_rpm) > 0 && m.applied_load_Nm ~= 0 ...
    && abs(m.mean_iq_feedback_mA-m.mean_iq_ref_mA) < 10;
end

function m = pwm_metrics(logs,Ts,pwmTs,f,levels,Vdc)
[tV,v] = series(logs,'vUVW',3); [tC,counts] = series(logs,'counts',3);
[~,trip] = series(logs,'trip',1); [~,enable] = series(logs,'enable',1);
m.no_trip = all(trip==0) && all(enable~=0);
m.counts_in_bounds = count_bounds(counts,levels);
m.carrier_Hz = f; m.plant_step_s = pwmTs;
m.complete_periods = floor(Ts*f+1e-10);
assert(m.complete_periods>=2,'PWM verification requires at least two carrier cycles per control hold.');
% Use the penultimate held command. Signals logged at t describe [t,t+dt).
start = tC(end-1); span = m.complete_periods/f;
phaseVoltage = Vdc*(counts(end-1,:)-mean(counts(end-1,:)))/levels;
mask = tV >= start-pwmTs/4 & tV < start+span-pwmTs/4;
testVoltage = v(mask,:);
assert(size(testVoltage,1)>2,'Insufficient samples in the PWM measurement interval.');
meanV = mean(testVoltage,1);
m.interval_start_s = start; m.interval_duration_s = span;
m.measured_mean_V = meanV; m.expected_mean_V = phaseVoltage;
m.max_mean_error_V = max(abs(meanV-phaseVoltage));
m.phase_peak_to_peak_V = max(testVoltage,[],1)-min(testVoltage,[],1);
m.max_neutral_residual_V = max(abs(sum(testVoltage,2)));
m.nontrivial_switching = max(m.phase_peak_to_peak_V) > .12*Vdc;
m.passed = m.no_trip && m.counts_in_bounds && m.nontrivial_switching ...
    && m.max_mean_error_V < .025*Vdc && m.max_neutral_residual_V < 1e-7*Vdc;
end

function ok = count_bounds(counts,levels)
ok = all(isfinite(counts(:)) & counts(:)>=0 & counts(:)<=levels-1 & counts(:)==round(counts(:)));
end

function y = sample(logs,name,t,width)
[ts,x] = series(logs,name,width);
if isequal(ts,t)
    y=x;
else
    % All signals are held. Tiny time-roundoff near a shared tick is immaterial.
    tq = min(ts(end),max(ts(1),t+16*eps(max(1,t(end)))));
    y = interp1(ts,x,tq,'previous');
end
end

function [t,x] = series(logs,name,width)
element = logs.getElement(name);
sig = element.Values; t = double(sig.Time(:)); raw = double(sig.Data);
n = numel(t); sz = size(raw);
if n == 1 || width == 1
    x = reshape(raw,n,width);
elseif sz(1)==n && prod(sz(2:end))==width
    x = reshape(raw,n,width);
elseif sz(end)==n && prod(sz(1:end-1))==width
    % Simulink matrix signal  [component,1,time]  -> [time,component].
    x = reshape(raw,width,n).';
else
    error('FOC:BadSignalShape','Signal %s has size %s with %d times and %d channels.',name,mat2str(sz),n,width);
end
assert(all(isfinite(x(:))) && all(isfinite(t)),'Nonfinite data in signal %s.',name);
end
