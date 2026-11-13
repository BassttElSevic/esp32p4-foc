function [prediction,meta] = simulate_bench_sine(rows,tuning)
%SIMULATE_BENCH_SINE Run one BENCH1 sine experiment against the project SLX.
% rows is a single bench CSV read by readtable. Optional tuning fields are
% alpha_i, Kp_i, Ki_i and Kp_w in esp32p4_foc_data.m units. Feedback is
% sampled on the nearest nominal 1 ms control tick to each board timestamp.
% Once the simulated protection trips, subsequent predictions are missing.
% The simulation runs the commanded duration even when the board stopped early.

if nargin < 2 || isempty(tuning), tuning = struct(); end
mode = string(rows.mode(1));
assert(ismember(mode,["speed","current"]) && all(string(rows.mode)==mode));
hz = rows.frequency_Hz(1);
amp = rows.requested_amplitude(1);
duration = rows.cycles(1)/hz;
assert(hz>0 && amp>0 && duration>0 && all(diff(rows.t_us)>0));
assert(all(rows.frequency_Hz==hz & rows.requested_amplitude==amp & rows.cycles==rows.cycles(1)));
assert(all(rows.t_us>=0 & rows.t_us*1e-6<=duration));

root = fileparts(fileparts(mfilename('fullpath')));
modelDir = fullfile(root,'simulink');
% The saved SLX resolves its workspace script by name during load_system.
% Keep that script on the search path until the source model has been copied.
originalPath = path;
addpath(modelDir);
pathCleanup = onCleanup(@() path(originalPath)); %#ok<NASGU>
source = 'esp32p4_foc';
sourceWasOpen = bdIsLoaded(source);
if ~sourceWasOpen
    load_system(fullfile(modelDir,[source '.slx']));
    sourceCleanup = onCleanup(@() close_system(source,0)); %#ok<NASGU>
end
[~,name] = fileparts(tempname);
name = matlab.lang.makeValidName(['bench_' name]);
new_system(name);
cleanup = onCleanup(@() close_system(name,0)); %#ok<NASGU>
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'SolverMode','SingleTasking','FixedStep','Ts_plant','StopTime',num2str(duration,17), ...
    'ReturnWorkspaceOutputs','on','SignalLogging','off','SaveTime','off','SaveOutput','off');
w = get_param(name,'ModelWorkspace');
w.DataSource = 'MATLAB File';
w.FileName = fullfile(modelDir,'esp32p4_foc_data.m');
reload(w);
keys = fieldnames(tuning);
assert(all(ismember(keys,{'alpha_i','Kp_i','Ki_i','Kp_w'})),'Unsupported tuning field.');
for k=1:numel(keys)
    value = tuning.(keys{k});
    validateattributes(value,{'double'},{'scalar','real','finite','positive'});
    assignin(w,keys{k},value);
end
assignin(w,'bench_speed',double(mode=="speed"));
assignin(w,'bench_amp',amp);
assignin(w,'bench_hz',hz);
assignin(w,'bench_duration',duration);
% Starts aligned and stationary, as in a BENCH1 preflight. Absolute AS5600
% codes in a capture do not reveal the physical electrical zero by themselves.
assignin(w,'motor_x0',zeros(4,1));
assignin(w,'pwm_switching',0);
code = fileread(fullfile(root,'firmware','sine-bench','main','bench.c'));
limits = {'IQ_LIMIT_MA','MAX_PHASE_MA','MOD_LIMIT','MAX_RAW_SPEED','MAX_FILTERED_SPEED', ...
    'STUCK_CHECK_MOD','STUCK_CHECK_US'};
for k=1:numel(limits)
    token = regexp(code,['(?m)^#define\s+' limits{k} '\s+([.0-9eE+\-]+)f?(?=\s|$)'],'tokens','once');
    assert(~isempty(token),'Missing numeric bench macro %s',limits{k});
    assignin(w,limits{k},str2double(token{1}));
end

% Reuse the saved motor, dual-shunt ADC, AS5600 and PWM/bridge blocks.
for part = ["PWM and inverter","PMSM and shaft","Current and position sensors"]
    add_block([source '/' char(part)],[name '/' char(part)]);
end
add_block('simulink/Sources/Digital Clock',[name '/Tick'],'SampleTime','Ts_ctrl');
add_block('simulink/Sources/Constant',[name '/No load'],'Value','0','SampleTime','Ts_plant');
add_block('simulink/Sources/Constant',[name '/Bus voltage'],'Value','V_dc','SampleTime','Ts_plant');
add_block('simulink/User-Defined Functions/MATLAB Function',[name '/Bench controller']);
chart = sfroot().find('-isa','Stateflow.EMChart','Path',[name '/Bench controller']);
% The local codegen function below is the exact code used by the diagram.
% Keep one editable copy and bind the remaining arguments to workspace data.
sourceText = strrep(fileread([mfilename('fullpath') '.m']),sprintf('\r\n'),newline);
chart.Script = char(extractAfter(sourceText, newline + "% BEGIN EMBEDDED CONTROLLER" + newline));
params = [{'bench_speed','bench_amp','bench_hz','bench_duration','Ts_ctrl','p_pairs', ...
    's_enc','theta_off','torque_sign','alpha_i','alpha_w','Kp_i','Ki_i','Kp_w','Ki_w', ...
    'pwm_bits','duty_mid'},limits];
for k=1:numel(params)
    datum = chart.find('-isa','Stateflow.Data','Name',params{k});
    if isempty(datum), datum = Stateflow.Data(chart); datum.Name=params{k}; end
    datum.Scope='Parameter';
end
add_block('simulink/Sinks/To Workspace',[name '/Tick log'], ...
    'VariableName','sampleLog','SaveFormat','Timeseries','MaxDataPoints','inf');
lines = {'Tick/1','Bench controller/1'; ...
    'PMSM and shaft/1','Current and position sensors/1'; ...
    'PMSM and shaft/2','Current and position sensors/2'; ...
    'Current and position sensors/1','Bench controller/2'; ...
    'Current and position sensors/2','Bench controller/3'; ...
    'Bench controller/1','PWM and inverter/1'; 'Bench controller/2','PWM and inverter/2'; ...
    'Bench controller/3','Tick log/1'; 'Bus voltage/1','PWM and inverter/3'; ...
    'PWM and inverter/1','PMSM and shaft/1'; 'No load/1','PMSM and shaft/2'};
for k=1:size(lines,1), add_line(name,lines{k,1},lines{k,2}); end
for port=3:5
    sink = sprintf('Unused motor output %d',port);
    add_block('simulink/Sinks/Terminator',[name '/' sink]);
    add_line(name,sprintf('PMSM and shaft/%d',port),[sink '/1']);
end
out = sim(name);
trace = out.sampleLog;
if trace.IsTimeFirst
    signals = reshape(trace.Data,numel(trace.Time),7);
else
    signals = reshape(trace.Data,7,numel(trace.Time)).';
end
assert(all(isfinite(signals),'all'),'Nonfinite Simulink state or output.');
ts = getVariable(w,'Ts_ctrl');
tick = min(numel(trace.Time),round(rows.t_us*1e-6/ts)+1);
assert(all(abs(trace.Time(tick)-rows.t_us*1e-6)<=0.501*ts),'Log sample rate mismatch.');
selected = signals(tick,1:6);
% BENCH1 logs the OR of any limiting since the preceding recorded sample.
limitCount = cumsum(signals(:,6)~=0);
selected(:,6) = diff([0; limitCount(tick)])>0;
trip = find(signals(:,7)~=0,1);
meta = struct('stop_reason',"ok",'stop_detail',"",'stop_s',NaN,'Ts_ctrl',ts);
if ~isempty(trip)
    causes = ["phase_overcurrent","overspeed","overspeed","current_stuck"];
    detail = ["phase_current","raw_speed","filtered_speed","flat_shunts"];
    reason = signals(trip,7);
    meta.stop_reason = causes(reason);
    meta.stop_detail = detail(reason);
    meta.stop_s = trace.Time(trip);
    selected(tick>=trip,2:end) = NaN;
end
prediction = array2table([rows.t_us,selected],'VariableNames', ...
    {'t_us','reference','iq_mA','iq_filtered_mA','omega_raw_e_rad_s','omega_filtered_e_rad_s','limited'});
end

% BEGIN EMBEDDED CONTROLLER
function [counts,en,logrow] = bench_tick(t,uv,enc,bench_speed,bench_amp, ...
    bench_hz,bench_duration,Ts_ctrl,p_pairs,s_enc,theta_off,torque_sign, ...
    alpha_i,alpha_w,Kp_i,Ki_i,Kp_w,Ki_w,pwm_bits,duty_mid, ...
    IQ_LIMIT_MA,MAX_PHASE_MA,MOD_LIMIT,MAX_RAW_SPEED,MAX_FILTERED_SPEED, ...
    STUCK_CHECK_MOD,STUCK_CHECK_US)
%#codegen
% BENCH1 starts after calibration with zero PI integrals and zero LPF states.
persistent zd zq zw idf iqf wf prevEnc prevUV flatSince prevMod trip
if isempty(trip)
    zd=0; zq=0; zw=0; idf=0; iqf=0; wf=0;
    prevEnc=enc; prevUV=uv; flatSince=0; prevMod=0; trip=0;
end
levels = 2^pwm_bits-1;
counts = ones(3,1)*floor(duty_mid*levels+0.5);
en = 0;
ref = bench_amp*sin(2*pi*bench_hz*t);
logrow = [ref;0;0;0;0;0;trip];
if t<=0 || t>=bench_duration || trip~=0, return; end
if max(abs([uv(1);uv(2);-sum(uv)]))>=MAX_PHASE_MA
    trip=1;
elseif prevMod<STUCK_CHECK_MOD || any(uv~=prevUV)
    flatSince=t;
elseif (t-flatSince)*1e6>=STUCK_CHECK_US
    trip=4;
end
prevUV=uv;
delta = mod(enc-prevEnc+pi,2*pi)-pi;
raw = p_pairs*delta/Ts_ctrl;
prevEnc=enc;
if trip==0
    if abs(raw)>MAX_RAW_SPEED
        trip=2;
    else
        wf=wf+alpha_w*(raw-wf);
        if abs(wf)>MAX_FILTERED_SPEED, trip=3; end
    end
end
if trip~=0, logrow(7)=trip; return; end
theta=s_enc*p_pairs*enc+theta_off+wf*Ts_ctrl;
c=cos(theta); sn=sin(theta); beta=(uv(1)+2*uv(2))/sqrt(3);
id=uv(1)*c+beta*sn; iq=-uv(1)*sn+beta*c;
idf=idf+alpha_i*(id-idf); iqf=iqf+alpha_i*(iq-iqf);
target=ref; speedLimited=false;
if bench_speed~=0
    [target,zw,speedLimited]=pi_step(zw,ref-wf,Kp_w,Ki_w,Ts_ctrl,IQ_LIMIT_MA);
    target=torque_sign*s_enc*target;
end
[d,zd,dLimited]=pi_step(zd,-idf,Kp_i,Ki_i,Ts_ctrl,MOD_LIMIT);
[q,zq,qLimited]=pi_step(zq,target-iqf,Kp_i,Ki_i,Ts_ctrl,MOD_LIMIT);
mag=hypot(d,q);
if mag>MOD_LIMIT, d=d*MOD_LIMIT/mag; q=q*MOD_LIMIT/mag; end
prevMod=min(mag,MOD_LIMIT);
a=d*c-q*sn; b=d*sn+q*c;
phases=[a; -0.5*a+sqrt(3)*0.5*b; -0.5*a-sqrt(3)*0.5*b];
counts=floor((duty_mid+phases)*levels+0.5);
en=1;
logrow=[ref;iq;iqf;raw;wf;double(speedLimited || dLimited || qLimited || mag>MOD_LIMIT);trip];
end

function [u,next,limited]=pi_step(x,error,kp,ki,dt,limit)
cmd=x+kp*error;
limited=abs(cmd)>limit;
next=x;
if ~limited || cmd*error<0, next=min(limit,max(-limit,x+ki*dt*error)); end
u=min(limit,max(-limit,cmd));
end
