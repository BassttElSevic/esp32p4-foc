function evidence = verify_foc_protection(model)
% Prove the strict phase-current threshold, consecutive counter, and EN delay.
% Only the copy of the controller in this temporary model is simulated.
model = char(model);
controller = [model '/FOC controller'];
assert(bdIsLoaded(model), 'Load the FOC model before running the protection test.');

[~, suffix] = fileparts(tempname);
harness = matlab.lang.makeValidName(['foc_ocp_' suffix]);
new_system(harness);
cleanup = onCleanup(@() close_system(harness,0)); %#ok<NASGU>
Ts = 1e-3;
ws = get_param(harness,'ModelWorkspace');
owner = get_param(model,'ModelWorkspace');
variables = whos(owner);
for k = 1:numel(variables)
    name = variables(k).name;
    assignin(ws,name,getVariable(owner,name));
end
assignin(ws,'Ts_ctrl',Ts);
assignin(ws,'p_pairs',7);
assignin(ws,'s_enc',1);
assignin(ws,'theta_off',0);

% Default settings give a threshold hit at tick 100, a trip at 250 and EN low at 270.
limit = getVariable(ws,'ocp_limit_mA');
hitCount = getVariable(ws,'ocp_samples');
holdTime = getVariable(ws,'ocp_disable_s');
bits = getVariable(ws,'pwm_bits');
midpoint = getVariable(ws,'duty_mid');
neutral = floor((2^bits-1)*midpoint+0.5);
resetTick = min(100,hitCount-1);
tripTick = resetTick+hitCount;
disableTick = tripTick+ceil(round(holdTime*1e6)/(Ts*1e6));
tick = (0:disableTick+15).';
set_param(harness,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','Ts_ctrl','StartTime','0','StopTime',num2str(tick(end)*Ts,17), ...
    'ReturnWorkspaceOutputs','on');
iu = (limit+1)*ones(size(tick));
iu(tick == resetTick) = limit;
assignin(ws,'testPhaseCurrents',[tick*Ts, iu, zeros(size(tick))]);
add_block(controller,[harness '/Controller'],'Position',[355 65 520 325]);
add_block('simulink/Sources/From Workspace',[harness '/Phase current mA'], ...
    'VariableName','testPhaseCurrents','Interpolate','off','OutputAfterFinalValue','Holding final value', ...
    'SampleTime','Ts_ctrl', ...
    'Position',[60 175 210 205]);
add_line(harness,'Phase current mA/1','Controller/3');
sourceNames = {'Electrical Hz','Limit mA','Encoder rad','Encoder OK'};
ports = [1 2 4 5];
values = {'0','I_console_max_mA','0','1'};
for n = 1:numel(ports)
    y = 30+45*ports(n);
    add_block('simulink/Sources/Constant',[harness '/' sourceNames{n}], ...
        'Value',values{n},'SampleTime','Ts_ctrl','Position',[80 y 145 y+25]);
    add_line(harness,[sourceNames{n} '/1'],sprintf('Controller/%d',ports(n)));
end

logNames = {'ocpCounts','ocpEnable','','','','','','ocpTrip'};
for port = 1:numel(logNames)
    y = 38+37*port;
    if isempty(logNames{port})
        sink = sprintf('Unused%d',port);
        add_block('simulink/Sinks/Terminator',[harness '/' sink], ...
            'Position',[680 y 700 y+18]);
    else
        sink = logNames{port};
        add_block('simulink/Sinks/To Workspace',[harness '/' sink], ...
            'VariableName',sink,'SaveFormat','Timeseries','MaxDataPoints','inf', ...
            'Decimation','1','Position',[650 y 755 y+23]);
    end
    add_line(harness,sprintf('Controller/%d',port),[sink '/1']);
end

out = sim(harness);
counts = out.get('ocpCounts');
enable = out.get('ocpEnable');
trip = out.get('ocpTrip');
tolerance = Ts*1e-5;
expectedTime = tick*Ts;
assertTickTimes(trip,expectedTime,tolerance);
assertTickTimes(enable,expectedTime,tolerance);
assertTickTimes(counts,expectedTime,tolerance);
tripValues = reshape(trip.Data,[],1);
enableValues = reshape(enable.Data,[],1);
assert(isequal(double(tripValues),double(tick>=tripTick)), ...
    'Expected trip to latch on tick %d after the threshold-valued sample.',tripTick);
assert(isequal(double(enableValues),double(tick<disableTick)), ...
    'Expected EN to fall on tick %d after the neutral-PWM hold.',disableTick);
% The sample before the trip must exercise an active modulation command.
before = getdatasamples(counts,find(tick==tripTick-1,1));
assert(numel(before)==3 && any(before(:)~=neutral), ...
    'The pre-trip control sample did not exercise an active PWM command.');
for n = find(tick>=tripTick).'
    sample = getdatasamples(counts,n);
    assert(numel(sample)==3 && all(sample(:)==neutral), ...
        'All three PWM counts must be neutral starting on the OCP trigger tick.');
end
evidence = struct('trip_s',trip.Time(find(tripValues==1,1)), ...
                  'disable_s',enable.Time(find(enableValues==0,1)));
end

function assertTickTimes(series,expected,tol)
time = series.Time(:);
assert(numel(time)==numel(expected) && all(abs(time-expected)<tol), ...
    'Protection harness must log every control sample including t=0.');
end
