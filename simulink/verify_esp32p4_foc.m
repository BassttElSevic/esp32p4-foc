function [results, trace] = verify_esp32p4_foc()
% Run closed-loop and fault-injection checks in MATLAB/Simulink.
folder = fileparts(mfilename('fullpath'));
model = 'esp32p4_foc';
addpath(folder);
wasOpen = bdIsLoaded(model);
if ~wasOpen
    load_system(fullfile(folder,[model '.slx']));
    cleanup = onCleanup(@() close_system(model,0)); %#ok<NASGU>
end
[results,trace] = verify_foc_dynamics(model);
fprintf('Forward %.3f rpm, reverse %.3f rpm, PWM mean voltage error %.4g V.\n', ...
    results.nominal.actual_rpm,results.reverse.actual_rpm,results.pwm.max_mean_error_V);
results.protection = verify_foc_protection(model);
results.parameter_override = verify_parameter_override(model);
end

function result = verify_parameter_override(model)
% Override two controller gains and PWM resolution without rebuilding the SLX.
w = get_param(model,'ModelWorkspace');
Ts = getVariable(w,'Ts_ctrl');
bits = getVariable(w,'pwm_bits')-1;
mid = getVariable(w,'duty_mid');
test = Simulink.SimulationInput(model);
test = setModelParameter(test,'StopTime',num2str(30*Ts,17));
settings = {'Kp_w',0;'Ki_w',0;'load_Nm',0;'start_load_Nm',0;'step_speed_s',0; ...
            'pwm_bits',bits;'motor_x0',zeros(4,1);'pwm_switching',0};
for k=1:size(settings,1)
    test = setVariable(test,settings{k,1},settings{k,2},'Workspace',model);
end
out = sim(test);
c = out.logsout.getElement('counts').Values.Data;
q = out.logsout.getElement('idq_ref_mA').Values.Data;
neutral = floor((2^bits-1)*mid+.5);
result = struct('bits',bits,'neutral_count',neutral, ...
    'passed',all(c(:)==neutral) && all(q(:)==0));
assert(result.passed,'External parameter changes did not reach the controller/PWM.');
end
