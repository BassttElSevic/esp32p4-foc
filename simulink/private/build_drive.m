function build_drive(path)
% Three synchronized edge-aligned LEDC channels and a floating-neutral load.
add_block('simulink/Sources/In1', [path '/counts'], 'Port','1', 'Position',[20 50 50 64]);
add_block('simulink/Sources/In1', [path '/enable'], 'Port','2', 'Position',[20 250 50 264]);
add_block('simulink/Sources/In1', [path '/vdc'], 'Port','3', 'Position',[20 300 50 314]);
add_block('simulink/Discrete/Zero-Order Hold', [path '/Hold counts'], 'SampleTime','Ts_plant', 'Position',[105 38 185 76]);
add_block('simulink/Discrete/Zero-Order Hold', [path '/Hold enable'], 'SampleTime','Ts_plant', 'Position',[105 238 185 276]);
add_block('simulink/Discrete/Zero-Order Hold', [path '/Hold vdc'], 'SampleTime','Ts_plant', 'Position',[105 288 185 326]);
add_block('simulink/Discrete/Unit Delay', [path '/Carrier phase'], 'SampleTime','Ts_plant', 'InitialCondition','0', 'Position',[290 160 365 198]);

lines = {
'function [high_fraction, next_phase] = pwm_interval(counts, phase)'
'%#codegen'
'% The hardware counter has 2^pwm_bits states, with a high fraction of N/levels.'
'% The interval starts at the stored leading-edge carrier phase.'
'levels = 2^pwm_bits;'
'periods = pwm_Hz * Ts_plant;'
'stop = phase + periods;'
'next_phase = mod(stop,1);'
'high_fraction = zeros(3,1);'
'for k = 1:3'
'    duty = min(levels-1,max(0,round(counts(k)))) / levels;'
'    if pwm_switching ~= 0'
'        % Integral of the rectangular pulse train, in carrier periods.'
'        start_area = min(phase,duty);'
'        stop_area = floor(stop)*duty + min(mod(stop,1),duty);'
'        high_fraction(k) = min(1,max(0,(stop_area-start_area)/periods));'
'    else'
'        high_fraction(k) = duty;'
'    end'
'end'
'end'};
mf_chart(path,'LEDC high time',[300 18 465 112],sprintf('%s\n',lines{:}),{'Ts_plant','pwm_switching','pwm_Hz','pwm_bits'});

lines = {
'function vUVW = bridge_voltage(high_fraction, enable, vdc)'
'%#codegen'
'% An ideal two-level bridge feeding a balanced three-wire star.'
'vUVW = zeros(3,1);'
'if enable ~= 0'
'    v_pole = vdc * high_fraction;'
'    v_neutral = (v_pole(1) + v_pole(2) + v_pole(3)) / 3;'
'    vUVW = v_pole - v_neutral;'
'end'
'% Disabled approximation: equal pole potentials, zero line-line voltages.'
'end'};
mf_chart(path,'Bridge and floating neutral',[550 170 735 290],sprintf('%s\n',lines{:}),{});
add_block('simulink/Sinks/Out1',[path '/vUVW'],'Port','1','Position',[800 222 830 236]);

wire(path,'counts/1','Hold counts/1');
wire(path,'enable/1','Hold enable/1');
wire(path,'vdc/1','Hold vdc/1');
wire(path,'Hold counts/1','LEDC high time/1');
wire(path,'Carrier phase/1','LEDC high time/2');
wire(path,'LEDC high time/2','Carrier phase/1');
wire(path,'LEDC high time/1','Bridge and floating neutral/1');
wire(path,'Hold enable/1','Bridge and floating neutral/2');
wire(path,'Hold vdc/1','Bridge and floating neutral/3');
wire(path,'Bridge and floating neutral/1','vUVW/1');
end
