function build_control(path)
% Generate the calibrated, running M2 control path from firmware/foc/main/main.c.
% The plant-side acquisition model supplies the two corrected phase currents.

inNames = {'f_Hz','Imax_mA','uv_mA','theta_enc','enc_ok'};
inY = [85 197 455 703 763];
for k = 1:numel(inNames)
    y = inY(k);
    add_block('simulink/Ports & Subsystems/In1',[path '/' inNames{k}], ...
        'Port',num2str(k),'Position',[22 y-7 52 y+7]);
end

% Top band: cascaded controllers and normalized voltage vector to PWM codes.
sat('Console_Hz', [104 61 193 106], 'f_console_max_Hz');
sat('Closed_loop_Hz', [231 61 330 106], 'f_closed_max_Hz');
gain('Electrical_rad_s',[367 63 436 103], '2*pi');
errsum('Speed_error',[540 66 575 100]);
sat('Console_mA',[240 173 340 218],'I_console_max_mA');
chart('Speed_PI',[654 57 810 145],pi_law('Kp_w','Ki_w',true), ...
    {'Ts_ctrl','Kp_w','Ki_w','ctrl_dt_max_s'});
gain('Torque_direction',[860 74 938 115],'torque_sign*s_enc');
errsum('Q_error',[1060 79 1095 113]);
currentLaw = pi_law('Kp_i','Ki_i',false);
currentParams = {'Ts_ctrl','Kp_i','Ki_i','mod_limit','ctrl_dt_max_s'};
chart('Q_current_PI',[1172 70 1314 142],currentLaw,currentParams);
add_block('simulink/Sources/Constant',[path '/D_reference'], ...
    'Value','id_target_mA','Position',[858 243 905 279]);
errsum('D_error',[1060 244 1095 278]);
chart('D_current_PI',[1172 235 1314 307],currentLaw,currentParams);
chart('DQ_circle_limit',[1400 148 1535 222],[
    "function vdq = fcn(uq,ud)"
    "r = sqrt(ud*ud + uq*uq);"
    "vdq = [ud;uq];"
    "if r > mod_limit"
    "    vdq = (mod_limit/r)*vdq;"
    "end"
    "end"],{'mod_limit'});
chart('Inverse_Park',[1615 151 1748 225],[
    "function ab = fcn(vdq,theta)"
    "c = cos(theta); s = sin(theta);"
    "ab = [vdq(1)*c-vdq(2)*s;vdq(1)*s+vdq(2)*c];"
    "end"],{});
chart('Inverse_Clarke',[1815 152 1952 216],[
    "function uvw = fcn(ab)"
    "a = ab(1); b = ab(2);"
    "uvw = [a; -0.5*a+0.5*sqrt(3)*b; -0.5*a-0.5*sqrt(3)*b];"
    "end"],{});
chart('SPWM_quantize',[2012 152 2130 216],[
    "function counts = fcn(uvw)"
    "counts = floor((2^pwm_bits-1)*max(0,min(1,duty_mid+uvw))+0.5);"
    "end"],{'pwm_bits','duty_mid'});
chart('Neutral_on_trip',[2210 152 2350 226],[
    "function counts = fcn(raw,trip)"
    "if trip ~= 0"
    "    counts = ones(3,1)*floor((2^pwm_bits-1)*duty_mid+0.5);"
    "else"
    "    counts = raw;"
    "end"
    "end"],{'pwm_bits','duty_mid'});

% Lower band: distinct measured signals from calibrated ADC and AS5600.
chart('Two_phase_Clarke',[205 421 345 486],[
    "function ab = fcn(uv)"
    "ab = [uv(1);(uv(1)+2*uv(2))/sqrt(3)];"
    "end"],{});
chart('Current_Park',[425 421 570 495],[
    "function dq = fcn(ab,theta)"
    "c = cos(theta); s = sin(theta);"
    "dq = [ab(1)*c+ab(2)*s;-ab(1)*s+ab(2)*c];"
    "end"],{});
chart('DQ_feedback_LPF',[655 401 838 509],[
    "function [id,iq,idq] = fcn(dq,trip)"
    "persistent filtered"
    "if isempty(filtered), filtered = zeros(2,1); end"
    "if trip == 0"
    "    filtered = filtered + alpha_i*(dq-filtered);"
    "end"
    "idq = filtered; id = filtered(1); iq = filtered(2);"
    "end"],{'alpha_i'});
chart('Raw_phase_OCP',[216 540 385 622],[
    "function [trip,en] = fcn(uv)"
    "persistent hits tripped elapsedTicks"
    "if isempty(hits), hits = 0; tripped = false; elapsedTicks = 0; end"
    "if tripped"
    "    elapsedTicks = elapsedTicks+1;"
    "else"
    "    peak = max([abs(uv(1));abs(uv(2));abs(-uv(1)-uv(2))]);"
    "    if peak > ocp_limit_mA, hits = hits+1; else, hits = 0; end"
    "    if hits >= ocp_samples, tripped = true; elapsedTicks = 0; end"
    "end"
    "trip = double(tripped);"
    "en = double(~tripped || round(elapsedTicks*Ts_ctrl*1e6) < round(ocp_disable_s*1e6));"
    "end"],{'Ts_ctrl','ocp_limit_mA','ocp_samples','ocp_disable_s'});

chart('AS5600_estimator',[211 687 397 789],[
    "function [omega,theta] = fcn(enc,ok,trip)"
    "persistent previous estimate ticks initialized"
    "if isempty(ticks)"
    "    previous = 0; estimate = 0; ticks = 0; initialized = false;"
    "end"
    "if trip == 0"
    "    ticks = ticks+1;"
    "    if ok ~= 0"
    "        delta = enc-previous;"
    "        if delta > pi, delta = delta-2*pi; end"
    "        if delta < -pi, delta = delta+2*pi; end"
    "        dt_us = round(ticks*Ts_ctrl*1e6);"
    "        if initialized && dt_us > enc_dt_min_us && dt_us < enc_dt_max_us"
    "            raw = p_pairs*delta/(dt_us*1e-6);"
    "            estimate = estimate + alpha_w*(raw-estimate);"
    "        end"
    "        previous = enc; initialized = true; ticks = 0;"
    "    else"
    "        estimate = enc_fail_decay*estimate;"
    "    end"
    "end"
    "omega = estimate;"
    "theta = mod(mod(s_enc*p_pairs*previous+theta_off,2*pi)+omega*min(ctrl_dt_max_s,max(0,Ts_ctrl)),2*pi);"
    "end"],{'Ts_ctrl','p_pairs','s_enc','theta_off','enc_dt_min_us','enc_dt_max_us', ...
    'alpha_w','enc_fail_decay','ctrl_dt_max_s'});

chart('DQ_reference',[1060 386 1182 456],[
    "function ref = fcn(id,iq)"
    "ref = [id;iq];"
    "end"],{});

% A local tag identifies the same scalar or vector at every receiver.
publish('W_REFERENCE',[464 17 577 43],'Electrical_rad_s/1');
publish('VDQ_COMMAND',[1512 78 1632 104],'DQ_circle_limit/1');
publish('IQ_REF',[936 175 1022 201],'Torque_direction/1');
publish('ID_F',[908 369 986 395],'DQ_feedback_LPF/1');
publish('IQ_F',[908 425 986 451],'DQ_feedback_LPF/2');
publish('IDQ_F',[908 483 986 509],'DQ_feedback_LPF/3');
publish('TRIP',[454 539 545 565],'Raw_phase_OCP/1');
publish('DRIVE_ENABLE',[454 599 594 625],'Raw_phase_OCP/2');
publish('W_ESTIMATE',[467 687 585 713],'AS5600_estimator/1');
publish('PARK_ANGLE',[467 756 585 782],'AS5600_estimator/2');
publish('IDQ_REF',[1220 407 1310 433],'DQ_reference/1');

receive('W_ESTIMATE','Speed_FB',[430 120 532 144],'Speed_error/2');
receive('TRIP','Speed_PI_stop',[480 227 574 251],'Speed_PI/3');
receive('IQ_F','Q_FB',[978 124 1041 148],'Q_error/2');
receive('ID_F','D_FB',[978 297 1041 321],'D_error/2');
receive('TRIP','Q_PI_stop',[1080 159 1151 183],'Q_current_PI/2');
receive('TRIP','D_PI_stop',[1080 322 1151 346],'D_current_PI/2');
receive('PARK_ANGLE','Mod_angle',[1470 264 1588 288],'Inverse_Park/2');
receive('TRIP','PWM_trip',[2102 269 2185 293],'Neutral_on_trip/2');
receive('PARK_ANGLE','ADC_angle',[295 490 410 514],'Current_Park/2');
receive('TRIP','LPF_stop',[569 529 640 553],'DQ_feedback_LPF/2');
receive('TRIP','Encoder_stop',[98 813 174 837],'AS5600_estimator/3');
receive('IQ_REF','Reference_q',[932 399 1008 423],'DQ_reference/2');

connections = {
    'f_Hz/1','Console_Hz/1'; 'Console_Hz/1','Closed_loop_Hz/1';
    'Closed_loop_Hz/1','Electrical_rad_s/1'; 'Electrical_rad_s/1','Speed_error/1';
    'Speed_error/1','Speed_PI/1'; 'Imax_mA/1','Console_mA/1';
    'Console_mA/1','Speed_PI/2'; 'Speed_PI/1','Torque_direction/1';
    'Torque_direction/1','Q_error/1'; 'Q_error/1','Q_current_PI/1';
    'D_reference/1','D_error/1'; 'D_reference/1','DQ_reference/1';
    'D_error/1','D_current_PI/1'; 'Q_current_PI/1','DQ_circle_limit/1';
    'D_current_PI/1','DQ_circle_limit/2'; 'DQ_circle_limit/1','Inverse_Park/1';
    'Inverse_Park/1','Inverse_Clarke/1'; 'Inverse_Clarke/1','SPWM_quantize/1';
    'SPWM_quantize/1','Neutral_on_trip/1'; 'uv_mA/1','Two_phase_Clarke/1';
    'uv_mA/1','Raw_phase_OCP/1'; 'Two_phase_Clarke/1','Current_Park/1';
    'Current_Park/1','DQ_feedback_LPF/1'; 'theta_enc/1','AS5600_estimator/1';
    'enc_ok/1','AS5600_estimator/2'};
for k = 1:size(connections,1)
    wire(path,connections{k,1},connections{k,2});
end

outNames = {'pwm_counts','enable','w_ref','w_hat','idq_ref_mA','idq_f_mA','vdq','trip'};
outTags = {'','DRIVE_ENABLE','W_REFERENCE','W_ESTIMATE','IDQ_REF','IDQ_F','VDQ_COMMAND','TRIP'};
for k = 1:numel(outNames)
    y = 189+(k-1)*60;
    add_block('simulink/Ports & Subsystems/Out1',[path '/' outNames{k}], ...
        'Port',num2str(k),'Position',[2510 y-7 2540 y+7]);
    if k == 1
        wire(path,'Neutral_on_trip/1','pwm_counts/1');
    else
        receive(outTags{k},['Read_' outNames{k}],[2335 y-13 2450 y+13],[outNames{k} '/1']);
    end
end

    function chart(name,pos,source,params)
        mf_chart(path,name,pos,char(join(source,newline)),params);
    end
    function sat(name,pos,limit)
        add_block('simulink/Discontinuities/Saturation',[path '/' name], ...
            'Position',pos,'LowerLimit',['-' limit],'UpperLimit',limit);
    end
    function gain(name,pos,value)
        add_block('simulink/Math Operations/Gain',[path '/' name], ...
            'Position',pos,'Gain',value);
    end
    function errsum(name,pos)
        add_block('simulink/Math Operations/Sum',[path '/' name],'Inputs','+-', ...
            'IconShape','round','Position',pos);
    end
    function publish(tag,pos,src)
        name = ['Write_' tag];
        add_block('simulink/Signal Routing/Goto',[path '/' name],'GotoTag',tag, ...
            'TagVisibility','local','ShowName','off','Position',pos);
        wire(path,src,[name '/1']);
    end
    function receive(tag,name,pos,dst)
        add_block('simulink/Signal Routing/From',[path '/' name],'GotoTag',tag, ...
            'ShowName','off','Position',pos);
        wire(path,[name '/1'],dst);
    end
end

function code = pi_law(kp,ki,speed)
% Output uses the old integral. The ordered comparisons also reproduce the
% firmware behavior for the negative current limits accepted by cmd_i.
if speed
    signature = "function u = fcn(e,L,trip)";
    limit = "% L is the signed console argument after command saturation.";
else
    signature = "function u = fcn(e,trip)";
    limit = "L = mod_limit;";
end
code = [signature
    "persistent x held"
    "if isempty(x), x = 0; held = 0; end"
    limit
    "if trip ~= 0, u = held; return; end"
    string(sprintf('b = x + %s*e;',kp))
    string(sprintf('delta = %s*e*min(ctrl_dt_max_s,max(0,Ts_ctrl));',ki))
    "if b > L"
    "    u = L; if e < 0, x = x+delta; end"
    "elseif b < -L"
    "    u = -L; if e > 0, x = x+delta; end"
    "else"
    "    u = b; x = x+delta;"
    "end"
    "if x > L, x = L; end"
    "if x < -L, x = -L; end"
    "held = u;"
    "end"];
end
