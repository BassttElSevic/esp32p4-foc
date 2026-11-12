function build_model(outdir)
% Generate an editable SLX with the adjoining MATLAB file as its data source.
name = 'esp32p4_foc';
if bdIsLoaded(name), close_system(name,0); end
load_system('simulink');
new_system(name);
cleanup = onCleanup(@() close_system(name,0));
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','Ts_plant','SolverMode','SingleTasking','StopTime','sim_end_s', ...
    'SignalLogging','on','SignalLoggingName','logsout', ...
    'ReturnWorkspaceOutputs','on','SaveTime','off','SaveOutput','off','SaveState','off');
set_param(name,'Description',sprintf(['ESP32-P4 M2: cascaded electrical speed and dq current control.\n' ...
    'Firmware BassttElSevic/esp32p4-foc @ c20fad4. Starts calibrated in RUN.\n' ...
    'Illustrative plant parameters are in the Model Workspace. Open the FOC subsystem to inspect the loops.']));
addpath(outdir);
mws = get_param(name,'ModelWorkspace');
mws.DataSource = 'MATLAB File';
mws.FileName = 'esp32p4_foc_data.m';
reload(mws);

part('FOC controller',[355 116 605 398],@build_control);
part('PWM and inverter',[903 105 1072 222],@build_drive);
part('PMSM and shaft',[1152 103 1345 268],@build_motor);
part('Current and position sensors',[285 542 522 640],@build_sensors);
src('Step','Speed command Hz',[30 126 126 165], ...
    'Time','step_speed_s','Before','start_Hz','After','target_Hz','SampleTime','Ts_ctrl');
src('Constant','Current limit mA',[31 190 127 221],'Value','Imax','SampleTime','Ts_ctrl');
src('Constant','Encoder available',[30 357 127 390],'Value','enc_ok','SampleTime','Ts_ctrl');
src('Constant','DC link V',[740 60 818 90],'Value','V_dc','SampleTime','Ts_plant');
src('Step','Load torque Nm',[1065 285 1135 322], ...
    'Time','step_load_s','Before','start_load_Nm','After','load_Nm','SampleTime','Ts_plant');

wire(name,'Speed command Hz/1','FOC controller/1');
wire(name,'Current limit mA/1','FOC controller/2');
wire(name,'Encoder available/1','FOC controller/5');
wire(name,'FOC controller/1','PWM and inverter/1');
wire(name,'FOC controller/2','PWM and inverter/2');
wire(name,'DC link V/1','PWM and inverter/3');
wire(name,'PWM and inverter/1','PMSM and shaft/1');
wire(name,'Load torque Nm/1','PMSM and shaft/2');
publish('I_UVW','PMSM and shaft/1',[1390 108 1487 134]);
publish('ANGLE_M','PMSM and shaft/2',[1390 147 1487 173]);
receive('I_UVW','Measured windings',[110 547 215 574],'Current and position sensors/1');
receive('ANGLE_M','Rotor position',[110 603 215 630],'Current and position sensors/2');
publish('I_UV_ADC','Current and position sensors/1',[560 546 665 572]);
publish('AS5600','Current and position sensors/2',[560 608 665 634]);
receive('I_UV_ADC','Current feedback',[135 255 245 283],'FOC controller/3');
receive('AS5600','Encoder feedback',[135 304 245 332],'FOC controller/4');
tags = {'W_REF','W_EST','IDQ_REF','IDQ_FB','VDQ','OCP'};
for k=1:numel(tags)
    y = 212+35*(k-1);
    publish(tags{k},sprintf('FOC controller/%d',k+2),[673 y 783 y+23]);
end
publish('W_SHAFT','PMSM and shaft/3',[1390 183 1487 209]);
terminate('True dq output','PMSM and shaft/4',[1390 224 1410 244]);
terminate('Torque output','PMSM and shaft/5',[1390 256 1410 276]);
% Monitor in engineering units; resample the shaft channel only for the scope.
scope('Speed  electrical rad per s',[1025 477 1135 555],3);
receive('W_REF','Scope speed ref',[769 474 859 497],'Speed  electrical rad per s/1');
receive('W_EST','Scope speed est',[769 510 859 533],'Speed  electrical rad per s/2');
add_block('simulink/Math Operations/Gain',[name '/Shaft to encoder electrical'], ...
    'Gain','s_enc*p_pairs','Position',[891 550 958 583]);
receive('W_SHAFT','Scope shaft speed',[769 557 859 580],'Shaft to encoder electrical/1');
wire(name,'Shaft to encoder electrical/1','Speed  electrical rad per s/3');
scope('Current  dq in mA',[1025 632 1135 702],2);
receive('IDQ_REF','Scope dq ref',[769 631 859 654],'Current  dq in mA/1');
receive('IDQ_FB','Scope dq measured',[769 676 859 699],'Current  dq in mA/2');
scope('Protection and modulation',[1310 512 1450 585],2);
receive('OCP','Scope protection',[1188 511 1262 535],'Protection and modulation/1');
receive('VDQ','Scope voltage dq',[1188 562 1262 585],'Protection and modulation/2');
% Rate-appropriate logging: no downsampling of current/PWM in carrier mode.
logport('FOC controller',{'counts','enable','omega_ref','omega_hat','idq_ref_mA','idq_f_mA','vdq','trip'});
logport('PMSM and shaft',{'iUVW_A','theta_m','omega_m','idq_A','Te_Nm'});
logport('Current and position sensors',{'uv_mA','theta_enc'});
logport('PWM and inverter',{'vUVW'});
logport('Load torque Nm',{'load_torque'});
annotate('ESP32-P4 · DENG FOC 2208  |  速度外环与 dq 电流内环',[22 14 590 42],17);
annotate('双击 FOC controller 查看速度 PI、两轴电流 PI、反馈与限幅。',[22 53 570 80],11);
annotate('电流与位置反馈',[45 476 406 510],14);
annotate('观测量  |  给定、估计与电机真值',[763 409 1385 447],14);
annotate('mA 与电 rad/s。控制、采样、电机和试验参数来自 esp32p4_foc_data.m。',[26 709 802 740],10);
set_param(name,'ZoomFactor','FitSystem');
set_param(name,'SimulationCommand','update');
save_system(name,fullfile(outdir,[name '.slx']));
fprintf('MODEL_BUILT %s\n',fullfile(outdir,[name '.slx']));

    function part(title,pos,builder)
        path = [name '/' title];
        add_block('built-in/Subsystem',path,'Position',pos);
        builder(path);
        set_param(path,'ShowPortLabels','FromPortBlockName');
    end
    function src(type,title,pos,varargin)
        add_block(['simulink/Sources/' type],[name '/' title],'Position',pos,varargin{:});
    end
    function publish(tag,source,pos)
        id=['Transmit ' tag];
        add_block('simulink/Signal Routing/Goto',[name '/' id], ...
            'GotoTag',tag,'TagVisibility','local','ShowName','off','Position',pos);
        wire(name,source,[id '/1']);
    end
    function receive(tag,id,pos,dest)
        add_block('simulink/Signal Routing/From',[name '/' id], ...
            'GotoTag',tag,'ShowName','off','Position',pos);
        wire(name,[id '/1'],dest);
    end
    function terminate(id,srcport,pos)
        add_block('simulink/Sinks/Terminator',[name '/' id],'Position',pos);
        wire(name,srcport,[id '/1']);
    end
    function scope(id,pos,num)
        add_block('simulink/Sinks/Scope',[name '/' id],'NumInputPorts',num2str(num),'Position',pos);
    end
    function logport(id,fields)
        ph=get_param([name '/' id],'PortHandles');
        for n=1:numel(fields)
            set_param(ph.Outport(n),'DataLogging','on','DataLoggingNameMode','Custom','DataLoggingName',fields{n});
        end
    end
    function annotate(title,pos,sz)
        obj=Simulink.Annotation(name,title);
        obj.FontSize=sz;
        obj.Position=pos(1:2);
    end
end
