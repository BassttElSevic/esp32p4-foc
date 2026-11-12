function build_motor(path)
% Balanced sinusoidal PMSM in rotor dq coordinates, mechanical SI units.
add_block('simulink/Sources/In1',[path '/vUVW'],'Port','1','Position',[20 54 50 68]);
add_block('simulink/Sources/In1',[path '/load_torque'],'Port','2','Position',[20 142 50 156]);
add_block('simulink/Discrete/Zero-Order Hold',[path '/Hold load'],'SampleTime','Ts_plant','Position',[105 131 182 167]);

lines = {
'function vAB = stator_voltage(vUVW)'
'%#codegen'
'% Amplitude-invariant Clarke transform; common mode cancels.'
'vAB = [(2*vUVW(1)-vUVW(2)-vUVW(3))/3; (vUVW(2)-vUVW(3))/sqrt(3)];'
'end'};
mf_chart(path,'Stator voltage Clarke',[95 34 240 89],sprintf('%s\n',lines{:}),{});

lines = {
'function x_next = advance_pmsm(vAB, load_torque, x)'
'%#codegen'
'% x = [id (A); iq (A); mechanical angle (rad); mechanical speed (rad/s)].'
'% Hold stationary-frame voltage and load over the integration interval.'
'h = Ts_plant;'
'k1 = derivative(x,vAB,load_torque,p_pairs,R_s,L_d,L_q,psi_pm,J_m,B_m);'
'k2 = derivative(x+h*k1/2,vAB,load_torque,p_pairs,R_s,L_d,L_q,psi_pm,J_m,B_m);'
'k3 = derivative(x+h*k2/2,vAB,load_torque,p_pairs,R_s,L_d,L_q,psi_pm,J_m,B_m);'
'k4 = derivative(x+h*k3,vAB,load_torque,p_pairs,R_s,L_d,L_q,psi_pm,J_m,B_m);'
'x_next = x + (h/6)*(k1+2*k2+2*k3+k4);'
'end'
''
'function dx = derivative(x,vAB,load_torque,p,R,Ld,Lq,psi,J,B)'
'id = x(1); iq = x(2); theta = x(3); omega = x(4);'
'ct = cos(p*theta); st = sin(p*theta);'
'vd =  ct*vAB(1) + st*vAB(2);'
'vq = -st*vAB(1) + ct*vAB(2);'
'we = p*omega;'
'did = (vd - R*id + we*Lq*iq)/Ld;'
'diq = (vq - R*iq - we*(Ld*id + psi))/Lq;'
'Te = 1.5*p*(psi + (Ld-Lq)*id)*iq;'
'dx = [did; diq; omega; (Te - load_torque - B*omega)/J];'
'end'};
params = {'Ts_plant','p_pairs','R_s','L_d','L_q','psi_pm','J_m','B_m'};
mf_chart(path,'PMSM equations and RK4',[305 48 490 170],sprintf('%s\n',lines{:}),params);
add_block('simulink/Discrete/Unit Delay',[path '/Physical state'],'SampleTime','Ts_plant','InitialCondition','motor_x0','Position',[540 146 625 206]);

lines = {
'function [iUVW,theta,omega,idq,Te] = outputs(x)'
'%#codegen'
'id = x(1); iq = x(2); theta = x(3); omega = x(4);'
'ct = cos(p_pairs*theta); st = sin(p_pairs*theta);'
'i_alpha = ct*id - st*iq;'
'i_beta = st*id + ct*iq;'
'iUVW = [i_alpha; -.5*i_alpha + sqrt(3)*.5*i_beta; -.5*i_alpha - sqrt(3)*.5*i_beta];'
'idq = [id; iq];'
'Te = 1.5*p_pairs*(psi_pm+(L_d-L_q)*id)*iq;'
'end'};
mf_chart(path,'Phase currents and shaft outputs',[700 50 880 254],sprintf('%s\n',lines{:}),{'p_pairs','L_d','L_q','psi_pm'});
names = {'iUVW_A','theta_m_rad','omega_m_radps','idq_A','Te_Nm'};
for k = 1:numel(names)
    y = 60 + 42*(k-1);
    add_block('simulink/Sinks/Out1',[path '/' names{k}],'Port',num2str(k),'Position',[945 y 975 y+14]);
    wire(path,['Phase currents and shaft outputs/' num2str(k)],[names{k} '/1']);
end
wire(path,'vUVW/1','Stator voltage Clarke/1');
wire(path,'Stator voltage Clarke/1','PMSM equations and RK4/1');
wire(path,'load_torque/1','Hold load/1');
wire(path,'Hold load/1','PMSM equations and RK4/2');
wire(path,'Physical state/1','PMSM equations and RK4/3');
wire(path,'PMSM equations and RK4/1','Physical state/1');
wire(path,'Physical state/1','Phase currents and shaft outputs/1');
end
