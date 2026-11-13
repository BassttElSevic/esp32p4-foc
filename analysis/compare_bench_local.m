function metrics = compare_bench_local(outdir)
%COMPARE_BENCH_LOCAL Fit the initial recordings, then predict all bench runs.
% From the repository root: addpath('analysis'); compare_bench_local
root = fileparts(fileparts(mfilename('fullpath')));
data = fullfile(root,'bench-results');
if nargin<1, outdir = fullfile(data,'local-model'); end
if ~isfolder(outdir), mkdir(outdir); end
fit = identify_bench_local(outdir);
% Use the same encoder scaling for the observations as the simulated sensor.
run(fullfile(root,'simulink','esp32p4_foc_data.m'));
cfg = fullfile(data,'tune','parameters.csv');
opts = setvartype(detectImportOptions(cfg),{'folder','group'},'string');
settings = readtable(cfg,opts);
summary = {};
waves = {};
for n=1:height(settings)
    folder = settings.folder(n);
    isTraining = folder==".";
    split = "validation";
    if isTraining, split="train"; end
    tuning = table2struct(settings(n,{'alpha_i','Kp_i','Ki_i','Kp_w'}));
    files = dir(fullfile(data,folder,'*.csv'));
    for k=1:numel(files)
        name = string(files(k).name);
        source = name;
        if ~isTraining, source=folder+"/"+name; end
        rows = readtable(fullfile(files(k).folder,name),'TextType','string');
        count = height(rows);
        mode = rows.mode(1);
        hz = rows.frequency_Hz(1); cycles = rows.cycles(1); amp = rows.requested_amplitude(1);
        t = rows.t_us*1e-6;
        assert(all(diff(t)>0) && all(rows.seq==(0:count-1)') && all(rows.expected_samples==count));
        assert(all(rows.mode==mode & rows.frequency_Hz==hz & rows.cycles==cycles & ...
            rows.requested_amplitude==amp & rows.board_end==rows.board_end(1)));
        fprintf('%s: %s -- nominal and identified plant\n',split,source);
        [nom,ns] = simulate_bench_sine(rows,tuning);
        if isTraining && mode=="speed"
            [loc,ls] = simulate_bench_sine(rows,tuning,fit,fullfile(outdir,'bench_local_2208.slx'));
        else
            [loc,ls] = simulate_bench_sine(rows,tuning,fit);
        end
        assert(isequal(nom.t_us,rows.t_us) && isequal(loc.t_us,rows.t_us));
        channel = 'iq_filtered_mA';
        if mode=="speed", channel = 'omega_filtered_e_rad_s'; end
        measured = rows.(channel); nominal = nom.(channel); local = loc.(channel);
        angle = unwrap(rows.angle_code*(2*pi/(2^enc_bits)));
        angle = angle-angle(1);
        nominalAngle = nom.theta_m_rad-nom.theta_m_rad(1);
        localAngle = loc.theta_m_rad-loc.theta_m_rad(1);
        common = isfinite(nominal) & isfinite(local);
        assert(isequal(isfinite(nominal),isfinite(nominalAngle)) && ...
            isequal(isfinite(local),isfinite(localAngle)));
        % The same sample mask is mandatory for cross-model RMSE. Report the
        % local model's error on its entire measured overlap as well.
        labels = ["measured";"nominal";"local"];
        out = [measured,nominal,local];
        angles = [angle,nominalAngle,localAngle];
        currents = [rows.iq_filtered_mA,nom.iq_filtered_mA,loc.iq_filtered_mA];
        ends = [rows.board_end(1);ns.stop_reason;ls.stop_reason];
        stats = table(source,settings.group(n),split,mode,hz,amp,cycles,rows.board_end(1),count,t(end), ...
            ns.stop_reason,ns.stop_detail,ns.stop_s,ls.stop_reason,ls.stop_detail,ls.stop_s, ...
            nnz(common),nnz(isfinite(nominal)),nnz(isfinite(local)), ...
            rmsError(nominal,measured,common),rmsError(local,measured,common), ...
            rmsError(local,measured,isfinite(local)), ...
            rmsError(nominalAngle,angle,common),rmsError(localAngle,angle,common), ...
            rmsError(currents(:,2),currents(:,1),common),rmsError(currents(:,3),currents(:,1),common), ...
            'VariableNames',{'file','group','split','mode','frequency_Hz','amplitude','cycles','bench_end', ...
            'bench_n','bench_last_s','nominal_end','nominal_branch','nominal_trip_s','local_end', ...
            'local_branch','local_trip_s','common_n','nominal_n','local_n','nominal_rmse_common', ...
            'local_rmse_common','local_rmse_own','nominal_angle_rmse_rad_common','local_angle_rmse_rad_common', ...
            'nominal_iq_rmse_mA_common','local_iq_rmse_mA_common'});
        % Peak amplitudes are diagnostic estimates on the final two complete
        % programmed cycles. Fault runs do not yield steady-state estimates.
        for j=1:3
            field = labels(j);
            [gain,r2,ang,iq] = finalHarmonics(t,hz,cycles,rows.reference, ...
                out(:,j),angles(:,j),currents(:,j),ends(j));
            stats.(field+"_gain") = gain;
            stats.(field+"_R2") = r2;
            stats.(field+"_angle_peak_deg") = rad2deg(ang);
            stats.(field+"_iq_peak_mA") = iq;
        end
        summary{end+1} = stats; %#ok<AGROW>
        waves{end+1} = table(repmat(source,count,1),rows.t_us,rows.reference,measured,nominal,local, ...
            angle,nominalAngle,localAngle,currents(:,1),currents(:,2),currents(:,3), ...
            rows.omega_filtered_e_rad_s,nom.omega_filtered_e_rad_s,loc.omega_filtered_e_rad_s, ...
            'VariableNames',{'file','t_us','reference','measured','nominal','local','angle_measured_rad', ...
            'angle_nominal_rad','angle_local_rad','iq_measured_mA','iq_nominal_mA','iq_local_mA', ...
            'omega_measured_e_rad_s','omega_nominal_e_rad_s','omega_local_e_rad_s'}); %#ok<AGROW>
    end
end
metrics = vertcat(summary{:});
writetable(metrics,fullfile(outdir,'metrics.csv'));
writetable(vertcat(waves{:}),fullfile(outdir,'waveforms.csv'));
end

function value = rmsError(y,actual,keep)
value = NaN;
if any(keep), value = sqrt(mean((y(keep)-actual(keep)).^2)); end
end

function [gain,r2,ang,iq] = finalHarmonics(t,hz,cycles,ref,y,theta,current,ending)
gain=NaN; r2=NaN; ang=NaN; iq=NaN;
if ending~="ok", return; end
turns = t*hz;
for start = cycles-2:cycles-1
    tt = turns(turns>=start & turns<start+1);
    if numel(tt)<12 || tt(end)-tt(1)<0.85 || any(diff(tt)>0.12), return; end
end
mask = turns>=cycles-2 & turns<cycles;
if any(~isfinite(y(mask))), return; end
phase = 2*pi*turns(mask);
A = [sin(phase),cos(phase),ones(size(phase))];
cr = A\ref(mask); cy = A\y(mask); ci = A\current(mask);
ca = [A, t(mask)-mean(t(mask))]\theta(mask);
gain = hypot(cy(1),cy(2))/hypot(cr(1),cr(2));
ang = hypot(ca(1),ca(2));
iq = hypot(ci(1),ci(2));
var = sum((y(mask)-mean(y(mask))).^2);
if var>0, r2 = 1-sum((y(mask)-A*cy).^2)/var; end
end
