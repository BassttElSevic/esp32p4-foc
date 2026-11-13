function metrics = compare_bench_sine(outdir)
%COMPARE_BENCH_SINE Run the nominal bench model for the recorded sine trials.
% From the repository root: addpath('analysis'); compare_bench_sine
base = fileparts(fileparts(mfilename('fullpath')));
recordings = fullfile(base,'bench-results');
if nargin < 1, outdir = fullfile(recordings,'comparison'); end
if ~isfolder(outdir), mkdir(outdir); end
cfgFile = fullfile(recordings,'tune','parameters.csv');
opts = detectImportOptions(cfgFile);
opts = setvartype(opts,{'folder','group'},'string');
configs = readtable(cfgFile,opts);
trials = {};
summaries = {};
waveforms = {};
for c = 1:height(configs)
    folder = configs.folder(c);
    listing = dir(fullfile(recordings,folder,'*.csv'));
    assert(~isempty(listing),'No bench captures in %s.',folder);
    tuning = table2struct(configs(c,{'alpha_i','Kp_i','Ki_i','Kp_w'}));
    for k = 1:numel(listing)
        filename = listing(k).name;
        if folder == ".", source = string(filename); else, source = folder + "/" + filename; end
        rows = readtable(fullfile(listing(k).folder,filename),'TextType','string');
        [d, time] = capture(rows,configs.group(c));
        fprintf('Simulating %s (%d measured points)\n',source,height(rows));
        [model,meta] = simulate_bench_sine(rows,tuning);
        assert(isequal(model.t_us,rows.t_us),'Simulation timestamps differ from the recording.');
        if d.mode == "current"
            unit = "mA";
            d.predicted = model.iq_filtered_mA;
        else
            unit = "electrical_rad_s";
            d.predicted = model.omega_filtered_e_rad_s;
        end
        d.simEndReason = string(meta.stop_reason);
        d.simEndTime = meta.stop_s;
        trials{end+1} = d; %#ok<AGROW>
        valid = isfinite(d.predicted);
        modelLimited = double(model.limited);
        modelLimited(~valid) = NaN;
        if any(valid)
            err = d.predicted(valid)-d.measured(valid);
            rmse = sqrt(mean(err.^2));
            commonEnd = time(find(valid,1,'last'));
        else
            rmse = NaN;
            commonEnd = NaN;
        end
        [mg,mp,mr,mq] = sineFit(d,time,d.ref,d.measured,d.endReason,rows.limited);
        [sg,sp,sr,sq] = sineFit(d,time,model.reference,d.predicted,d.simEndReason,modelLimited);
        summaries{end+1} = table(source,d.group,d.mode,d.hz,d.amp,unit,d.cycles,d.endReason, ...
            time(end),height(rows),sum(rows.limited),d.simEndReason,string(meta.stop_detail),d.simEndTime,sum(valid),commonEnd, ...
            sum(modelLimited==1),rmse,mg,mp,mr,mq,sg,sp,sr,sq,'VariableNames', ...
            {'file','group','mode','frequency_Hz','amplitude','unit','cycles','bench_end', ...
            'last_bench_s','bench_samples','bench_limited_rows','model_end','model_trip_branch','model_trip_s', ...
            'paired_samples','last_paired_s','model_limited_paired_rows','paired_rmse', ...
            'bench_gain','bench_phase_deg','bench_fit_R2','bench_fit_status', ...
            'model_gain','model_phase_deg','model_fit_R2','model_fit_status'}); %#ok<AGROW>
        waveforms{end+1} = table(repmat(source,height(rows),1),rows.t_us,d.ref,model.reference, ...
            d.measured,d.predicted,rows.limited,modelLimited, ...
            rows.iq_filtered_mA,model.iq_filtered_mA,rows.omega_filtered_e_rad_s,model.omega_filtered_e_rad_s,'VariableNames', ...
            {'file','t_us','bench_reference','model_reference','bench_feedback', ...
            'model_feedback','bench_limited','model_limited','bench_iq_filtered_mA','model_iq_filtered_mA', ...
            'bench_omega_filtered_e_rad_s','model_omega_filtered_e_rad_s'}); %#ok<AGROW>
    end
end
metrics = vertcat(summaries{:});
plot_bench_comparison([trials{:}],outdir);
writetable(metrics,fullfile(outdir,'metrics.csv'));
writetable(vertcat(waveforms{:}),fullfile(outdir,'aligned_waveforms.csv'));
end

function [d,t] = capture(rows,group)
d.group = group;
d.mode = rows.mode(1);
d.hz = rows.frequency_Hz(1);
d.amp = rows.requested_amplitude(1);
d.cycles = rows.cycles(1);
d.endReason = rows.board_end(1);
t = rows.t_us*1e-6;
d.t = t;
d.ref = rows.reference;
assert(all(rows.seq == (0:height(rows)-1)') && all(diff(t)>0) && ...
    all(rows.expected_samples == height(rows)),'Invalid sequence or row count.');
assert(all(rows.mode == d.mode & rows.board_end == d.endReason & rows.frequency_Hz == d.hz & ...
    rows.requested_amplitude == d.amp & rows.cycles == d.cycles),'Trial metadata changed.');
assert(all(abs(d.ref - d.amp*sin(2*pi*d.hz*t)) < 2e-5*max(1,d.amp)), ...
    'Recorded reference does not match the declared sine at the board timestamps.');
if d.mode == "current"
    d.measured = rows.iq_filtered_mA;
else
    assert(d.mode == "speed",'Unrecognized trial mode.');
    d.measured = rows.omega_filtered_e_rad_s;
end
end

function [gain,phase,r2,status] = sineFit(d,t,ref,y,ending,limited)
% Compare the same last two programmed cycles. Preserve finite-window
% diagnostic fits, labelling weak or inconsistent estimates in the CSV.
gain=NaN; phase=NaN; r2=NaN;
if ending ~= "ok", status="stopped"; return; end
turns=d.hz*t;
sections = {turns>=d.cycles-2 & turns<d.cycles-1, turns>=d.cycles-1 & turns<d.cycles};
for k=1:2
    ix=sections{k}; q=turns(ix);
    if numel(q)<12 || any(~isfinite(y(ix))) || q(end)-q(1)<0.85 || any(diff(q)>0.12)
        status="incomplete_window"; return
    end
end
if any(limited==1), status="limited"; return; end
sections{3}=sections{1}|sections{2};
H=zeros(3,1);
for k=1:3
    ix=sections{k}; phi=2*pi*turns(ix);
    A=[sin(phi),cos(phi),ones(size(phi))];
    yr=y(ix); cr=A\ref(ix); cy=A\yr;
    H(k)=complex(cy(1),cy(2))/complex(cr(1),cr(2));
end
gain=abs(H(3)); phase=atan2d(imag(H(3)),real(H(3)));
ss=sum((yr-mean(yr)).^2);
if ss>0, r2=1-sum((yr-A*cy).^2)/ss; else, r2=0; end
if d.mode=="current" && hypot(cy(1),cy(2))<2
    status="below_2mA";
elseif r2<0.60
    status="weak_fit";
elseif min(abs(H))<1e-12 || abs(abs(H(1))-abs(H(2)))>0.25*gain || ...
        abs(rad2deg(angle(H(2)*conj(H(1)))))>20
    status="inconsistent_cycles";
else
    status="ok";
end
end
