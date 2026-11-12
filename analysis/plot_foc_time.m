function checks = plot_foc_time(model,metrics,outDir,options)
%PLOT_FOC_TIME Simulate small reference steps and steady-state sine tracking.
% Inputs are q current in A (free shaft, speed loop open) and electrical
% speed in rad/s (both loops closed). The default signals probe the smooth
% linear model. Optional name-value arguments override the excitations.
arguments
    model (1,1) struct
    metrics (1,1) struct
    outDir (1,1) string
    options.CurrentAmplitudeA (1,1) double {mustBePositive} = 1e-7
    options.SpeedAmplitudeRadps (1,1) double {mustBePositive} = 0.01
    options.CurrentFrequenciesHz (1,2) double = [NaN NaN]
    options.SpeedFrequenciesHz (1,2) double = [NaN NaN]
end
if ~isfolder(outDir), mkdir(outDir); end
ts = model.Ts;
names = {'current','speed'};
titles = {'Free-shaft q current','Electrical speed'};
units = {'q current (\muA)','electrical speed (rad/s)'};
amplitudes = [options.CurrentAmplitudeA, options.SpeedAmplitudeRadps];
displayScale = [1e6,1];
requested = [options.CurrentFrequenciesHz; options.SpeedFrequenciesHz];
stepFig = figure('Color','w','Visible','off','Position',[60 70 1100 720]);
sineFig = figure('Color','w','Visible','off','Position',[60 70 1100 720]);
cleanFigs = onCleanup(@() close([stepFig sineFig])); %#ok<NASGU>
if isprop(stepFig,'Theme')
    stepFig.Theme = 'light';
    sineFig.Theme = 'light';
end
stepLayout = tiledlayout(stepFig,2,2,'TileSpacing','compact','Padding','compact');
sineLayout = tiledlayout(sineFig,2,2,'TileSpacing','compact','Padding','compact');
title(stepLayout,'Closed-loop step response','FontWeight','normal');
title(sineLayout,'Closed-loop steady-state sine response','FontWeight','normal');

for row = 1:2
    key = names{row};
    loop = model.(key);
    amp = amplitudes(row);
    rho = max(abs([pole(loop.closed); pole(loop.measured); 0]));
    if ~isfinite(rho) || rho >= 1
        error('plot_foc_time:NoSteadyState',...
            '%s loop needs stable poles for settling and steady-state sine analysis.',key);
    end
    % End after the dominant stable mode falls to 0.01% of its initial value.
    nEnd = max(80,decaySamples(rho,1e-4));
    % Simulate densely at the start and use exact integer sample advances
    % through the slow tail. The nominal current loop takes millions of ticks.
    if nEnd <= 10000
        ticks = (0:nEnd)';
    else
        stride = ceil(nEnd/2200);
        ticks = unique([0:600, round(logspace(log10(600),log10(nEnd),800)), ...
                        0:stride:nEnd, nEnd])';
    end
    F = drivenState(loop.closed,1,1);
    current = stepSamples(loop.closed,F,amp,ticks);
    measured = stepSamples(loop.measured,drivenState(loop.measured,1,1),amp,ticks);
    finalValue = amp*dcgain(loop.closed);
    checks.(key).step = stepSummary(current,ticks,ts,amp,finalValue,loop.closed,F);
    shortN = min(nEnd,40);
    for col = 1:2
        ax = nexttile(stepLayout);
        if col == 1
            use = ticks <= shortN;
            t = ticks(use)*ts*1000;
            xlab = 'time (ms)';
            label = 'first samples';
        else
            settling = checks.(key).step.settlingTime2PctS;
            fullTime = min(ticks(end)*ts, max(2*settling,2*shortN*ts));
            if ~isfinite(fullTime), fullTime = ticks(end)*ts; end
            use = ticks*ts <= fullTime;
            t = ticks(use)*ts;
            xlab = 'time (s)';
            label = 'full response';
        end
        % The command is applied at t=0; include its zero pre-step value.
        plotSignals(ax,t,[amp*ones(size(t)), current(use), measured(use)]*displayScale(row));
        h = ax.Children(end);
        h.XData = [0; t]; h.YData = [0; amp*displayScale(row)*ones(size(t))];
        title(ax,sprintf('%s | %s',titles{row},label));
        xlabel(ax,xlab); ylabel(ax,units{row}); xlim(ax,[0 t(end)]);
    end

    f = requested(row,:);
    if all(isnan(f))
        bw = metrics.(key).bandwidthHz;
        if isfinite(bw) && bw > 0
            f = [0.2*bw, bw];
        else
            % For a loop without a -3 dB crossing, test low-frequency tracking
            % and the measured peak. Stay inside the sampled frequency range.
            peak = metrics.(key).peakHz;
            if ~isfinite(peak) || peak <= 0, peak = 0.25/ts; end
            peak = min(peak,0.49/ts);
            f = [peak/100, peak];
        end
    end
    if any(~isfinite(f) | f <= 0 | f >= 0.5/ts)
        error('plot_foc_time:Frequency','Use two positive frequencies below Nyquist for %s.',key);
    end
    for col = 1:2
        [trace,test] = sineTrial(loop,ts,f(col),amp,rho);
        checks.(key).sine(col) = test;
        ax = nexttile(sineLayout);
        plotSignals(ax,trace.cycles,trace.signals*displayScale(row));
        title(ax,sprintf('%s | %.5g Hz',titles{row},f(col)));
        subtitle(ax,sprintf('actual / reference = %.4g    phase = %.2f%s',...
            test.closed.simulatedGain,test.closed.simulatedPhaseDeg,char(176)));
        xlabel(ax,'time (input cycles)'); ylabel(ax,units{row});
        xlim(ax,[0 trace.cycles(end)]);
    end
end
for fig = [stepFig sineFig]
    axesList = findall(fig,'Type','axes');
    lg = legend(axesList(end),'Location','southoutside','Orientation','horizontal','Box','off');
    lg.Layout.Tile = 'south';
end
exportgraphics(stepFig,fullfile(outDir,'step_response.png'),'Resolution',160,'BackgroundColor','white');
exportgraphics(sineFig,fullfile(outDir,'sine_response.png'),'Resolution',160,'BackgroundColor','white');
end

function n = decaySamples(rho,target)
if rho == 0, n = 1; else, n = ceil(log(target)/log(rho)); end
end

function F = drivenState(sys,drive,readout)
% Lift the time-domain recurrence by appending autonomous input states.
[A,B] = ssdata(sys);
F = [A, B*readout; zeros(size(drive,1),size(A,2)), drive];
end

function states = simulateTicks(F,initial,ticks)
% F^m advances exactly m original control ticks, including the input state.
states = zeros(numel(initial),numel(ticks));
states(:,1) = initial;
[strides,~,indices] = unique(diff(ticks));
advances = cell(size(strides));
for j = 1:numel(strides)
    advances{j} = F^strides(j);
end
for k = 1:numel(ticks)-1
    states(:,k+1) = advances{indices(k)}*states(:,k);
end
end

function y = stepSamples(sys,F,amp,ticks)
[~,~,C,D] = ssdata(sys);
initial = [zeros(size(C,2),1); amp];
y = ([C D]*simulateTicks(F,initial,ticks)).';
end

function stats = stepSummary(y,ticks,ts,amp,finalValue,sys,F)
info = stepinfo(y,ticks*ts,finalValue,'SettlingTimeThreshold',0.02);
stats = struct('referenceAmplitude',amp,'steadyValue',finalValue,...
    'peakAbs',info.Peak,'peakTimeS',info.PeakTime,'overshootPct',info.Overshoot,...
    'riseTimeS',info.RiseTime,'settlingTime2PctS',info.SettlingTime,'endTimeS',ticks(end)*ts);
% Refine the last crossing of the 2% band to an original controller tick.
errBand = 0.02*abs(finalValue);
last = find(abs(y-finalValue)>errBand,1,'last');
if isempty(last) || last==numel(ticks), return; end
lo = ticks(last); hi = ticks(last+1);
[~,~,C,D] = ssdata(sys);
initial = [zeros(size(C,2),1); amp];
while hi-lo > 1
    mid = floor((hi+lo)/2);
    if abs([C D]*(F^mid)*initial-finalValue) > errBand
        lo = mid;
    else
        hi = mid;
    end
end
stats.settlingTime2PctS = hi*ts;
end

function [trace,stats] = sineTrial(loop,ts,f,amp,rho)
angle = 2*pi*f*ts;
osc = [cos(angle),sin(angle); -sin(angle),cos(angle)];
% Start the simulated system at rest and run the driven difference equations
% through a warm-up. Choose initial source phase to cross zero at display start.
nWarm = decaySamples(rho,1e-13);
phi = -mod(nWarm*angle,2*pi);
nPeriod = 1/(f*ts);
stride = max(1,floor(nPeriod/140));
nEnd = ceil(3*nPeriod);
ticks = unique([0:stride:nEnd,nEnd])';
phase = ticks*angle;
u = amp*sin(phase);
signals = zeros(numel(ticks),3);
signals(:,1) = u;
stats = struct('frequencyHz',f,'referenceAmplitude',amp,'warmupTimeS',nWarm*ts);
channels = {'closed','measured'};
for j = 1:2
    sys = loop.(channels{j});
    [~,~,C,D] = ssdata(sys);
    F = drivenState(sys,osc,[amp 0]);
    start = (F^nWarm)*[zeros(size(C,2),1);sin(phi);cos(phi)];
    start(end-1:end) = [0;1];
    states = simulateTicks(F,start,ticks);
    y = ([C D*amp 0]*states).';
    signals(:,j+1) = y;
    fit = [sin(phase), cos(phase), ones(size(phase))]\y;
    gain = hypot(fit(1),fit(2))/amp;
    phaseDeg = atan2d(fit(2),fit(1));
    H = freqresp(sys,2*pi*f);
    predictedPhase = atan2d(imag(H),real(H));
    delta = phaseDeg-predictedPhase;
    stats.(channels{j}) = struct('simulatedGain',gain,'simulatedPhaseDeg',phaseDeg,...
        'bodeGain',abs(H),'bodePhaseDeg',predictedPhase,...
        'gainError',abs(gain-abs(H)),'phaseErrorDeg',abs(atan2d(sind(delta),cosd(delta))));
end
trace = struct('cycles',ticks*ts*f,'signals',signals);
end

function plotSignals(ax,x,Y)
hold(ax,'on');
plot(ax,x,Y(:,1),':','Color',[0.26 0.26 0.26],'LineWidth',1.5,'DisplayName','reference');
plot(ax,x,Y(:,2),'-','Color',[0.08 0.25 0.41],'LineWidth',1.4,'DisplayName','actual');
plot(ax,x,Y(:,3),'--','Color',[0.58 0.31 0.16],'LineWidth',1.15,'DisplayName','feedback estimate');
ax.FontName = 'Arial'; ax.FontSize = 10; ax.Box = 'on'; ax.LineWidth = 0.65;
ax.GridAlpha = 0.12; grid(ax,'on');
y = [min(Y(:)),max(Y(:))];
if diff(y)>0, ylim(ax,y+[-1 1]*0.07*diff(y)); end
end
