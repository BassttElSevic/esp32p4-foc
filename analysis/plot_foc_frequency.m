function metrics = plot_foc_frequency(model, outDir)
% Sampled Bode plots, margins and discrete root loci. All reported frequencies are Hz.
% open is the positive return ratio L; the sign in the characteristic equation is 1+k*L.
if ~isfolder(outDir), mkdir(outDir); end
names = {'current','speed','locked'};
titles = {'Free-rotor q current','Electrical speed','Locked-rotor q current'};
for n = 1:numel(names)
    name = names{n};
    loop = model.(name);
    f = frequencyGrid(loop,model.Ts);
    m = measure(loop,f,model.Ts);
    metrics.(name) = m;
    if n <= 2
        drawBode(loop,m,f,titles{n},fullfile(outDir,['bode_' name '.png']));
    end
end
drawLoci(model,metrics,titles,fullfile(outDir,'root_locus.png'));
end

function f = frequencyGrid(loop,T)
nyquist = 1/(2*T);
roots = [pole(loop.open); pole(loop.closed); zero(loop.open); zero(loop.closed)];
% Distance from z=1 sets the very-low-frequency resolution. Exclude true integrators.
roots = roots(abs(roots-1)>1e-12 & abs(roots)>0 & isfinite(roots));
corners = abs(log(complex(roots)))/(2*pi*T);
lower = min([nyquist; corners(:)])/100;
f = logspace(log10(lower),log10(nyquist),4801);
f(end) = nyquist;
end

function response = samples(sys,f)
response = reshape(freqresp(sys,2*pi*f),1,[]);
end

function value = at(sys,f,T)
if f == 1/(2*T)
    value = evalfr(sys,-1);
else
    value = evalfr(sys,exp(2i*pi*f*T));
end
end

function r = crossingRoots(f,y,fun)
indices = find(y(1:end-1).*y(2:end)<0);
r = zeros(1,numel(indices));
for j = 1:numel(indices)
    bracket = log(f(indices(j)+(0:1)));
    r(j) = exp(fzero(@(x) fun(exp(x)),bracket));
end
r = [r, f(y==0)];
end

function r = uniqueHz(x,upper)
r = sort(x(isfinite(x) & x>0 & x<=upper*(1+1e-12)));
r = min(r,upper);
if isempty(r), return; end
r = r([true, diff(r)>1e-6*r(2:end)]);
end

function [bw,peakDb,peakHz] = closedMetrics(sys,f,T)
dc = abs(dcgain(sys));
mag = abs(samples(sys,f));
threshold = dc*10^(-3/20);
ix = find(mag<=threshold,1);
if isempty(ix)
    bw = Inf;  % no -3 dB crossing on [0,Nyquist]
elseif ix == 1
    bw = fzero(@(x) abs(at(sys,x,T))-threshold,[0 f(1)]);
else
    bw = exp(fzero(@(x) abs(at(sys,exp(x),T))-threshold,log(f(ix-1:ix))));
end
[maximum,k] = max([dc,mag]);
if k == 1
    peakHz = 0;
elseif k == numel(mag)+1
    peakHz = f(end);
else
    ix = k-1;
    limits = log(f([max(1,ix-1),ix+1]));
    [logPeak,negativeMax] = fminbnd(@(x) -abs(at(sys,exp(x),T)),limits(1),limits(2));
    peakHz = exp(logPeak);
    maximum = -negativeMax;
end
peakDb = 20*log10(maximum);
end

function m = measure(loop,f,T)
nyquist = f(end);
L = samples(loop.open,f);
margins = allmargin(loop.open);
% Seed with MATLAB's algebraic crossings (including tangencies), check against
% direct state-space frequency response, and explicitly include z=-1.
unity = crossingRoots(f,abs(L)-1,@(x) abs(at(loop.open,x,T))-1);
unity = uniqueHz([unity, reshape(margins.PMFrequency,1,[])/(2*pi), nyquist],nyquist);
unity = unity(arrayfun(@(x) abs(abs(at(loop.open,x,T))-1)<1e-7,unity));
negative = crossingRoots(f,imag(L),@(x) imag(at(loop.open,x,T)));
negative = uniqueHz([negative,reshape(margins.GMFrequency,1,[])/(2*pi),nyquist],nyquist);
candidate = arrayfun(@(x) at(loop.open,x,T),negative);
negative = negative(real(candidate)<0 & abs(imag(candidate))<1e-7*abs(candidate));
complexUnity = arrayfun(@(x) at(loop.open,x,T),unity);
m.gainCrossHz = unity;
m.phaseMarginsDeg = angle(-complexUnity)*180/pi;
m.phaseCrossHz = negative;
m.gainMarginsDb = -20*log10(abs(arrayfun(@(x) at(loop.open,x,T),negative)));
[m.phaseMarginDeg,m.phaseMarginHz] = limiting(m.phaseMarginsDeg,unity);
[m.gainMarginDb,m.gainMarginHz] = limiting(m.gainMarginsDb,negative);
up = find(m.gainMarginsDb>=0);
m.criticalGain = Inf;
m.criticalHz = NaN;
if ~isempty(up)
    [db,k] = min(m.gainMarginsDb(up));
    m.criticalGain = 10^(db/20);
    m.criticalHz = negative(up(k));
end
m.dcGain = dcgain(loop.closed);
[m.bandwidthHz,m.peakDb,m.peakHz] = closedMetrics(loop.closed,f,T);
m.measuredBandwidthHz = closedMetrics(loop.measured,f,T);
m.nyquistHz = nyquist;
m.nyquistGainDb = 20*log10(abs(at(loop.closed,nyquist,T)));
m.nominalPoleRadius = max(abs(pole(loop.closed)));
m.nominalStable = isstable(loop.closed);
end

function [value,f] = limiting(values,frequencies)
if isempty(values)
    value = Inf;
    f = NaN;
else
    [~,i] = min(abs(values));
    value = values(i);
    f = frequencies(i);
end
end

function drawBode(loop,m,f,name,path)
responses = [samples(loop.open,f);samples(loop.closed,f);samples(loop.measured,f)];
phase = unwrap(angle(responses),[],2)*180/pi;
wide = m.peakHz>0 && isfinite(m.peakHz) && m.peakDb>3;
fig = figure('Visible','off','Color','w','Position',[100 100 960+200*wide 660]);
theme(fig,'light');
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
cols = 1+wide;
layout = tiledlayout(fig,2,cols,'TileSpacing','compact','Padding','compact');
title(layout,[name ' | sampled Bode response'],'FontWeight','bold');
if isfinite(m.bandwidthHz)
    bwText = sprintf('True -3 dB BW %.4g Hz',m.bandwidthHz);
else
    bwText = sprintf('True output above -3 dB through %.4g Hz',m.nyquistHz);
end
subtitle(layout,sprintf('%s    |    PM %.3g deg    |    GM %.3g dB',bwText,m.phaseMarginDeg,m.gainMarginDb));
for j = 1:cols
    limits = [f(1),f(end)];
    if j == 2
        detail = [m.peakHz,m.gainCrossHz,m.phaseCrossHz,m.measuredBandwidthHz];
        limits(1) = max(f(1),min(detail(isfinite(detail) & detail>0))/8);
    end
    a = nexttile(layout,j); b = nexttile(layout,j+cols);
    bodeAxes(a,b,f,responses,phase,m,limits,j==1)
    if wide
        if j == 1, title(a,'Full sampled band'); else, title(a,'Crossover and resonance'); end
    end
end
exportgraphics(fig,path,'Resolution',160,'BackgroundColor','white');
end

function bodeAxes(a,b,f,responses,phase,m,limits,withLegend)
blue = [0.12 0.28 0.43]; rust = [0.66 0.31 0.17]; gray = [0.40 0.43 0.46];
use = f>=limits(1) & f<=limits(2);
semilogx(a,f(use),20*log10(abs(responses(1,use))),'Color',blue,'LineWidth',1.7); hold(a,'on')
semilogx(a,f(use),20*log10(abs(responses(2,use))),'Color',rust,'LineWidth',1.7);
semilogx(a,f(use),20*log10(abs(responses(3,use))),'--','Color',gray,'LineWidth',1.3);
if withLegend
    legend(a,'Open return ratio','Closed: true output','Closed: feedback measurement', ...
        'Location','best','FontSize',9,'AutoUpdate','off');
end
yline(a,0,':','Color',gray);
yline(a,20*log10(abs(m.dcGain))-3,'--','Color',[.72 .72 .72]);
plot(a,m.gainCrossHz,zeros(size(m.gainCrossHz)),'o','Color',blue,'MarkerFaceColor','w');
plot(a,m.phaseCrossHz,-m.gainMarginsDb,'s','Color',blue,'MarkerFaceColor','w');
if isfinite(m.bandwidthHz)
    plot(a,m.bandwidthHz,20*log10(abs(m.dcGain))-3,'o','Color',rust,'MarkerFaceColor',rust);
end
if m.peakHz>0 && m.peakDb>3 && m.peakHz>=limits(1)
    plot(a,m.peakHz,m.peakDb,'^','Color',rust,'MarkerFaceColor',rust,'MarkerSize',6);
end
ylabel(a,'Magnitude (dB)'); grid(a,'on'); xlim(a,limits);
semilogx(b,f(use),phase(1,use),'Color',blue,'LineWidth',1.7); hold(b,'on')
semilogx(b,f(use),phase(2,use),'Color',rust,'LineWidth',1.7);
semilogx(b,f(use),phase(3,use),'--','Color',gray,'LineWidth',1.3);
yline(b,-180,':','Color',gray);
if ~isempty(m.gainCrossHz)
    plot(b,m.gainCrossHz,interp1(log(f),phase(1,:),log(m.gainCrossHz)),'o','Color',blue,'MarkerFaceColor','w');
end
if ~isempty(m.phaseCrossHz)
    plot(b,m.phaseCrossHz,interp1(log(f),phase(1,:),log(m.phaseCrossHz)),'s','Color',blue,'MarkerFaceColor','w');
end
ylabel(b,'Phase (degrees)'); xlabel(b,'Frequency (Hz)'); grid(b,'on'); xlim(b,limits);
end

function drawLoci(model,metrics,titles,path)
fig = figure('Visible','off','Color','w','Position',[100 100 1060 555]);
theme(fig,'light');
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
layout = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
title(layout,'Discrete root loci | common multiplier k on both PI gains');
subtitle(layout,'Positive k, fixed PI zero; nominal controller k = 1');
names = {'current','speed'};
blue = [0.12 0.28 0.43]; rust = [0.66 0.31 0.17];
for j = 1:2
    loop = model.(names{j});
    m = metrics.(names{j});
    kmax = 100;
    if isfinite(m.criticalGain), kmax = max(kmax,4*m.criticalGain); end
    gain = unique([0,1,logspace(-5,log10(kmax),900),m.criticalGain(isfinite(m.criticalGain))]);
    r = rlocus(loop.open,gain);
    ax = nexttile(layout); hold(ax,'on');
    theta = linspace(0,2*pi,400);
    circle = plot(ax,cos(theta),sin(theta),'--','Color',[.55 .55 .55]);
    branches = plot(ax,real(r.'),imag(r.'),'Color',blue,'LineWidth',1.45);
    op = pole(loop.open); oz = zero(loop.open); cp = pole(loop.closed);
    h1 = plot(ax,real(op),imag(op),'x','Color',blue,'LineWidth',1.4,'MarkerSize',9);
    h2 = plot(ax,real(oz),imag(oz),'o','Color',blue,'LineWidth',1.2,'MarkerSize',7);
    h3 = plot(ax,real(cp),imag(cp),'o','MarkerFaceColor',rust,'MarkerEdgeColor',rust,'MarkerSize',5);
    handles = [circle,branches(1),h1,h2,h3];
    labels = {'Unit circle','Root locus','Open poles','Open zeros','Nominal poles'};
    title(ax,titles{j});
    if isfinite(m.criticalGain)
        zcrit = exp(2i*pi*m.criticalHz*model.Ts);
        h4 = plot(ax,real([zcrit conj(zcrit)]),imag([zcrit conj(zcrit)]),'d', ...
            'Color',rust,'LineWidth',1.4,'MarkerSize',9);
        handles(end+1) = h4;
        labels{end+1} = 'Stability boundary';
        subtitle(ax,sprintf('First gain increase to boundary: k = %.4g',m.criticalGain));
    end
    if j == 1, legend(ax,handles,labels,'Location','southwest','FontSize',8); end
    axis(ax,'equal'); xlim(ax,[-1.2 1.2]); ylim(ax,[-1.2 1.2]);
    xlabel(ax,'Real(z)'); ylabel(ax,'Imag(z)'); grid(ax,'on'); box(ax,'on');
end
exportgraphics(fig,path,'Resolution',160,'BackgroundColor','white');
end
