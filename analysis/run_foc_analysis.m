function [metrics, checks, model] = run_foc_analysis(outDir, dataFile)
%RUN_FOC_ANALYSIS Reproduce the sampled 2208 frequency and time response study.
% From the repository root: addpath('analysis'); [m,c,sys] = run_foc_analysis;
% Optional inputs select an output directory and a Simulink model data script.
if nargin < 1 || isempty(outDir)
    outDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if nargin < 2 || isempty(dataFile)
    model = foc_loop_models();
else
    model = foc_loop_models(dataFile);
end
if ~isfolder(outDir)
    mkdir(outDir);
end
metrics = plot_foc_frequency(model, outDir);
checks = plot_foc_time(model, metrics, outDir);

labels = ["自由转子 q 轴电流"; "电角速度"; "堵转 q 轴电流"];
data = [metrics.current; metrics.speed; metrics.locked];
loopTable = table(labels, [data.bandwidthHz]', [data.measuredBandwidthHz]', ...
    [data.phaseMarginDeg]', [data.phaseMarginHz]', [data.gainMarginDb]', ...
    [data.gainMarginHz]', [data.criticalGain]', 'VariableNames', ...
    {'Loop','Bandwidth_Hz','FeedbackBandwidth_Hz','PM_deg','GainCrossover_Hz', ...
    'GM_dB','PhaseCrossover_Hz','CriticalGainMultiplier'});
disp(loopTable)
writetable(loopTable, fullfile(outDir, 'loop_metrics.csv'), 'Encoding','UTF-8');
writeReport(model, metrics, checks, outDir);
end

function writeReport(model, m, checks, outDir)
% Assemble the verified MATLAB plots with a single-page summary for sharing.
pdf = fullfile(outDir, 'foc_loop_analysis.pdf');
fig = figure('Visible','off','Color','w','Units','pixels',...
    'Position',[60 60 1170 827],'MenuBar','none','ToolBar','none');
guard = onCleanup(@() close(fig));
theme(fig,'light');
font = 'Microsoft YaHei';
if ~any(strcmpi(listfonts, font))
    font = get(groot,'DefaultAxesFontName');
end
ax = pageAxes(fig);
ink = [0.09 0.13 0.17]; navy = [0.16 0.27 0.39];
note(ax,.08,.88,'DENG FOC 2208  /  ESP32-P4',13,navy,font,'bold')
note(ax,.08,.80,'电流内环与速度外环的采样响应',29,ink,font,'bold')
note(ax,.08,.72,sprintf('静止空载工作点     控制周期  %.4g ms     奈奎斯特频率  %.5g Hz', ...
    1000*model.Ts, 0.5/model.Ts),14,ink,font)

columns = [.08 .335 .47 .60 .745 .86];
headings = {'环路 / 真实输出','带宽 / Hz','相位裕度','交越 / Hz¹','增益裕度','交越 / Hz²'};
for j = 1:numel(columns)
    note(ax,columns(j),.615,headings{j},12,navy,font,'bold')
end
line(ax,[.08 .95],[.592 .592],'Color',navy,'LineWidth',1.1);
names = {'自由转子 q 电流','电角速度','堵转 q 电流'};
data = {m.current, m.speed, m.locked};
for j = 1:3
    a = data{j};
    if isfinite(a.bandwidthHz)
        bwText = sprintf('%.5g',a.bandwidthHz);
    else
        bwText = '无 −3 dB 交点';
    end
    cells = {names{j},bwText,sprintf('%.2f°',a.phaseMarginDeg),...
        sprintf('%.5g',a.phaseMarginHz),sprintf('%.2f dB',a.gainMarginDb),...
        sprintf('%.5g',a.gainMarginHz)};
    y = .55-(j-1)*.065;
    for k = 1:numel(columns)
        note(ax,columns(k),y,cells{k},13,ink,font)
    end
end
line(ax,[.08 .95],[.382 .382],'Color',navy,'LineWidth',.7);
note(ax,.08,.34,'¹ 开环 0 dB 交越。   ² 开环负实轴交越，计入奈奎斯特端点。',12,ink,font)
note(ax,.08,.285,'带宽取给定到真实输出相对直流增益首次下降 3 dB 的频率。速度反馈来自编码器差分和低通。',12,ink,font)
note(ax,.08,.23,sprintf('真实速度峰值 %.3g dB（%.4g Hz）。编码器估计速度闭环的 −3 dB 带宽为 %s Hz。', ...
    m.speed.peakDb, m.speed.peakHz, num2str(m.speed.measuredBandwidthHz,'%.5g')),12,ink,font)
note(ax,.08,.175,'电流环断开速度反馈；速度环保留闭合的电流环。堵转数据用于观察反电动势对自由轴的影响。',12,ink,font)
note(ax,.08,.115,'模型采用平均桥、平滑量化和未限幅 PI。惯量与摩擦取模型参数，实机辨识后可重新运行。',12,ink,font)
exportPage(fig, pdf, false)

files = {'bode_current.png','bode_speed.png','root_locus.png','step_response.png','sine_response.png'};
titles = {'电流内环的开环和闭环伯德图','速度外环的开环和闭环伯德图','离散域根轨迹',...
    '阶跃给定：电流内环与速度外环','正弦给定：电流内环与速度外环'};
sinSpeed = checks.speed.sine(end);
captions = {sprintf('自由轴在低频会随转矩加速。相同 PI 在堵转时的电流跟踪带宽为 %.5g Hz。',m.locked.bandwidthHz),...
    sprintf('真实电角速度在 %.4g Hz 达到峰值，幅值比 %.3g；奈奎斯特频率处仍有 %+.2f dB。', ...
        m.speed.peakHz,10^(m.speed.peakDb/20),m.speed.nyquistGainDb),...
    sprintf('保持 PI 零点，同步缩放该环的比例与积分增益。自由轴电流与速度环首次触及单位圆的倍数分别为 %.4g 和 %.4g。', ...
        m.current.criticalGain,m.speed.criticalGain),...
    sprintf('真实电流进入并保持在 2%% 误差带需 %.5g s。真实电角速度超调 %.2f%%，2%% 调节时间 %.4g ms。', ...
        checks.current.step.settlingTime2PctS,checks.speed.step.overshootPct,1e3*checks.speed.step.settlingTime2PctS),...
    sprintf('实际值与滤波值在低频电流试验中几乎重合。%.4g Hz 速度试验中，真实输出与反馈的幅值比分别为 %.4g 和 %.4g。', ...
        sinSpeed.frequencyHz,sinSpeed.closed.simulatedGain,sinSpeed.measured.simulatedGain)};
for k = 1:numel(files)
    clf(fig)
    ax = pageAxes(fig);
    note(ax,.045,.955,titles{k},20,ink,font,'bold')
    note(ax,.90,.955,sprintf('%d / %d',k+1,numel(files)+1),11,navy,font)
    note(ax,.045,.045,captions{k},11,ink,font)
    plotAx = axes(fig,'Units','normalized','Position',[.026 .087 .948 .811]);
    pixels = imread(fullfile(outDir,files{k}));
    % Preserve edge labels when placing a tightly cropped PNG on a PDF page.
    pad = 18;
    padded = repmat(uint8(255),size(pixels,1)+2*pad,size(pixels,2)+2*pad,3);
    padded(pad+(1:size(pixels,1)),pad+(1:size(pixels,2)),:) = pixels;
    image(plotAx,padded);
    axis(plotAx,'image')
    axis(plotAx,'off')
    exportPage(fig, pdf, true)
end
end

function ax = pageAxes(fig)
ax = axes(fig,'Units','normalized','Position',[0 0 1 1],...
    'XLim',[0 1],'YLim',[0 1],'Visible','off');
hold(ax,'on')
end

function note(ax,x,y,str,sz,color,font,weight)
if nargin < 8, weight = 'normal'; end
text(ax,x,y,str,'Interpreter','none','FontSize',sz,'FontName',font,...
    'FontWeight',weight,'Color',color,'VerticalAlignment','middle');
end

function exportPage(fig, pdf, append)
exportgraphics(fig,pdf,'Append',append,'ContentType','vector','BackgroundColor','white',...
    'Units','inches','Width',11.7,'Height',8.27,'Padding','figure','Resolution',200);
end
