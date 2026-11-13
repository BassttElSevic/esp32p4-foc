function plot_bench_comparison(trials,outdir)
%PLOT_BENCH_COMPARISON Overlay BENCH1 data and simulations at board timestamps.
% The two columns display the recorded filtered feedback channels in their
% physical units. Only recorded time windows are shown, including fault runs.
if ~isfolder(outdir), mkdir(outdir); end
groups = string({trials.group});
modes = string({trials.mode});
font = comparisonFont();
ink = [0.13 0.17 0.20];
gray = [0.40 0.42 0.43];
simColor = [0.09 0.30 0.51];
benchColor = [0.66 0.26 0.13];

fig = figure('Visible','off','Color','white','Units','pixels', ...
    'Position',[80 40 1350 1530],'DefaultAxesFontName',font, ...
    'DefaultTextFontName',font,'DefaultAxesFontSize',10);
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
if isprop(fig,'Theme'), fig.Theme = 'light'; end
layout = tiledlayout(fig,6,2,'TileSpacing','compact','Padding','compact');
title(layout,'DENG FOC 2208  |  正弦给定下的仿真与实测', ...
    'FontName',font,'FontSize',19,'FontWeight','bold','Color',ink);

% Keep repeat runs opposite each other and order the initial amplitudes.
initialCurrent = find(groups=="initial" & modes=="current");
[~,order] = sort([trials(initialCurrent).amp]);
slots = zeros(size(trials));
slots(initialCurrent(order)) = [1 3];
slots(groups=="initial" & modes=="speed") = 2;
for k = 1:4
    group = compose("%02d",k);
    slots(groups==group & modes=="current") = 2*k+3;
    slots(groups==group & modes=="speed") = 2*k+4;
end

for k = 1:numel(trials)
    d = trials(k);
    ax = nexttile(layout,slots(k));
    hold(ax,'on');
    isCurrent = modes(k)=="current";
    isFault = string(d.endReason)~="ok";
    t = d.t(:);
    xEnd = t(end);
    if ~isFault, xEnd = max(xEnd,d.cycles/d.hz); end
    inMs = xEnd < 0.10;
    scale = 1;
    if inMs, scale = 1000; end
    x = scale*t;
    reference = plot(ax,x,d.ref,':','Color',gray,'LineWidth',1.4);
    plot(ax,x,d.predicted,'--','Color',simColor,'LineWidth',1.5);
    measurement = plot(ax,x,d.measured,'-','Color',benchColor,'LineWidth',1.0);
    if numel(x)<=20
        measurement.Marker = 'o';
        measurement.MarkerSize = 3.5;
        measurement.MarkerFaceColor = benchColor;
    end
    % Mark the input extrema even where simulation and input coincide.
    if ~isCurrent && numel(t)>20
        peaks=(0.25:0.5:t(end)*d.hz)/d.hz;
        if ~isempty(peaks)
            reference.Marker='o';
            reference.MarkerIndices=unique(interp1(t,1:numel(t),peaks,'nearest'));
            reference.MarkerSize=3.6;
            reference.MarkerFaceColor='white';
            uistack(reference,'top');
        end
    end
    showSimStop = isfinite(d.simEndTime) && d.simEndTime>=0 && d.simEndTime<=xEnd;
    if showSimStop
        xline(ax,scale*d.simEndTime,'-.','Color',simColor,'LineWidth',0.9);
    end
    ax.FontName = font;
    ax.FontSize = 10;
    ax.XColor = ink;
    ax.YColor = ink;
    ax.Box = 'off';
    ax.TickDir = 'out';
    ax.LineWidth = 0.6;
    ax.GridColor = [0.64 0.67 0.70];
    ax.GridAlpha = 0.22;
    grid(ax,'on');
    xlim(ax,[0 scale*xEnd]);
    channels = [d.ref(:);d.predicted(:);d.measured(:)];
    peak = max(abs(channels(isfinite(channels))));
    if peak>0, ylim(ax,1.12*peak*[-1 1]); end
    if isCurrent
        name = "电流内环";
        unit = "mA";
        ylabel(ax,'q 轴电流 (mA)');
    else
        name = "速度外环";
        unit = "电 rad/s";
        ylabel(ax,'电角速度 (rad/s)');
    end
    if inMs
        xlabel(ax,'时间 (ms)');
    else
        xlabel(ax,'时间 (s)');
    end
    if groups(k)=="initial"
        groupLabel = "首批";
    else
        groupLabel = groups(k)+" 组";
    end
    caption = sprintf('%s · %s    %g Hz，±%g %s',groupLabel,name,d.hz,d.amp,unit);
    if isFault
        caption = sprintf('%s  |  实测 %s，%d 点',caption,string(d.endReason),numel(x));
    end
    if showSimStop
        caption = sprintf('%s  |  仿真保护 %g ms',caption,1000*d.simEndTime);
    elseif isfinite(d.simEndTime) && d.simEndTime>xEnd
        caption = sprintf('%s  |  仿真 %g ms 停（窗外）',caption,1000*d.simEndTime);
    end
    title(ax,caption,'FontName',font,'FontWeight','normal', ...
        'FontSize',11.5,'Color',ink,'Interpreter','none');
end

key = nexttile(layout,4);
hold(key,'on');
axis(key,[0 1 0 1]);
axis(key,'off');
labels = {'输入正弦','仿真反馈','实测反馈'};
styles = {':','--','-'};
colors = {gray,simColor,benchColor};
for j = 1:numel(labels)
    x0 = 0.02 + 0.31*(j-1);
    plot(key,[x0 x0+0.070],[0.75 0.75],styles{j},'Color',colors{j},'LineWidth',1.7);
    text(key,x0+0.083,0.75,labels{j},'FontSize',11,'Color',ink);
end
text(key,0.02,0.46,'两列分别比较滤波后的 q 轴电流和电角速度。','Color',ink,'FontSize',10.5);
text(key,0.02,0.26,'实机触发保护的记录截于末个已保存的采样点。','Color',ink,'FontSize',10.5);
if any(isfinite([trials.simEndTime]))
    text(key,0.02,0.06,'蓝色竖点划线标出记录窗口内的仿真保护时刻。','Color',ink,'FontSize',10.5);
end

exportgraphics(fig,fullfile(outdir,'bench_sine_comparison.png'), ...
    'Resolution',145,'BackgroundColor','white');
exportgraphics(fig,fullfile(outdir,'bench_sine_comparison.pdf'), ...
    'ContentType','vector','BackgroundColor','white');
end

function name = comparisonFont()
available = listfonts();
for candidate = ["Microsoft YaHei","Noto Sans CJK SC","Noto Sans SC","PingFang SC","SimHei"]
    if any(strcmpi(candidate,available))
        name = char(candidate);
        return;
    end
end
name = get(groot,'DefaultAxesFontName');
end
