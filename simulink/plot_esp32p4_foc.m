function fig = plot_esp32p4_foc(T)
% Plot the nominal trace returned by verify_esp32p4_foc.
fig = figure('Color','w','Name','DENG FOC 2208','Position',[60 60 1040 620]);
if isprop(fig,'Theme'), fig.Theme = 'light'; end
layout = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
ax(1) = nexttile(layout);
plot(T.time_s,T.ref_rpm,'k--','LineWidth',1.1);
hold on;
plot(T.time_s,T.actual_rpm,'Color',[.48 .48 .48],'LineWidth',.8);
plot(T.time_s,T.measured_rpm,'k-','LineWidth',1);
ylabel('机械转速 (rpm)');
legend('给定','电机轴真值','编码器估计','Location','southeast','Orientation','horizontal');
ax(2) = nexttile(layout);
plot(T.time_s,T.iq_ref_mA,'k--','LineWidth',1.1);
hold on;
plot(T.time_s,T.iq_f_mA,'k-','LineWidth',1);
plot(T.time_s,T.id_f_mA,'Color',[.55 .55 .55],'LineWidth',.8);
ylabel('电流 (mA)'); xlabel('时间 (s)');
legend('q 轴给定','q 轴反馈','d 轴反馈','Location','northeast','Orientation','horizontal');
loadIndex = find(diff(T.load_Nm) ~= 0,1,'first');
for a = ax
    a.Color = [1 1 1]; a.XColor = [0 0 0]; a.YColor = [0 0 0];
    a.GridColor = [.55 .55 .55]; a.GridAlpha = .25;
    grid(a,'on'); a.Box='off'; xlim(a,T.time_s([1 end]));
    if ~isempty(loadIndex)
        xline(a,T.time_s(loadIndex+1),':','施加负载', ...
            'HandleVisibility','off','LabelVerticalAlignment','bottom');
    end
end
linkaxes(ax,'x');
title(layout,'DENG FOC 2208 速度阶跃与负载扰动');
end
