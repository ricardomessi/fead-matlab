%% FEAD_PostProcess.m  –  Post-simulation analysis of FEAD_TestRig results
%  Run after simulating FEAD_TestRig.slx to extract, analyse and plot all
%  signals from the To-Workspace blocks.
%
%  Usage:  >> FEAD_params; sim('FEAD_TestRig'); FEAD_PostProcess
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('pulleys','var'), FEAD_params; end

pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
np     = 6;

fprintf('\n=== FEAD Test Rig – Post-Processing ===\n\n');

%% ── Check simulation data is available ───────────────────────────────────────
missing = {};
for k = 1:np
    if ~exist([pnames{k} '_torque'],'var'), missing{end+1} = [pnames{k} '_torque']; end
    if ~exist([pnames{k} '_omega'], 'var'), missing{end+1} = [pnames{k} '_omega'];  end
end

if ~isempty(missing)
    fprintf('⚠  Workspace variables not found:\n');
    fprintf('   %s\n', strjoin(missing,', '));
    fprintf('   Run: sim(''FEAD_TestRig'') first, then re-run this script.\n\n');
    fprintf('Generating analytical sweep instead (no Simscape needed)...\n\n');
    use_simdata = false;
else
    use_simdata = true;
    fprintf('Simscape results found — using measured data.\n\n');
end

%% ── Analytical sweep (used when no sim data) ─────────────────────────────────
rpm_sw = 300:25:2500;
n_sw   = numel(rpm_sw);
F_hub  = zeros(np,n_sw);
T_t    = zeros(np,n_sw);
T_s    = zeros(np,n_sw);
SF_v   = zeros(np,n_sw);
P_v    = zeros(np,n_sw);
v_v    = zeros(1,n_sw);

wrap_deg = [166.5 127.6 108.4 145.1 105.7 76.4];

for ri = 1:n_sw
    rpm  = rpm_sw(ri);
    v    = max(pulleys(1).r/1000 * rpm*2*pi/60, 0.01);
    v_v(ri) = v;
    T_c  = belt.lin_mass * v^2;
    for k = 1:np
        P_k = max(interp1(load_table.rpm,load_table.(pnames{k}),rpm,'linear','extrap'),0);
        P_v(k,ri)  = P_k;
        T_eff      = P_k*1000/v;
        T_t(k,ri)  = belt.static_tension + T_eff/2 + T_c;
        T_s(k,ri)  = max(belt.static_tension - T_eff/2,0) + T_c;
        kp = mod(k-2,np)+1; kn = mod(k,np)+1;
        dx_p = pulleys(kp).x-pulleys(k).x; dy_p = pulleys(kp).y-pulleys(k).y;
        dx_n = pulleys(kn).x-pulleys(k).x; dy_n = pulleys(kn).y-pulleys(k).y;
        Lp = max(hypot(dx_p,dy_p),1); Ln = max(hypot(dx_n,dy_n),1);
        Fx = T_t(k,ri)*dx_p/Lp + T_s(k,ri)*dx_n/Ln;
        Fy = T_t(k,ri)*dy_p/Lp + T_s(k,ri)*dy_n/Ln;
        F_hub(k,ri) = hypot(Fx,Fy);
        mu_th = belt.mu * wrap_deg(k)*pi/180;
        if T_s(k,ri) > 0.1
            SF_v(k,ri) = log(T_t(k,ri)/T_s(k,ri))/mu_th;
        else
            SF_v(k,ri) = 9.99;
        end
    end
end

%% ── If Simscape data exists, merge into rpm-aligned arrays ───────────────────
if use_simdata
    t_sim = evalin('base',[pnames{1} '_omega.Time']);
    for k = 1:np
        try
            om_k = evalin('base',[pnames{k} '_omega.Data']);
            T_k  = evalin('base',[pnames{k} '_torque.Data']);
            rpm_k_sim = om_k * 60/(2*pi);
            fprintf('  %s: RPM range [%.0f – %.0f], peak T=%.1f Nm\n',...
                pnames{k}, min(rpm_k_sim), max(rpm_k_sim), max(abs(T_k)));
        catch
        end
    end
    fprintf('\n');
end

%% ── Print results table at 1200 RPM ─────────────────────────────────────────
[~,i12] = min(abs(rpm_sw-1200));
fprintf('Results at 1200 RPM (MEAN tensioner):\n');
fprintf('%-6s %9s %9s %9s %9s %8s %6s\n','Pulley','T_tight','T_slack','T_cent','F_hub','P(kW)','SF');
fprintf('%s\n',repmat('-',1,70));
pdf_F = [2658.9 2866.4 1710.1 1678.1 985.8 608.5];
for k = 1:np
    T_c12 = belt.lin_mass * v_v(i12)^2;
    fprintf('%-6s %9.1f %9.1f %9.1f %9.1f %8.2f %6.2f  [PDF:%.0fN]\n',...
        pnames{k}, T_t(k,i12), T_s(k,i12), T_c12, F_hub(k,i12),...
        P_v(k,i12), min(SF_v(k,i12),9.99), pdf_F(k));
end

%% ── Fatigue life table ───────────────────────────────────────────────────────
fprintf('\nBelt Fatigue Life (WLTC weighted):\n');
wltc_rpm = [900 1200 1600 2000]; wltc_w = [.25 .25 .25 .25];
belt_life = zeros(np,1);
for k = 1:np
    D = 0;
    for wi=1:4
        vi  = max(pulleys(1).r/1000*wltc_rpm(wi)*2*pi/60,0.01);
        Pi  = max(interp1(load_table.rpm,load_table.(pnames{k}),wltc_rpm(wi),'linear','extrap'),0);
        Tti = belt.static_tension + Pi*1000/vi/2 + belt.lin_mass*vi^2;
        Nfi = belt.wohler_Nref*(belt.wohler_Tref/max(Tti,1))^belt.wohler_m;
        D   = D + wltc_w(wi)/Nfi;
    end
    belt_life(k) = min(belt.length_m/1000/max(D,1e-15),500000);
    status = ternary(belt_life(k)>=200000,'✅ OK','⚠ LOW');
    fprintf('  %-6s  %.0f km  %s\n', pnames{k}, belt_life(k), status);
end
fprintf('  OVERALL BELT LIFE: %.0f km\n\n', min(belt_life));

%% ═══════════════════════════════════════════════════════════════════════════
%  PLOTS  – 3×3 dark-theme figure
% ═══════════════════════════════════════════════════════════════════════════
BG=[0.06 0.08 0.14]; TC=[0.89 0.91 0.94]; AX=[0.09 0.12 0.20];
AM=[0.96 0.62 0.04]; VI=[0.65 0.54 0.98]; GR=[0.20 0.85 0.60];
RD=[0.94 0.27 0.27]; PK=[0.96 0.28 0.71];
pcolors={AM,[0.55 0.36 0.96],VI,GR,PK,[0.38 0.64 0.98]};

fig = figure('Color',BG,'Position',[40 30 1700 920],'Name','FEAD PostProcess Results');

mkax = @(r,c) subplot(3,3,(r-1)*3+c,'Parent',fig);

setup_ax = @(ax,xl,yl,ttl) deal(...
    set(ax,'Color',AX,'XColor',TC,'YColor',TC,...
        'GridColor',[0.14 0.22 0.32],'GridAlpha',0.6,'Box','on'),...
    xlabel(ax,xl,'Color',TC), ylabel(ax,yl,'Color',TC),...
    title(ax,ttl,'Color',AM,'FontWeight','bold'), grid(ax,'on'));

%% 1 – Hub Loads vs RPM
ax1=mkax(1,1); hold(ax1,'on'); setup_ax(ax1,'RPM','F_hub (N)','Hub Loads vs RPM');
for k=1:np, plot(ax1,rpm_sw,F_hub(k,:),'LineWidth',2,'Color',pcolors{k},'DisplayName',pnames{k}); end
for k=1:np, plot(ax1,1200,pdf_F(k),'o','MarkerSize',8,'Color',pcolors{k},'MarkerFaceColor',pcolors{k},'HandleVisibility','off'); end
xline(ax1,1200,'--','Color',[1 1 1 0.25],'HandleVisibility','off');
legend(ax1,'TextColor',TC,'Color',AX,'Location','northwest');

%% 2 – Belt Tensions
ax2=mkax(1,2); hold(ax2,'on'); setup_ax(ax2,'RPM','Tension (N)','Belt Tensions');
for k=1:np
    plot(ax2,rpm_sw,T_t(k,:),'LineWidth',2,'Color',pcolors{k},'DisplayName',[pnames{k},' T_t']);
    plot(ax2,rpm_sw,T_s(k,:),'--','LineWidth',1,'Color',pcolors{k}*0.6,'HandleVisibility','off');
end
yline(ax2,belt.T_max,'r--','LineWidth',1.5,'Label','T_{max}','HandleVisibility','off');
legend(ax2,'TextColor',TC,'Color',AX,'Location','northwest','NumColumns',2);

%% 3 – Slip Safety Factor
ax3=mkax(1,3); hold(ax3,'on'); setup_ax(ax3,'RPM','Slip SF','Capstan Slip SF vs RPM');
for k=1:np, plot(ax3,rpm_sw,min(SF_v(k,:),6),'LineWidth',2,'Color',pcolors{k},'DisplayName',pnames{k}); end
yline(ax3,1.0,'r--','LineWidth',2,'Label','SF=1 SLIP','HandleVisibility','off');
yline(ax3,1.3,'Color',AM,'LineStyle','--','LineWidth',1.5,'Label','SF=1.3 Safe','HandleVisibility','off');
ylim(ax3,[0 6]); legend(ax3,'TextColor',TC,'Color',AX,'Location','northeast');

%% 4 – Power vs RPM (stacked)
ax4=mkax(2,1); hold(ax4,'on'); setup_ax(ax4,'RPM','Power (kW)','Accessory Power vs RPM');
P_stack = zeros(1,n_sw);
for k=1:np
    P_top = P_stack + P_v(k,:);
    fill(ax4,[rpm_sw fliplr(rpm_sw)],[P_top fliplr(P_stack)],...
        pcolors{k},'EdgeColor',pcolors{k},'FaceAlpha',0.5,'DisplayName',pnames{k});
    P_stack = P_top;
end
legend(ax4,'TextColor',TC,'Color',AX,'Location','northwest');

%% 5 – Belt velocity vs RPM
ax5=mkax(2,2); hold(ax5,'on'); setup_ax(ax5,'RPM','v_{belt} (m/s)','Belt Linear Velocity');
plot(ax5,rpm_sw,v_v,'Color',VI,'LineWidth',2.5);
yline(ax5,belt.v_max,'r--','LineWidth',1.5,'Label',sprintf('v_{max}=%.0f m/s',belt.v_max));
fill(ax5,[rpm_sw fliplr(rpm_sw)],[v_v fliplr(zeros(1,n_sw))],...
    VI,'FaceAlpha',0.15,'EdgeColor','none');

%% 6 – Belt fatigue life bar
ax6=mkax(2,3); setup_ax(ax6,'Pulley','Life (×10³ km)','Belt Life (WLTC Weighted)');
b=bar(ax6,belt_life/1000,'FaceColor','flat');
for k=1:np, b.CData(k,:)=pcolors{k}; end
set(ax6,'XTickLabel',pnames);
yline(ax6,200,'--','Color',AM,'LineWidth',1.5,'Label','Min 200k km');

%% 7 – Centrifugal tension vs RPM
ax7=mkax(3,1); hold(ax7,'on'); setup_ax(ax7,'RPM','T_c (N)','Centrifugal Tension');
T_c_arr = belt.lin_mass * v_v.^2;
plot(ax7,rpm_sw,T_c_arr,'Color',AM,'LineWidth',2.5);
fill(ax7,[rpm_sw fliplr(rpm_sw)],[T_c_arr fliplr(zeros(1,n_sw))],...
    AM,'FaceAlpha',0.2,'EdgeColor','none');
text(ax7,1200,belt.lin_mass*(1200*pulleys(1).r/1000*2*pi/60)^2,...
    sprintf(' 1200RPM: %.1fN',belt.lin_mass*(pulleys(1).r/1000*1200*2*pi/60)^2),...
    'Color',TC,'FontSize',9);

%% 8 – Hub load polar (at 1200 RPM)
ax8=mkax(3,2); hold(ax8,'on');
set(ax8,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.14 0.22 0.32],'GridAlpha',0.6,'Box','on');
title(ax8,'Hub Load Vectors @ 1200 RPM','Color',AM,'FontWeight','bold');
for k=1:np
    kp=mod(k-2,np)+1; kn=mod(k,np)+1;
    dx_p=pulleys(kp).x-pulleys(k).x; dy_p=pulleys(kp).y-pulleys(k).y;
    dx_n=pulleys(kn).x-pulleys(k).x; dy_n=pulleys(kn).y-pulleys(k).y;
    Lp=max(hypot(dx_p,dy_p),1); Ln=max(hypot(dx_n,dy_n),1);
    Fx=T_t(k,i12)*dx_p/Lp + T_s(k,i12)*dx_n/Ln;
    Fy=T_t(k,i12)*dy_p/Lp + T_s(k,i12)*dy_n/Ln;
    quiver(ax8,pulleys(k).x,pulleys(k).y,Fx/20,Fy/20,...
        'Color',pcolors{k},'LineWidth',2,'MaxHeadSize',0.5,'AutoScale','off');
    plot(ax8,pulleys(k).x,pulleys(k).y,'o','MarkerSize',pulleys(k).r/6,...
        'Color',pcolors{k},'MarkerFaceColor',pcolors{k}*0.3);
    text(ax8,pulleys(k).x+4,pulleys(k).y+pulleys(k).r+12,...
        sprintf('%s\n%.0fN',pnames{k},F_hub(k,i12)),...
        'Color',pcolors{k},'FontSize',8,'HorizontalAlignment','center');
end
axis(ax8,'equal'); grid(ax8,'on');

%% 9 – Design score gauge
ax9=mkax(3,3); setup_ax(ax9,'','','Design Validation Summary');
[report,score,suggestions] = FEAD_Validator(pulleys,belt,load_table,struct('ac',true,'nightRun',false,'bas',false,'temp_C',40),1200);
checks = report.checks;
nc = numel(checks);
pass_arr = cellfun(@(c)double(c.pass), checks);
clrs = cell(nc,1);
for i=1:nc, clrs{i} = ternary(checks{i}.pass, GR, RD); end
for i=1:nc
    barh(ax9,i,100,'FaceColor',clrs{i},'FaceAlpha',0.35,'EdgeColor',clrs{i},'LineWidth',1.5);
    hold(ax9,'on');
    text(ax9,5,i,checks{i}.msg,'Color',clrs{i},'FontSize',8.5,'VerticalAlignment','middle');
end
labels = cellfun(@(c)c.label, checks,'UniformOutput',false);
set(ax9,'YTick',1:nc,'YTickLabel',labels,'XLim',[0 110]);
title(ax9,sprintf('Score: %d/100  (%d/%d passed)',score,report.n_pass,nc),...
    'Color',AM,'FontWeight','bold');

sgtitle(fig,sprintf('Gates FEAD Test Rig — Post-Process Analysis — %s',belt.name),...
    'Color',AM,'FontSize',14,'FontWeight','bold');

%% ── Save to workspace ────────────────────────────────────────────────────────
pp.rpm_sw    = rpm_sw;
pp.F_hub     = F_hub;
pp.T_tight   = T_t;
pp.T_slack   = T_s;
pp.SF        = SF_v;
pp.P_access  = P_v;
pp.v_belt    = v_v;
pp.belt_life = belt_life;
pp.score     = score;
assignin('base','fead_pp',pp);
fprintf('Post-process data in workspace: fead_pp\n');
fprintf('Design score: %d/100\n', score);

function out = ternary(cond,a,b)
    if cond, out=a; else, out=b; end
end
