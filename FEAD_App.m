%% FEAD_App.m  –  FEAD Belt Drive Test Rig – Complete Interactive App
%  Fast live results via pre-computed vectorised sweep.
%  All physics computed analytically (no Validator loop per plot point).
%  Usage:  >> FEAD_params;  FEAD_App
% ─────────────────────────────────────────────────────────────────────────────

function FEAD_App()

%% ── Load parameters from workspace or run FEAD_params ─────────────────────
if ~exist('pulleys','var') || ~exist('belt','var')
    FEAD_params;
end

pulleys_ws = evalin('base','pulleys');
belt_ws    = evalin('base','belt');
lt_ws      = evalin('base','load_table');
wp_ws      = evalin('base','wp');
ten_ws     = evalin('base','ten_pos');

belt_lib   = FEAD_BeltLibrary();

%% ── App state struct ───────────────────────────────────────────────────────
S.pulleys   = pulleys_ws;
S.belt      = belt_lib(1);
S.belt.static_tension = belt_ws.static_tension;
S.belt.length_m       = belt_ws.length_m;
S.belt.length_mm      = belt_ws.length_m * 1000;
S.load_table= lt_ws;
S.wp        = wp_ws;
S.ten_pos   = ten_ws;
S.rpm       = 1200;
S.ten_idx   = 4;
S.conditions= struct('ac',true,'nightRun',false,'bas',false,'temp_C',40);
S.dragging  = -1;
S.running   = false;
S.dw        = [];
S.ang_off   = zeros(1,6);
S.cache     = [];   % pre-computed sweep cache

pnames  = {'CRK','FAN','IDR','ALT','AC','TEN'};
np      = 6;
rpm_sw  = (300:50:2500)';   % fixed sweep vector

%% ── Colours ────────────────────────────────────────────────────────────────
BG=[0.06 0.08 0.14]; AX=[0.09 0.12 0.20]; TC=[0.89 0.91 0.94];
AM=[0.96 0.62 0.04]; VI=[0.65 0.54 0.98]; GR=[0.20 0.85 0.60];
RD=[0.94 0.27 0.27]; PK=[0.96 0.28 0.71];
PC={AM,[0.55 0.36 0.96],VI,GR,PK,[0.38 0.64 0.98]};

%% ═══ FIGURE ════════════════════════════════════════════════════════════════
fig = uifigure('Name','FEAD Belt Drive Test Rig  –  H6 Engine',...
    'Position',[20 30 1840 980],'Color',BG,'Resize','on',...
    'CloseRequestFcn',@on_close);

%% ═══ LEFT PANEL ════════════════════════════════════════════════════════════
lp = uipanel(fig,'Position',[5 5 330 970],'BackgroundColor',AX,'BorderType','none');
yc = 940;

lbl = @(txt,clr,fz) uilabel(lp,'Text',txt,'Position',[5 yc 320 22],...
    'FontSize',fz,'FontColor',clr,'BackgroundColor',AX,...
    'HorizontalAlignment','center');

lbl('FEAD Test Rig Controls',AM,13); yc=yc-28;

% Belt selector
lbl('── Belt Selection ──',VI,10); yc=yc-24;
S.dd_belt = uidropdown(lp,'Position',[5 yc 320 28],...
    'Items',{belt_lib.name},'Value',belt_lib(1).name,...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(s,~)on_belt(s.Value)); yc=yc-30;
S.lbl_binfo = uilabel(lp,'Position',[5 yc 320 18],...
    'Text',belt_info_str(S.belt),...
    'FontSize',9,'FontColor',[0.55 0.62 0.72],'BackgroundColor',AX);
yc=yc-26;

% RPM slider
lbl('── Engine RPM ──',VI,10); yc=yc-24;
S.lbl_rpm=uilabel(lp,'Position',[228 yc 90 22],'Text','1200 RPM',...
    'FontColor',AM,'FontWeight','bold','BackgroundColor',AX);
S.sl_rpm=uislider(lp,'Position',[5 yc+9 218 3],'Limits',[400 2500],'Value',1200,...
    'MajorTicks',[500 1000 1500 2000 2500],...
    'ValueChangedFcn',@(s,~)on_rpm(s.Value)); yc=yc-40;

% Tension slider
lbl('── Static Tension (N) ──',VI,10); yc=yc-24;
S.lbl_ten=uilabel(lp,'Position',[228 yc 90 22],'Text','480 N',...
    'FontColor',AM,'FontWeight','bold','BackgroundColor',AX);
S.sl_ten=uislider(lp,'Position',[5 yc+9 218 3],'Limits',[100 1000],'Value',480,...
    'MajorTicks',[200 400 600 800 1000],...
    'ValueChangedFcn',@(s,~)on_ten(s.Value)); yc=yc-40;

% Tensioner position
lbl('── Tensioner Position ──',VI,10); yc=yc-24;
S.dd_tenpos=uidropdown(lp,'Position',[5 yc 320 28],...
    'Items',{S.ten_pos.label},'Value','MEAN',...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(s,~)on_tenpos(s.Value)); yc=yc-36;

% Operating conditions checkboxes
lbl('── Operating Conditions ──',VI,10); yc=yc-24;
S.cb_ac   =uicheckbox(lp,'Text','AC Compressor ON','Value',1,...
    'Position',[10 yc 210 22],'FontColor',TC,'BackgroundColor',AX,...
    'ValueChangedFcn',@(s,~)on_cond('ac',logical(s.Value))); yc=yc-26;
S.cb_nite =uicheckbox(lp,'Text','Night Run (Alternator max)','Value',0,...
    'Position',[10 yc 230 22],'FontColor',TC,'BackgroundColor',AX,...
    'ValueChangedFcn',@(s,~)on_cond('nightRun',logical(s.Value))); yc=yc-26;
S.cb_bas  =uicheckbox(lp,'Text','BAS (Belt-Alt-Starter) mode','Value',0,...
    'Position',[10 yc 240 22],'FontColor',TC,'BackgroundColor',AX,...
    'ValueChangedFcn',@(s,~)on_cond('bas',logical(s.Value))); yc=yc-28;
uilabel(lp,'Text','Temp (°C):','Position',[10 yc 90 22],...
    'FontColor',TC,'BackgroundColor',AX);
S.ef_temp=uieditfield(lp,'numeric','Value',40,'Limits',[-40 150],...
    'Position',[105 yc 80 22],...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(s,~)on_cond('temp_C',s.Value)); yc=yc-34;

% Pulley layout table
lbl('── Pulley Layout Datum ──',VI,10); yc=yc-20;
uilabel(lp,'Text','  Pulley    X(mm)   Y(mm)   R(mm)',...
    'Position',[5 yc 320 16],'FontSize',8,...
    'FontColor',[0.45 0.55 0.65],'BackgroundColor',AX); yc=yc-20;
S.ef_px=cell(np,1); S.ef_py=cell(np,1); S.ef_pr=cell(np,1);
for k=1:np
    c=PC{k};
    uilabel(lp,'Text',pnames{k},'Position',[5 yc 38 22],...
        'FontColor',c,'FontWeight','bold','BackgroundColor',AX);
    S.ef_px{k}=uieditfield(lp,'numeric','Value',S.pulleys(k).x,'Limits',[-600 600],...
        'Position',[44 yc 80 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(s,~)on_pedit(k,'x',s.Value));
    S.ef_py{k}=uieditfield(lp,'numeric','Value',S.pulleys(k).y,'Limits',[-100 700],...
        'Position',[128 yc 80 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(s,~)on_pedit(k,'y',s.Value));
    S.ef_pr{k}=uieditfield(lp,'numeric','Value',S.pulleys(k).r,'Limits',[10 150],...
        'Position',[212 yc 80 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(s,~)on_pedit(k,'r',s.Value));
    yc=yc-24;
end
yc=yc-6;

% Buttons  (2 columns × 4 rows)
bw=155; bh=30; bg=5;
btns = {
    '▶ Animate',  [0.08 0.20 0.12], GR,  @toggle_anim;
    '✔ Validate', [0.08 0.12 0.24], VI,  @do_validate;
    '⚙ Optimise', [0.12 0.08 0.24], VI,  @do_optimise;
    '📊 Data Win',[0.10 0.15 0.28], [0.38 0.64 0.98], @open_dw;
    '🔧 Simscape', [0.16 0.10 0.06], AM, @do_simscape;
    '⬆ GitHub',   [0.06 0.14 0.10], GR,  @do_github;
    '↺ Reset',    AX,               TC,  @do_reset;
    '⬇ Web Import',AX,              TC,  @do_import;
};
btn_handles=cell(size(btns,1),1);
for bi=1:size(btns,1)
    col=mod(bi-1,2); row=floor((bi-1)/2);
    xb=5+col*(bw+bg); yb=yc-row*(bh+bg);
    btn_handles{bi}=uibutton(lp,'push','Text',btns{bi,1},...
        'Position',[xb yb bw bh],'BackgroundColor',btns{bi,2},...
        'FontColor',btns{bi,3},'FontWeight','bold',...
        'ButtonPushedFcn',btns{bi,4});
end
S.btn_anim=btn_handles{1};

yc = yc - 4*(bh+bg) - 10;
S.lbl_score=uilabel(lp,'Position',[5 yc 320 34],...
    'Text','Design Score:  —','FontSize',14,'FontWeight','bold',...
    'FontColor',GR,'HorizontalAlignment','center','BackgroundColor',AX);

%% ═══ CENTRE: Animation Canvas ══════════════════════════════════════════════
cp=uipanel(fig,'Position',[340 5 840 970],'BackgroundColor',BG,'BorderType','none');
ax_anim=uiaxes(cp,'Position',[5 150 830 815]);
ax_anim.Color=[0.05 0.08 0.12];
ax_anim.XColor=TC; ax_anim.YColor=TC;
ax_anim.GridColor=[0.10 0.16 0.22]; ax_anim.GridAlpha=0.8;
ax_anim.XGrid='on'; ax_anim.YGrid='on';
ax_anim.XLim=[-420 230]; ax_anim.YLim=[-100 550];

% Per-pulley badge labels below canvas
for k=1:np
    S.badge{k}=uilabel(cp,'Position',[(k-1)*138+5 10 133 32],...
        'Text',sprintf('%s\n—  N',pnames{k}),...
        'FontSize',9,'FontWeight','bold','FontColor',PC{k},...
        'BackgroundColor',[0.08 0.12 0.20],'HorizontalAlignment','center');
end

%% ═══ RIGHT PANEL: 5-Tab Results ════════════════════════════════════════════
rp=uipanel(fig,'Position',[1185 5 650 970],'BackgroundColor',BG,'BorderType','none');
uilabel(rp,'Text','Live Results','Position',[5 940 640 24],...
    'FontSize',13,'FontWeight','bold','FontColor',AM,...
    'HorizontalAlignment','center','BackgroundColor',BG);
tgr=uitabgroup(rp,'Position',[5 5 640 930]);

function ax=maketab(title_str)
    tab=uitab(tgr,'Title',title_str,'BackgroundColor',AX);
    ax=uiaxes(tab,'Position',[5 5 625 875]);
    ax.Color=[0.07 0.10 0.17]; ax.XColor=TC; ax.YColor=TC;
    ax.GridColor=[0.15 0.22 0.32]; ax.GridAlpha=0.6;
    ax.XGrid='on'; ax.YGrid='on'; ax.FontSize=9;
end

S.ax_hub  = maketab('Hub Loads');
S.ax_tens = maketab('Tensions');
S.ax_sf   = maketab('Slip SF');
S.ax_life = maketab('Belt Life');

tab_val=uitab(tgr,'Title','Validation','BackgroundColor',AX);
S.ax_val =uiaxes(tab_val,'Position',[5 180 625 695]);
S.ax_val.Color=[0.07 0.10 0.17]; S.ax_val.XColor=TC; S.ax_val.YColor=TC;
S.tx_sug =uitextarea(tab_val,'Position',[5 5 625 170],...
    'Editable','off','BackgroundColor',[0.07 0.10 0.17],'FontColor',TC,'FontSize',9);

%% ── Initialise animation graphics ─────────────────────────────────────────
S.anim = init_anim(ax_anim, S.pulleys, PC);

%% ── Animation timer ───────────────────────────────────────────────────────
S.timer=timer('Name','FEAD_AppTimer','Period',1/25,'ExecutionMode','fixedRate',...
    'TimerFcn',@(~,~)anim_step());

%% ── Mouse drag callbacks ───────────────────────────────────────────────────
fig.WindowButtonDownFcn  =@on_mdown;
fig.WindowButtonMotionFcn=@on_mmove;
fig.WindowButtonUpFcn    =@on_mup;

%% ── Initial compute ───────────────────────────────────────────────────────
recompute();

fprintf('FEAD App ready. Use left panel controls.\n');

%% ══════════════════════════════════════════════════════════════════════════
%  CORE PHYSICS ENGINE  (vectorised – called once per parameter change)
%% ══════════════════════════════════════════════════════════════════════════

function C = compute_sweep(pl, bl, lt, cond, rpm_vec)
    % Returns struct with arrays [6 × N_rpm]:
    %   F_hub, T_tight, T_slack, SF, P_kW, v_belt, belt_life [6×1], score, checks, suggestions
    nr  = numel(rpm_vec);
    npp = numel(pl);
    wrap_d = [166.5 127.6 108.4 145.1 105.7 76.4];

    F_hub   = zeros(npp,nr);
    T_t     = zeros(npp,nr);
    T_s     = zeros(npp,nr);
    SF_arr  = zeros(npp,nr);
    P_arr   = zeros(npp,nr);
    v_arr   = zeros(1,nr);

    for ri = 1:nr
        rpm = rpm_vec(ri);
        v   = max(pl(1).r/1000 * rpm*2*pi/60, 0.01);
        v_arr(ri) = v;
        T_c = bl.lin_mass * v^2;

        for k = 1:npp
            P_k = max(interp1(lt.rpm, lt.(pnames{k}), rpm,'linear','extrap'), 0);
            if strcmp(pnames{k},'AC') && ~cond.ac,        P_k=0; end
            if strcmp(pnames{k},'ALT') && cond.nightRun, P_k=max(P_k,3.7); end
            if strcmp(pnames{k},'CRK') && cond.bas, v_eff=max(v,0.01); P_k=P_k+5000/v_eff/1000; end
            P_arr(k,ri) = P_k;
            T_eff = P_k*1000/v;
            T_t(k,ri) = bl.static_tension + T_eff/2 + T_c;
            T_s(k,ri) = max(bl.static_tension - T_eff/2, 0) + T_c;
            mu_th = bl.mu * wrap_d(k)*pi/180;
            if T_s(k,ri) > 0.1
                SF_arr(k,ri) = min(log(T_t(k,ri)/T_s(k,ri)) / mu_th, 9.99);
            else
                SF_arr(k,ri) = 9.99;
            end
            kp=mod(k-2,npp)+1; kn=mod(k,npp)+1;
            dx_p=pl(kp).x-pl(k).x; dy_p=pl(kp).y-pl(k).y;
            dx_n=pl(kn).x-pl(k).x; dy_n=pl(kn).y-pl(k).y;
            Lp=max(hypot(dx_p,dy_p),1); Ln=max(hypot(dx_n,dy_n),1);
            Fx=T_t(k,ri)*dx_p/Lp + T_s(k,ri)*dx_n/Ln;
            Fy=T_t(k,ri)*dy_p/Lp + T_s(k,ri)*dy_n/Ln;
            F_hub(k,ri) = hypot(Fx,Fy);
        end
    end

    % Belt life (WLTC weighted)
    wltc_rpm=[900 1200 1600 2000]; ww=[.25 .25 .25 .25];
    belt_life=zeros(npp,1);
    for k=1:npp
        D=0;
        for wi=1:4
            vi=max(pl(1).r/1000*wltc_rpm(wi)*2*pi/60,0.01);
            Pi=max(interp1(lt.rpm,lt.(pnames{k}),wltc_rpm(wi),'linear','extrap'),0);
            Tti=bl.static_tension+Pi*1000/vi/2+bl.lin_mass*vi^2;
            Nfi=bl.wohler_Nref*(bl.wohler_Tref/max(Tti,1))^bl.wohler_m;
            D=D+ww(wi)/Nfi;
        end
        belt_life(k)=min(bl.length_m/1000/max(D,1e-15),500000);
    end

    % 8-check validation at sim_rpm
    [report,score,suggestions] = FEAD_Validator(pl,bl,lt,cond,1200);
    report.suggestions = suggestions;
    report.belt_life_km= belt_life;

    C.F_hub     = F_hub;
    C.T_tight   = T_t;
    C.T_slack   = T_s;
    C.SF        = SF_arr;
    C.P_kW      = P_arr;
    C.v_belt    = v_arr;
    C.belt_life = belt_life;
    C.score     = score;
    C.report    = report;
    C.suggestions = suggestions;
end

%% ── Recompute everything ───────────────────────────────────────────────────
function recompute()
    S.cache = compute_sweep(S.pulleys, S.belt, S.load_table, S.conditions, rpm_sw);
    update_plots();
    update_score();
    update_badges();
    if ~isempty(S.dw) && isvalid(S.dw.fig)
        FEAD_DataWindow_Update(S.dw, S.cache.report, S.belt, S.rpm, S.wp, S.load_table, S.pulleys);
    end
end

%% ── Update all 5 right-panel plots ─────────────────────────────────────────
function update_plots()
    C = S.cache;
    pdf_F=[2658.9 2866.4 1710.1 1678.1 985.8 608.5];

    %% Hub loads
    cla(S.ax_hub); hold(S.ax_hub,'on');
    for k=1:np
        plot(S.ax_hub,rpm_sw,C.F_hub(k,:),'LineWidth',2,'Color',PC{k},'DisplayName',pnames{k});
    end
    for k=1:np, plot(S.ax_hub,1200,pdf_F(k),'o','MarkerSize',7,...
            'Color',PC{k},'MarkerFaceColor',PC{k},'HandleVisibility','off'); end
    xline(S.ax_hub,S.rpm,'--','Color',[1 1 1 0.3],'HandleVisibility','off');
    xlabel(S.ax_hub,'RPM','Color',TC); ylabel(S.ax_hub,'F (N)','Color',TC);
    title(S.ax_hub,'Hub Loads vs RPM','Color',AM,'FontWeight','bold');
    legend(S.ax_hub,'TextColor',TC,'Color',AX,'Location','northwest','FontSize',8);

    %% Tensions
    cla(S.ax_tens); hold(S.ax_tens,'on');
    for k=1:np
        plot(S.ax_tens,rpm_sw,C.T_tight(k,:),'LineWidth',2,'Color',PC{k},'DisplayName',[pnames{k},' Tt']);
        plot(S.ax_tens,rpm_sw,C.T_slack(k,:),'--','LineWidth',1,'Color',PC{k}*0.55,'HandleVisibility','off');
    end
    yline(S.ax_tens,S.belt.T_max,'r--','LineWidth',2,'Label',sprintf('T_{max}=%.0fN',S.belt.T_max),...
        'LabelHorizontalAlignment','left','HandleVisibility','off');
    yline(S.ax_tens,S.belt.static_tension,'Color',AM,'LineStyle',':','LineWidth',1.5,...
        'Label','T_0','HandleVisibility','off');
    xlabel(S.ax_tens,'RPM','Color',TC); ylabel(S.ax_tens,'Tension (N)','Color',TC);
    title(S.ax_tens,'Belt Tensions: Tight (solid) / Slack (dashed)','Color',AM,'FontWeight','bold');
    legend(S.ax_tens,'TextColor',TC,'Color',AX,'Location','northwest','FontSize',8,'NumColumns',2);

    %% Slip SF
    cla(S.ax_sf); hold(S.ax_sf,'on');
    for k=1:np
        plot(S.ax_sf,rpm_sw,min(C.SF(k,:),6),'LineWidth',2,'Color',PC{k},'DisplayName',pnames{k});
    end
    yline(S.ax_sf,1.0,'r-','LineWidth',2,'Label','SF=1 SLIP','HandleVisibility','off');
    yline(S.ax_sf,1.3,'Color',AM,'LineStyle','--','LineWidth',1.5,'Label','SF=1.3','HandleVisibility','off');
    yline(S.ax_sf,2.0,'Color',GR,'LineStyle',':','LineWidth',1,'Label','SF=2.0 Safe','HandleVisibility','off');
    ylim(S.ax_sf,[0 6]);
    xlabel(S.ax_sf,'RPM','Color',TC); ylabel(S.ax_sf,'Slip Safety Factor','Color',TC);
    title(S.ax_sf,'Capstan Slip SF vs RPM','Color',AM,'FontWeight','bold');
    legend(S.ax_sf,'TextColor',TC,'Color',AX,'Location','northeast','FontSize',8);

    %% Belt life bar
    cla(S.ax_life);
    b=bar(S.ax_life,C.belt_life/1000,'FaceColor','flat','EdgeColor','none');
    for k=1:np, b.CData(k,:)=PC{k}; end
    S.ax_life.XTickLabel=pnames; S.ax_life.XColor=TC;
    yline(S.ax_life,200,'--','Color',AM,'LineWidth',1.5,...
        'Label','Min 200k km','LabelHorizontalAlignment','right');
    xlabel(S.ax_life,'Pulley','Color',TC); ylabel(S.ax_life,'Life (×10³ km)','Color',TC);
    title(S.ax_life,sprintf('Belt Fatigue Life – %s',S.belt.name),'Color',AM,'FontWeight','bold');
    for k=1:np
        text(S.ax_life,k,C.belt_life(k)/1000+5,...
            sprintf('%.0fk',C.belt_life(k)/1000),'Color',PC{k},...
            'HorizontalAlignment','center','FontSize',8,'FontWeight','bold');
    end

    %% Validation
    report  = C.report;
    checks  = report.checks;
    nc      = numel(checks);
    cla(S.ax_val); hold(S.ax_val,'on');
    for i=1:nc
        clr=ternary(checks{i}.pass,GR,RD);
        barh(S.ax_val,i,100,'FaceColor',clr,'FaceAlpha',0.3,'EdgeColor',clr,'LineWidth',1.5);
        tick_txt = sprintf('%s  →  %s',checks{i}.label, checks{i}.msg);
        text(S.ax_val,3,i,tick_txt,'Color',clr,'FontSize',9,'VerticalAlignment','middle');
    end
    set(S.ax_val,'YTick',1:nc,'YTickLabel',repmat({' '},1,nc),...
        'XLim',[0 115],'XColor',TC,'YColor',TC);
    title(S.ax_val,...
        sprintf('Validation  –  Score: %d/100  (%d/%d checks passed)',...
        report.score, report.n_pass, nc),'Color',AM,'FontWeight','bold');

    S.tx_sug.Value = C.suggestions;

    drawnow limitrate;
end

%% ── Update score badge ─────────────────────────────────────────────────────
function update_score()
    sc = S.cache.score;
    if sc>=80,    c=GR; elseif sc>=60, c=AM; else, c=RD; end
    S.lbl_score.Text      = sprintf('Design Score:  %d / 100', sc);
    S.lbl_score.FontColor = c;
end

%% ── Update per-pulley badges ───────────────────────────────────────────────
function update_badges()
    [~,ri]=min(abs(rpm_sw-S.rpm));
    for k=1:np
        S.badge{k}.Text=sprintf('%s\n%.0f N',pnames{k},S.cache.F_hub(k,ri));
    end
end

%% ═══ ANIMATION ENGINE ══════════════════════════════════════════════════════
function ha = init_anim(ax, pl, colors)
    cla(ax); hold(ax,'on');
    th  = linspace(0,2*pi,96);
    npl = numel(pl);
    ha.th = th; ha.h_disc=gobjects(npl,1); ha.h_hub=gobjects(npl,1);
    ha.h_ring=gobjects(npl,1); ha.h_spk=cell(npl,1);
    ha.h_span=gobjects(npl,1); ha.h_arr=gobjects(npl,1);
    ha.h_atxt=gobjects(npl,1); ha.h_lbl=gobjects(npl,1);
    ha.h_mrk=gobjects(16,1);

    for k=1:npl
        px=pl(k).x; py=pl(k).y; r=pl(k).r; c=colors{k};
        kn=mod(k,npl)+1;
        ha.h_span(k)=plot(ax,[px pl(kn).x],[py pl(kn).y],...
            'Color',[AM 0.5],'LineWidth',4);
        ha.h_disc(k)=patch(ax,px+r*cos(th),py+r*sin(th),c*0.15,...
            'EdgeColor',c,'LineWidth',2,'FaceAlpha',0.9);
        ha.h_ring(k)=patch(ax,px+(r+6)*cos(th),py+(r+6)*sin(th),'none',...
            'EdgeColor',GR,'LineWidth',3);
        ha.h_hub(k) =patch(ax,px+r*.18*cos(th),py+r*.18*sin(th),...
            [0.04 0.06 0.10],'EdgeColor',[0.5 0.6 0.7],'LineWidth',1.5);
        ha.h_spk{k}=gobjects(4,1);
        for s=1:4
            a=(s-1)*pi/2;
            ha.h_spk{k}(s)=plot(ax,...
                [px+r*.18*cos(a) px+r*.85*cos(a)],...
                [py+r*.18*sin(a) py+r*.85*sin(a)],...
                'Color',c,'LineWidth',2.5);
        end
        ha.h_lbl(k)=text(ax,px,py+r+16,pnames{k},...
            'HorizontalAlignment','center','Color',c,...
            'FontSize',9,'FontWeight','bold');
        ha.h_arr(k) =quiver(ax,px,py,0,0,'Color',RD,...
            'LineWidth',2,'MaxHeadSize',0.6,'AutoScale','off');
        ha.h_atxt(k)=text(ax,px+4,py+4,'','Color',[0.97 0.60 0.60],...
            'FontSize',8,'FontWeight','bold');
    end
    for m=1:16
        ha.h_mrk(m)=plot(ax,0,0,'s','Color',[1.0 0.82 0.25],...
            'MarkerSize',4.5,'MarkerFaceColor',[1.0 0.82 0.25]);
    end
    ha.lbl_rpm  =text(ax,-405,520,'RPM: —','Color',AM,'FontSize',11,'FontWeight','bold');
    ha.lbl_vel  =text(ax,-405,493,'Belt v: —','Color',VI,'FontSize',10);
    ha.lbl_t0   =text(ax,-405,466,'T₀: —','Color',[0.38 0.64 0.98],'FontSize',10);
    ha.lbl_score=text(ax,-405,432,'Score: —','Color',GR,'FontSize',11,'FontWeight','bold');
    ha.mrk_pos  =linspace(0,1,16);
end

function anim_step()
    if ~isvalid(fig) || ~S.running, return; end
    dt=1/25; pl=S.pulleys;
    v=pl(1).r/1000*S.rpm*2*pi/60;
    npl=numel(pl);

    for k=1:npl
        r_k=pl(k).r/1000; om=v/r_k;
        if ~pl(k).cw, om=-om; end
        S.ang_off(k)=S.ang_off(k)+om*dt;
        for s=1:4
            a=(s-1)*pi/2+S.ang_off(k);
            px=pl(k).x; py=pl(k).y; r=pl(k).r;
            set(S.anim.h_spk{k}(s),...
                'XData',[px+r*.18*cos(a) px+r*.85*cos(a)],...
                'YData',[py+r*.18*sin(a) py+r*.85*sin(a)]);
        end
    end

    % Belt marker travel
    spans_x=[pl.x pl(1).x]; spans_y=[pl.y pl(1).y];
    cum=zeros(1,npl+1);
    for k=1:npl
        cum(k+1)=cum(k)+hypot(spans_x(k+1)-spans_x(k),spans_y(k+1)-spans_y(k));
    end
    bspeed=v*dt/max(cum(end),1); 
    S.anim.mrk_pos=mod(S.anim.mrk_pos+bspeed,1);
    for m=1:16
        pm=S.anim.mrk_pos(m)*cum(end);
        sg=find(cum<=pm,1,'last'); sg=min(sg,npl);
        fr=(pm-cum(sg))/max(cum(sg+1)-cum(sg),1);
        set(S.anim.h_mrk(m),...
            'XData',spans_x(sg)*(1-fr)+spans_x(sg+1)*fr,...
            'YData',spans_y(sg)*(1-fr)+spans_y(sg+1)*fr);
    end

    % Hub load arrows + slip rings from cache
    if ~isempty(S.cache)
        [~,ri]=min(abs(rpm_sw-S.rpm));
        for k=1:npl
            F=S.cache.F_hub(k,ri); sf=S.cache.SF(k,ri);
            kp=mod(k-2,npl)+1; kn=mod(k,npl)+1;
            dx_p=pl(kp).x-pl(k).x; dy_p=pl(kp).y-pl(k).y;
            dx_n=pl(kn).x-pl(k).x; dy_n=pl(kn).y-pl(k).y;
            Lp=max(hypot(dx_p,dy_p),1); Ln=max(hypot(dx_n,dy_n),1);
            Tt=S.cache.T_tight(k,ri); Ts=S.cache.T_slack(k,ri);
            Fx=Tt*dx_p/Lp+Ts*dx_n/Ln; Fy=Tt*dy_p/Lp+Ts*dy_n/Ln;
            sc=min(F/4500,1)*65+8;
            if F>10
                ex=Fx/max(F,1)*sc; ey=Fy/max(F,1)*sc;
                set(S.anim.h_arr(k),'XData',pl(k).x,'YData',pl(k).y,'UData',ex,'VData',ey);
                set(S.anim.h_atxt(k),'Position',[pl(k).x+ex+4,pl(k).y+ey+4],'String',sprintf('%.0fN',F));
            end
            rc=ternary(sf>=1.3, GR, ternary(sf>=1.0, AM, RD));
            S.anim.h_ring(k).EdgeColor=rc;
        end
        S.anim.lbl_rpm.String  = sprintf('Engine: %d RPM',round(S.rpm));
        S.anim.lbl_vel.String  = sprintf('Belt v: %.2f m/s',v);
        S.anim.lbl_t0.String   = sprintf('T₀: %d N',round(S.belt.static_tension));
        sc=S.cache.score;
        scl=ternary(sc>=80,GR,ternary(sc>=60,AM,RD));
        S.anim.lbl_score.String=sprintf('Score: %d/100',sc);
        S.anim.lbl_score.Color =scl;
    end
    drawnow limitrate;
end

function update_anim_layout()
    pl=S.pulleys; npl=numel(pl);
    th=S.anim.th;
    for k=1:npl
        px=pl(k).x; py=pl(k).y; r=pl(k).r;
        kn=mod(k,npl)+1;
        S.anim.h_disc(k).XData=px+r*cos(th); S.anim.h_disc(k).YData=py+r*sin(th);
        S.anim.h_ring(k).XData=px+(r+6)*cos(th); S.anim.h_ring(k).YData=py+(r+6)*sin(th);
        S.anim.h_hub(k).XData =px+r*.18*cos(th); S.anim.h_hub(k).YData =py+r*.18*sin(th);
        S.anim.h_lbl(k).Position(1:2)=[px,py+r+16];
        S.anim.h_span(k).XData=[px pl(k+mod(0,npl)).x]; % will fix below
    end
    for k=1:npl
        kn=mod(k,npl)+1;
        S.anim.h_span(k).XData=[pl(k).x pl(kn).x];
        S.anim.h_span(k).YData=[pl(k).y pl(kn).y];
    end
end

%% ═══ CALLBACKS ═════════════════════════════════════════════════════════════
function toggle_anim()
    S.running=~S.running;
    if S.running
        S.btn_anim.Text='⏹ Stop'; S.btn_anim.BackgroundColor=[0.22 0.06 0.06];
        if strcmp(S.timer.Running,'off'), start(S.timer); end
    else
        S.btn_anim.Text='▶ Animate'; S.btn_anim.BackgroundColor=[0.08 0.20 0.12];
    end
end

function do_validate()
    recompute();
    r=S.cache.report;
    uialert(fig,sprintf('Score: %d/100\n%d of %d checks passed\n\n%s',...
        r.score,r.n_pass,r.n_total,strjoin(S.cache.suggestions,newline)),...
        'Validation','Icon','info');
end

function do_optimise()
    [~,~,~,opt]=FEAD_Validator(S.pulleys,S.belt,S.load_table,S.conditions,1200);
    if opt(6).x~=S.pulleys(6).x || opt(6).y~=S.pulleys(6).y
        S.pulleys(6).x=opt(6).x; S.pulleys(6).y=opt(6).y;
        S.ef_px{6}.Value=opt(6).x; S.ef_py{6}.Value=opt(6).y;
        update_anim_layout(); recompute();
        uialert(fig,sprintf('Tensioner → X=%.0f, Y=%.0f mm\nHub loads minimised.',opt(6).x,opt(6).y),...
            'Optimised','Icon','success');
    else
        uialert(fig,'Tensioner already at optimal position.','Optimise','Icon','info');
    end
end

function open_dw()
    if isempty(S.dw)||~isvalid(S.dw.fig), S.dw=FEAD_DataWindow(); end
    figure(S.dw.fig);
    FEAD_DataWindow_Update(S.dw,S.cache.report,S.belt,S.rpm,S.wp,S.load_table,S.pulleys);
end

function do_simscape()
    assignin('base','pulleys',S.pulleys);
    assignin('base','belt',   S.belt);
    try
        FEAD_TestRig_Builder(S.pulleys,S.belt,S.load_table,S.conditions);
        uialert(fig,'FEAD_TestRig.slx built and opened.','Simscape','Icon','success');
    catch e
        uialert(fig,['Build error: ' e.message],'Simscape','Icon','error');
    end
end

function do_github()
    try
        push_to_github(S.pulleys, S.belt, S.cache.report);
    catch e
        uialert(fig,['GitHub push error: ' e.message],'GitHub','Icon','error');
    end
end

function do_reset()
    FEAD_params;
    S.pulleys=evalin('base','pulleys');
    for k=1:np
        S.ef_px{k}.Value=S.pulleys(k).x;
        S.ef_py{k}.Value=S.pulleys(k).y;
        S.ef_pr{k}.Value=S.pulleys(k).r;
    end
    update_anim_layout(); recompute();
end

function do_import()
    [f,p]=uigetfile('*.json','Select web tool JSON');
    if isnumeric(f), return; end
    try
        d=jsondecode(fileread(fullfile(p,f)));
        if isfield(d,'pulleys')
            fn=fieldnames(d.pulleys);
            for ki=1:min(numel(fn),np)
                pf=d.pulleys.(fn{ki});
                idx=find(strcmp(pnames,fn{ki}),1);
                if isempty(idx), continue; end
                if isfield(pf,'x'), S.pulleys(idx).x=pf.x; S.ef_px{idx}.Value=pf.x; end
                if isfield(pf,'y'), S.pulleys(idx).y=pf.y; S.ef_py{idx}.Value=pf.y; end
                if isfield(pf,'r'), S.pulleys(idx).r=pf.r; S.ef_pr{idx}.Value=pf.r; end
            end
        end
        if isfield(d,'rpm'),           S.rpm=d.rpm; S.sl_rpm.Value=S.rpm; on_rpm(S.rpm); end
        if isfield(d,'staticTension'), S.belt.static_tension=d.staticTension; S.sl_ten.Value=d.staticTension; end
        update_anim_layout(); recompute();
        uialert(fig,'Import successful.','Import','Icon','success');
    catch e
        uialert(fig,['Import failed: ' e.message],'Import','Icon','error');
    end
end

function on_belt(val)
    idx=find(strcmp({belt_lib.name},val),1);
    if isempty(idx), return; end
    S.belt=belt_lib(idx);
    S.belt.static_tension=S.sl_ten.Value;
    S.belt.length_m=S.belt.length_mm/1000;
    S.lbl_binfo.Text=belt_info_str(S.belt);
    recompute();
end

function on_rpm(val)
    S.rpm=round(val/50)*50;
    S.sl_rpm.Value=S.rpm;
    S.lbl_rpm.Text=sprintf('%d RPM',S.rpm);
    update_badges();
    % Vertical line on hub/tens/SF plots
    try
        ax_list={S.ax_hub,S.ax_tens,S.ax_sf};
        for ai=1:3
            ax_i=ax_list{ai};
            kids=ax_i.Children;
            for ki=1:numel(kids)
                if isprop(kids(ki),'LineStyle') && strcmp(kids(ki).LineStyle,'--') && numel(kids(ki).XData)==2 && kids(ki).XData(1)==kids(ki).XData(2)
                    kids(ki).XData=[S.rpm S.rpm];
                end
            end
        end
    catch, end
    drawnow limitrate;
end

function on_ten(val)
    S.belt.static_tension=round(val/10)*10;
    S.lbl_ten.Text=sprintf('%d N',S.belt.static_tension);
    recompute();
end

function on_tenpos(val)
    idx=find(strcmp({S.ten_pos.label},val),1);
    if isempty(idx), return; end
    S.ten_idx=idx;
    S.pulleys(6).x=S.ten_pos(idx).x; S.pulleys(6).y=S.ten_pos(idx).y;
    S.belt.static_tension=S.ten_pos(idx).T;
    S.ef_px{6}.Value=S.pulleys(6).x; S.ef_py{6}.Value=S.pulleys(6).y;
    S.sl_ten.Value=S.belt.static_tension;
    S.lbl_ten.Text=sprintf('%d N',round(S.belt.static_tension));
    update_anim_layout(); recompute();
end

function on_cond(field,val)
    S.conditions.(field)=val;
    recompute();
end

function on_pedit(k,field,val)
    S.pulleys(k).(field)=val;
    update_anim_layout(); recompute();
end

function on_mdown(~,~)
    cp=ax_anim.CurrentPoint;
    if isempty(cp), return; end
    for k=1:np
        if hypot(cp(1,1)-S.pulleys(k).x,cp(1,2)-S.pulleys(k).y)<S.pulleys(k).r*1.5
            S.dragging=k; return;
        end
    end
end

function on_mmove(~,~)
    if S.dragging<1, return; end
    cp=ax_anim.CurrentPoint;
    if isempty(cp), return; end
    k=S.dragging;
    S.pulleys(k).x=round(cp(1,1));
    S.pulleys(k).y=round(cp(1,2));
    S.ef_px{k}.Value=S.pulleys(k).x;
    S.ef_py{k}.Value=S.pulleys(k).y;
    update_anim_layout();
    drawnow limitrate;
end

function on_mup(~,~)
    if S.dragging>0
        S.dragging=-1;
        recompute();
    end
end

function on_close(~,~)
    if strcmp(S.timer.Running,'on'), stop(S.timer); end
    delete(S.timer);
    if ~isempty(S.dw)&&isvalid(S.dw.fig), delete(S.dw.fig); end
    delete(fig);
end

end % FEAD_App

%% ── Helpers ─────────────────────────────────────────────────────────────────
function s=belt_info_str(b)
    s=sprintf('L=%.0fmm  μ=%.2f  %d-rib  %s  T_max=%.0fN',...
        b.length_mm,b.mu,b.ribs,b.core,b.T_max);
end

function out=ternary(cond,a,b)
    if cond, out=a; else, out=b; end
end
