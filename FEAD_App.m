%% FEAD_App.m  –  FEAD Belt Drive Test Rig  – Complete Working App
%  Clean single-file app. All inputs via checkboxes/dropdowns/sliders.
%  Simulation results update instantly via vectorised physics.
%  Usage:  >> FEAD_params;  FEAD_App
% ─────────────────────────────────────────────────────────────────────────────
function FEAD_App()

%% ── Load parameters ─────────────────────────────────────────────────────────
if ~evalin('base','exist(''pulleys'',''var'')')
    evalin('base','FEAD_params');
end
PL0  = evalin('base','pulleys');
BLT0 = evalin('base','belt');
LT   = evalin('base','load_table');
WP   = evalin('base','wp');
TPOS = evalin('base','ten_pos');
BLIB = FEAD_BeltLibrary();

pnames   = {'CRK','FAN','IDR','ALT','AC','TEN'};
np       = 6;
rpm_sw   = (300:50:2500)';
wrap_deg = [166.5 127.6 108.4 145.1 105.7 76.4];
pdf_F    = [2658.9 2866.4 1710.1 1678.1 985.8 608.5];

%% ── State ───────────────────────────────────────────────────────────────────
S.pulleys = PL0;
S.belt    = BLIB(1);
S.belt.static_tension = BLT0.static_tension;
S.belt.length_m       = BLT0.length_m;
S.belt.length_mm      = BLT0.length_m * 1000;
S.belt.T_max          = BLIB(1).T_max;
S.rpm     = 1200;
S.ac_on   = true;
S.night   = false;
S.bas     = false;
S.eng_brk = false;
S.temp_C  = 40;
S.ten_idx = 4;        % MEAN
S.dragging= -1;
S.running = false;
S.dw      = [];
S.ang     = zeros(1,np);
S.mrk_pos = linspace(0,1,16);
S.cache   = [];

%% ── Colours ─────────────────────────────────────────────────────────────────
BG=[0.06 0.08 0.14]; PANEL=[0.09 0.12 0.20]; TC=[0.89 0.91 0.94];
AM=[0.96 0.62 0.04]; VI=[0.65 0.54 0.98];    GR=[0.20 0.85 0.60];
RD=[0.94 0.27 0.27]; PK=[0.96 0.28 0.71];
PC={AM,[0.55 0.36 0.96],VI,GR,PK,[0.38 0.64 0.98]};

%% ════════════════════════════════════════════════════════════════════════════
%  FIGURE  1840×980
%% ════════════════════════════════════════════════════════════════════════════
fig = uifigure('Name','FEAD Belt Drive Test Rig  –  Ashok Leyland H6',...
    'Position',[20 30 1840 980],'Color',BG,'Resize','on',...
    'CloseRequestFcn',@cb_close);

%% ════════════════════════════════════════════════════════════════════════════
%  LEFT PANEL  (x:5 y:5 w:340 h:970)
%% ════════════════════════════════════════════════════════════════════════════
lp = uipanel(fig,'Position',[5 5 340 970],'BackgroundColor',PANEL,'BorderType','none');

y = 940;  % cursor

% ── Title ─────────────────────────────────────────────────────────────────
uilabel(lp,'Text','FEAD Test Rig Controls','Position',[5 y 330 26],...
    'FontSize',13,'FontWeight','bold','FontColor',AM,...
    'HorizontalAlignment','center','BackgroundColor',PANEL);
y = y-30;

% ══ BELT SELECTION ══════════════════════════════════════════════════════════
uilabel(lp,'Text','─── Belt Type ───','Position',[5 y 330 20],...
    'FontSize',10,'FontColor',VI,'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-24;

belt_names = {BLIB.name};
S.dd_belt = uidropdown(lp,'Position',[5 y 330 28],...
    'Items',belt_names,'Value',belt_names{1},...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@cb_belt); y=y-24;

S.lbl_belt = uilabel(lp,'Position',[5 y 330 18],...
    'Text',beltstr(S.belt),...
    'FontSize',8,'FontColor',[0.55 0.62 0.72],'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-28;

% ══ ENGINE RPM ══════════════════════════════════════════════════════════════
uilabel(lp,'Text','─── Engine RPM ───','Position',[5 y 330 20],...
    'FontSize',10,'FontColor',VI,'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-22;

S.lbl_rpm = uilabel(lp,'Position',[235 y 95 22],'Text','1200 RPM',...
    'FontColor',AM,'FontWeight','bold','BackgroundColor',PANEL);
S.sl_rpm = uislider(lp,'Position',[5 y+9 222 3],...
    'Limits',[400 2500],'Value',1200,...
    'MajorTicks',[500 1000 1500 2000 2500],...
    'ValueChangedFcn',@cb_rpm); y=y-38;

% ══ STATIC TENSION ══════════════════════════════════════════════════════════
uilabel(lp,'Text','─── Static Tension (N) ───','Position',[5 y 330 20],...
    'FontSize',10,'FontColor',VI,'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-22;

S.lbl_ten = uilabel(lp,'Position',[235 y 95 22],'Text','480 N',...
    'FontColor',AM,'FontWeight','bold','BackgroundColor',PANEL);
S.sl_ten = uislider(lp,'Position',[5 y+9 222 3],...
    'Limits',[100 1000],'Value',480,...
    'MajorTicks',[200 400 600 800 1000],...
    'ValueChangedFcn',@cb_tension); y=y-38;

% ══ TENSIONER POSITION  (checkbox-style radio group) ═══════════════════════
uilabel(lp,'Text','─── Tensioner Position ───','Position',[5 y 330 20],...
    'FontSize',10,'FontColor',VI,'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-24;

ten_labels = {TPOS.label};
ten_T      = [TPOS.T];
S.bg_ten   = uibuttongroup(lp,'Position',[5 y 330 24],...
    'BackgroundColor',PANEL,'BorderType','none',...
    'SelectionChangedFcn',@cb_tenpos); 
col_w = 55;
S.rb_ten = gobjects(6,1);
for ti=1:6
    S.rb_ten(ti) = uiradiobutton(S.bg_ten,...
        'Text',ten_labels{ti},...
        'Position',[(ti-1)*col_w 2 col_w-2 20],...
        'FontColor',TC,'FontSize',8,'BackgroundColor',PANEL);
end
S.rb_ten(4).Value = true;   % default MEAN
y = y-28;

% tension readout
S.lbl_ten2 = uilabel(lp,'Position',[5 y 330 18],...
    'Text',sprintf('T₀ = 480 N  (MEAN position, arm=15.4mm)'),...
    'FontSize',8,'FontColor',[0.55 0.62 0.72],...
    'BackgroundColor',PANEL,'HorizontalAlignment','center'); y=y-26;

% ══ OPERATING CONDITIONS  (checkboxes) ═════════════════════════════════════
uilabel(lp,'Text','─── Operating Conditions ───','Position',[5 y 330 20],...
    'FontSize',10,'FontColor',VI,'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-24;

S.cb_ac  = uicheckbox(lp,'Text','AC Compressor ON (adds AC torque load)',...
    'Value',1,'Position',[10 y 320 22],...
    'FontColor',TC,'BackgroundColor',PANEL,...
    'ValueChangedFcn',@(s,~)cb_cond('ac',logical(s.Value))); y=y-26;
S.cb_nite= uicheckbox(lp,'Text','Night Run  (Alternator at max 3.7 kW)',...
    'Value',0,'Position',[10 y 320 22],...
    'FontColor',TC,'BackgroundColor',PANEL,...
    'ValueChangedFcn',@(s,~)cb_cond('night',logical(s.Value))); y=y-26;
S.cb_bas = uicheckbox(lp,'Text','BAS mode  (Belt-Alternator-Starter +5000N)',...
    'Value',0,'Position',[10 y 320 22],...
    'FontColor',TC,'BackgroundColor',PANEL,...
    'ValueChangedFcn',@(s,~)cb_cond('bas',logical(s.Value))); y=y-26;
S.cb_ebk = uicheckbox(lp,'Text','Engine Braking  (reduce drive torque 30%)',...
    'Value',0,'Position',[10 y 320 22],...
    'FontColor',TC,'BackgroundColor',PANEL,...
    'ValueChangedFcn',@(s,~)cb_cond('eng_brk',logical(s.Value))); y=y-30;

uilabel(lp,'Text','Operating Temp (°C):','Position',[10 y 160 22],...
    'FontColor',TC,'BackgroundColor',PANEL);
S.ef_temp = uieditfield(lp,'numeric','Value',40,'Limits',[-40 150],...
    'Position',[175 y 80 22],...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(s,~)cb_cond('temp_C',s.Value)); y=y-34;

% ══ PULLEY LAYOUT TABLE ══════════════════════════════════════════════════════
uilabel(lp,'Text','─── Pulley Layout Datum (mm) ───','Position',[5 y 330 20],...
    'FontSize',10,'FontColor',VI,'BackgroundColor',PANEL,...
    'HorizontalAlignment','center'); y=y-20;
uilabel(lp,'Text','  Pulley     X        Y        R  ',...
    'Position',[5 y 330 16],'FontSize',8,...
    'FontColor',[0.45 0.55 0.65],'BackgroundColor',PANEL); y=y-20;

S.ef_x=cell(np,1); S.ef_y=cell(np,1); S.ef_r=cell(np,1);
for k=1:np
    c=PC{k};
    uilabel(lp,'Text',pnames{k},'Position',[5 y 38 22],...
        'FontColor',c,'FontWeight','bold','BackgroundColor',PANEL);
    S.ef_x{k}=uieditfield(lp,'numeric','Value',S.pulleys(k).x,...
        'Limits',[-600 600],'Position',[46 y 82 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(s,~)cb_pedit(k,'x',s.Value));
    S.ef_y{k}=uieditfield(lp,'numeric','Value',S.pulleys(k).y,...
        'Limits',[-100 700],'Position',[132 y 82 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(s,~)cb_pedit(k,'y',s.Value));
    S.ef_r{k}=uieditfield(lp,'numeric','Value',S.pulleys(k).r,...
        'Limits',[10 150],'Position',[218 y 82 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(s,~)cb_pedit(k,'r',s.Value));
    y=y-23;
end
y=y-6;

% ══ DESIGN SCORE BADGE ═══════════════════════════════════════════════════════
S.lbl_score = uilabel(lp,'Position',[5 y 330 36],...
    'Text','Design Score:  —','FontSize',15,'FontWeight','bold',...
    'FontColor',GR,'HorizontalAlignment','center','BackgroundColor',PANEL);
y=y-42;

% ══ BUTTONS  (2 columns) ═════════════════════════════════════════════════════
BW=160; BH=32; BG_=5;
BTNS = {
    '▶  Start Animation',  [0.08 0.20 0.12], GR;
    '✔  Validate Design',  [0.08 0.12 0.24], VI;
    '⚙  Optimise Layout',  [0.12 0.08 0.22], VI;
    '📊  Data Window',     [0.10 0.15 0.28], [0.38 0.64 0.98];
    '🔧  Build Simscape',  [0.16 0.10 0.06], AM;
    '⬆  Push GitHub',      [0.06 0.14 0.10], GR;
    '↺  Reset Datum',      PANEL,             TC;
    '⬇  Import Web JSON',  PANEL,             TC;
};
CBS = {@cb_anim,@cb_validate,@cb_optimise,@cb_dw,@cb_simscape,@cb_github,@cb_reset,@cb_import};
S.btn_anim=[];
for bi=1:size(BTNS,1)
    col=mod(bi-1,2); row=floor((bi-1)/2);
    xb=5+col*(BW+BG_); yb=y-row*(BH+BG_);
    btn=uibutton(lp,'push','Text',BTNS{bi,1},...
        'Position',[xb yb BW BH],'BackgroundColor',BTNS{bi,2},...
        'FontColor',BTNS{bi,3},'FontWeight','bold',...
        'ButtonPushedFcn',CBS{bi});
    if bi==1, S.btn_anim=btn; end
end

%% ════════════════════════════════════════════════════════════════════════════
%  CENTRE PANEL  – Animation Canvas (x:350 w:830)
%% ════════════════════════════════════════════════════════════════════════════
cp = uipanel(fig,'Position',[350 5 830 970],'BackgroundColor',BG,'BorderType','none');
S.ax = uiaxes(cp,'Position',[5 145 820 820]);
S.ax.Color=[0.05 0.08 0.12]; S.ax.XColor=TC; S.ax.YColor=TC;
S.ax.GridColor=[0.10 0.16 0.22]; S.ax.GridAlpha=0.8;
S.ax.XGrid='on'; S.ax.YGrid='on';
S.ax.XLim=[-420 230]; S.ax.YLim=[-100 550];
S.ax.Title.String='FEAD Test Rig – Live Animation';
S.ax.Title.Color=AM;

% Per-pulley badge labels at bottom of canvas
for k=1:np
    S.badge{k}=uilabel(cp,'Position',[(k-1)*136+5 5 131 36],...
        'Text',sprintf('%s\n—',pnames{k}),...
        'FontSize',9,'FontWeight','bold','FontColor',PC{k},...
        'BackgroundColor',[0.08 0.12 0.20],'HorizontalAlignment','center');
end
% Results readout strip
S.txt_strip = uilabel(cp,'Position',[5 42 820 28],...
    'Text','Results will appear after first computation...',...
    'FontSize',9,'FontColor',TC,'BackgroundColor',[0.07 0.10 0.18],...
    'HorizontalAlignment','center');

%% ════════════════════════════════════════════════════════════════════════════
%  RIGHT PANEL – Results tabs (x:1185 w:650)
%% ════════════════════════════════════════════════════════════════════════════
rp = uipanel(fig,'Position',[1185 5 650 970],'BackgroundColor',BG,'BorderType','none');
uilabel(rp,'Text','Simulation Results','Position',[5 940 640 24],...
    'FontSize',13,'FontWeight','bold','FontColor',AM,...
    'HorizontalAlignment','center','BackgroundColor',BG);

tgr = uitabgroup(rp,'Position',[5 5 640 930]);

% Helper: create a dark axes in a tab
function ax = newtab(title_str)
    t=uitab(tgr,'Title',title_str,'BackgroundColor',PANEL);
    ax=uiaxes(t,'Position',[5 5 625 875]);
    ax.Color=[0.07 0.10 0.17]; ax.XColor=TC; ax.YColor=TC;
    ax.GridColor=[0.14 0.20 0.30]; ax.GridAlpha=0.7;
    ax.XGrid='on'; ax.YGrid='on'; ax.FontSize=9;
end

S.ax_hub  = newtab('Hub Loads');
S.ax_tens = newtab('Tensions');
S.ax_sf   = newtab('Slip SF');
S.ax_life = newtab('Belt Life');

% Validation tab
tvl=uitab(tgr,'Title','Validation','BackgroundColor',PANEL);
S.ax_val = uiaxes(tvl,'Position',[5 175 625 700]);
S.ax_val.Color=[0.07 0.10 0.17]; S.ax_val.XColor=TC; S.ax_val.YColor=TC;
S.tx_sug  = uitextarea(tvl,'Position',[5 5 625 168],...
    'Editable','off','BackgroundColor',[0.07 0.10 0.17],'FontColor',TC,'FontSize',9);

% Results table tab
tbl_tab=uitab(tgr,'Title','Results Table','BackgroundColor',PANEL);
S.tbl = uitable(tbl_tab,'Position',[5 5 625 875],...
    'ColumnName',{'Pulley','P (kW)','T_tight (N)','T_slack (N)','F_hub (N)','Slip SF','Life (km)','Status'},...
    'ColumnWidth',{60 65 90 90 80 70 75 60},...
    'RowName',[],'Data',cell(np,8),...
    'BackgroundColor',repmat([PANEL; PANEL*1.25],3,1),...
    'FontColor',TC,'FontSize',10);

%% ════════════════════════════════════════════════════════════════════════════
%  ANIMATION INITIALISATION
%% ════════════════════════════════════════════════════════════════════════════
S.gh = init_graphics(S.ax, S.pulleys);

%% ── Timer ───────────────────────────────────────────────────────────────────
S.tmr = timer('Name','FEAD_Tmr','Period',1/25,'ExecutionMode','fixedRate',...
    'TimerFcn',@(~,~)anim_step());

%% ── Mouse drag ───────────────────────────────────────────────────────────────
fig.WindowButtonDownFcn  = @mdown;
fig.WindowButtonMotionFcn= @mmove;
fig.WindowButtonUpFcn    = @mup;

%% ── First compute ────────────────────────────────────────────────────────────
recompute();
fprintf('FEAD App launched.\n');

%% ════════════════════════════════════════════════════════════════════════════
%  PHYSICS ENGINE  –  vectorised RPM sweep
%% ════════════════════════════════════════════════════════════════════════════
function C = phys(pl, bl, lt, ac, nite, bas, ebk)
    nr = numel(rpm_sw);
    Fh=zeros(np,nr); Tt=zeros(np,nr); Ts=zeros(np,nr);
    SF=zeros(np,nr); Pk=zeros(np,nr); vb=zeros(1,nr);

    for ri=1:nr
        rpm=rpm_sw(ri);
        v=max(pl(1).r/1000*rpm*2*pi/60, 0.01);
        vb(ri)=v; Tc=bl.lin_mass*v^2;

        for k=1:np
            Pk0=max(interp1(lt.rpm,lt.(pnames{k}),rpm,'linear','extrap'),0);
            if k==5 && ~ac,   Pk0=0;           end
            if k==4 && nite,  Pk0=max(Pk0,3.7);end
            if k==1 && bas,   Pk0=Pk0+5;       end  % +5kW BAS
            if k==1 && ebk,   Pk0=Pk0*0.7;     end  % engine braking
            Pk(k,ri)=Pk0;
            Te=Pk0*1000/v;
            Tt(k,ri)=bl.static_tension+Te/2+Tc;
            Ts(k,ri)=max(bl.static_tension-Te/2,0)+Tc;
            mu_th=bl.mu*wrap_deg(k)*pi/180;
            if Ts(k,ri)>0.1
                SF(k,ri)=min(log(Tt(k,ri)/Ts(k,ri))/mu_th, 9.99);
            else
                SF(k,ri)=9.99;
            end
            kp=mod(k-2,np)+1; kn=mod(k,np)+1;
            dxp=pl(kp).x-pl(k).x; dyp=pl(kp).y-pl(k).y;
            dxn=pl(kn).x-pl(k).x; dyn=pl(kn).y-pl(k).y;
            Lp=max(hypot(dxp,dyp),1); Ln=max(hypot(dxn,dyn),1);
            Fx=Tt(k,ri)*dxp/Lp+Ts(k,ri)*dxn/Ln;
            Fy=Tt(k,ri)*dyp/Lp+Ts(k,ri)*dyn/Ln;
            Fh(k,ri)=hypot(Fx,Fy);
        end
    end

    % Belt life (WLTC 4-phase)
    wR=[900 1200 1600 2000]; wW=[.25 .25 .25 .25];
    life=zeros(np,1);
    for k=1:np
        D=0;
        for wi=1:4
            vi=max(pl(1).r/1000*wR(wi)*2*pi/60,0.01);
            Pi=max(interp1(lt.rpm,lt.(pnames{k}),wR(wi),'linear','extrap'),0);
            Tti=bl.static_tension+Pi*1000/vi/2+bl.lin_mass*vi^2;
            Nfi=bl.wohler_Nref*(bl.wohler_Tref/max(Tti,1))^bl.wohler_m;
            D=D+wW(wi)/Nfi;
        end
        life(k)=min(bl.length_m/1000/max(D,1e-15),500000);
    end

    % 8-check validation at 1200 RPM
    cond_s=struct('ac',ac,'nightRun',nite,'bas',bas,'temp_C',S.temp_C);
    try
        [rpt,scr,sug]=FEAD_Validator(pl,bl,lt,cond_s,1200);
    catch
        rpt=struct('checks',{{}},'score',0,'n_pass',0,'n_total',8,...
            'F_hub',Fh(:,find(rpm_sw>=1199,1)),...
            'T_tight',Tt(:,find(rpm_sw>=1199,1)),...
            'T_slack',Ts(:,find(rpm_sw>=1199,1)),...
            'SF',SF(:,find(rpm_sw>=1199,1)),...
            'belt_life_km',life,...
            'suggestions',{{'Run FEAD_Validator for detailed checks.'}});
        scr=50; sug={'Validator unavailable - run FEAD_Validator manually.'};
    end

    C.Fh=Fh; C.Tt=Tt; C.Ts=Ts; C.SF=SF; C.Pk=Pk; C.vb=vb;
    C.life=life; C.score=scr; C.sug=sug; C.rpt=rpt;
end

%% ── Recompute & update all ───────────────────────────────────────────────────
function recompute()
    S.cache = phys(S.pulleys,S.belt,LT,S.ac_on,S.night,S.bas,S.eng_brk);
    update_plots();
    update_score_badge();
    update_badges();
    update_strip();
    update_table();
    if ~isempty(S.dw) && isvalid(S.dw.fig)
        rpt=S.cache.rpt; rpt.belt_life_km=S.cache.life;
        rpt.suggestions=S.cache.sug;
        FEAD_DataWindow_Update(S.dw,rpt,S.belt,S.rpm,WP,LT,S.pulleys);
    end
end

%% ── Plot updates ─────────────────────────────────────────────────────────────
function update_plots()
    C=S.cache;

    %% 1. Hub loads
    cla(S.ax_hub); hold(S.ax_hub,'on');
    for k=1:np
        plot(S.ax_hub,rpm_sw,C.Fh(k,:),'LineWidth',2,...
            'Color',PC{k},'DisplayName',pnames{k});
    end
    for k=1:np
        plot(S.ax_hub,1200,pdf_F(k),'o','MarkerSize',8,'MarkerFaceColor',PC{k},...
            'Color',PC{k},'HandleVisibility','off');
    end
    xl=xline(S.ax_hub,S.rpm,'--','Color',[1 1 1 0.25],'HandleVisibility','off');
    xl.LabelHorizontalAlignment='center';
    xlabel(S.ax_hub,'Engine RPM','Color',TC,'FontSize',9);
    ylabel(S.ax_hub,'Hub Load  F (N)','Color',TC,'FontSize',9);
    title(S.ax_hub,'Hub Loads vs RPM   (○ = Gates PDF reference @ 1200 RPM)',...
        'Color',AM,'FontWeight','bold','FontSize',10);
    legend(S.ax_hub,'TextColor',TC,'Color',[0.07 0.10 0.17],...
        'Location','northwest','FontSize',8);
    S.ax_hub.YLim(1)=0;

    %% 2. Tensions
    cla(S.ax_tens); hold(S.ax_tens,'on');
    for k=1:np
        plot(S.ax_tens,rpm_sw,C.Tt(k,:),'LineWidth',2,...
            'Color',PC{k},'DisplayName',[pnames{k},' T_tight']);
        plot(S.ax_tens,rpm_sw,C.Ts(k,:),'--','LineWidth',1.2,...
            'Color',PC{k}*0.55,'HandleVisibility','off');
    end
    yline(S.ax_tens,S.belt.T_max,'r-','LineWidth',2,...
        'Label',sprintf('T_{max}=%.0fN',S.belt.T_max),...
        'LabelHorizontalAlignment','left','HandleVisibility','off');
    yline(S.ax_tens,S.belt.static_tension,'Color',AM,'LineStyle',':',...
        'LineWidth',1.5,'Label','T₀','HandleVisibility','off');
    xlabel(S.ax_tens,'Engine RPM','Color',TC,'FontSize',9);
    ylabel(S.ax_tens,'Tension (N)','Color',TC,'FontSize',9);
    title(S.ax_tens,'Belt Tensions: solid=T_{tight}  dashed=T_{slack}',...
        'Color',AM,'FontWeight','bold','FontSize',10);
    legend(S.ax_tens,'TextColor',TC,'Color',[0.07 0.10 0.17],...
        'Location','northwest','FontSize',7,'NumColumns',2);
    S.ax_tens.YLim(1)=0;

    %% 3. Slip SF
    cla(S.ax_sf); hold(S.ax_sf,'on');
    for k=1:np
        sf_k=min(C.SF(k,:),6);
        plot(S.ax_sf,rpm_sw,sf_k,'LineWidth',2,...
            'Color',PC{k},'DisplayName',pnames{k});
    end
    yline(S.ax_sf,1.0,'r-','LineWidth',2,...
        'Label','SF=1.0 (SLIP)','HandleVisibility','off');
    yline(S.ax_sf,1.3,'Color',AM,'LineStyle','--','LineWidth',1.5,...
        'Label','SF=1.3 (min OK)','HandleVisibility','off');
    yline(S.ax_sf,2.0,'Color',GR,'LineStyle',':','LineWidth',1,...
        'Label','SF=2.0 (safe)','HandleVisibility','off');
    ylim(S.ax_sf,[0 6]);
    xlabel(S.ax_sf,'Engine RPM','Color',TC,'FontSize',9);
    ylabel(S.ax_sf,'Slip Safety Factor','Color',TC,'FontSize',9);
    title(S.ax_sf,'Capstan Slip SF vs RPM','Color',AM,'FontWeight','bold','FontSize',10);
    legend(S.ax_sf,'TextColor',TC,'Color',[0.07 0.10 0.17],...
        'Location','northeast','FontSize',8);

    %% 4. Belt Life
    cla(S.ax_life); hold(S.ax_life,'on');
    life_k=C.life/1e3;
    b=bar(S.ax_life,life_k,'FaceColor','flat','EdgeColor','none','BarWidth',0.65);
    for k=1:np, b.CData(k,:)=PC{k}; end
    yline(S.ax_life,200,'--','Color',AM,'LineWidth',2,...
        'Label','Min 200 000 km','LabelHorizontalAlignment','right',...
        'HandleVisibility','off');
    for k=1:np
        text(S.ax_life,k,life_k(k)+3,...
            sprintf('%.0f k',life_k(k)),'Color',PC{k},...
            'HorizontalAlignment','center','FontSize',8,'FontWeight','bold');
    end
    set(S.ax_life,'XTick',1:np,'XTickLabel',pnames,'XColor',TC);
    xlabel(S.ax_life,'Pulley','Color',TC,'FontSize',9);
    ylabel(S.ax_life,'Belt Life (×10³ km)','Color',TC,'FontSize',9);
    title(S.ax_life,sprintf('Belt Fatigue Life – %s  (WLTC weighted)',S.belt.name),...
        'Color',AM,'FontWeight','bold','FontSize',10);

    %% 5. Validation
    rpt=C.rpt;
    checks=rpt.checks;
    nc=numel(checks);
    cla(S.ax_val); hold(S.ax_val,'on');
    if nc>0
        for i=1:nc
            clr=iif(checks{i}.pass,GR,RD);
            barh(S.ax_val,i,100,'FaceColor',clr,'FaceAlpha',0.25,...
                'EdgeColor',clr,'LineWidth',1.5);
            sym=iif(checks{i}.pass,'✅','❌');
            txt=sprintf('%s  %s  →  %s',sym,checks{i}.label,checks{i}.msg);
            text(S.ax_val,2,i,txt,'Color',clr,...
                'FontSize',8.5,'VerticalAlignment','middle');
        end
        set(S.ax_val,'YTick',1:nc,...
            'YTickLabel',repmat({' '},1,nc),...
            'XLim',[0 115],'YLim',[0 nc+1]);
        title(S.ax_val,...
            sprintf('Validation Score:  %d / 100   (%d of %d checks passed)',...
            rpt.score,rpt.n_pass,nc),...
            'Color',AM,'FontWeight','bold','FontSize',10);
    end
    S.tx_sug.Value=C.sug;

    drawnow limitrate;
end

%% ── Score badge ─────────────────────────────────────────────────────────────
function update_score_badge()
    sc=S.cache.score;
    if sc>=80, c=GR; elseif sc>=60, c=AM; else, c=RD; end
    S.lbl_score.Text      = sprintf('Design Score:  %d / 100', sc);
    S.lbl_score.FontColor = c;
end

%% ── Per-pulley badge readouts ────────────────────────────────────────────────
function update_badges()
    if isempty(S.cache), return; end
    [~,ri]=min(abs(rpm_sw-S.rpm));
    for k=1:np
        sf=S.cache.SF(k,ri);
        sfclr=iif(sf>=1.3,'✅','⚠');
        S.badge{k}.Text=sprintf('%s\nF=%.0fN  SF=%.1f %s',...
            pnames{k},S.cache.Fh(k,ri),min(sf,9.99),sfclr);
    end
end

%% ── Top-strip summary ────────────────────────────────────────────────────────
function update_strip()
    if isempty(S.cache), return; end
    [~,ri]=min(abs(rpm_sw-S.rpm));
    v=S.cache.vb(ri);
    fmax=max(S.cache.Fh(:,ri));
    sfmin=min(S.cache.SF(:,ri));
    S.txt_strip.Text=sprintf(...
        'RPM=%d  |  v_belt=%.2fm/s  |  T₀=%.0fN  |  F_hub_max=%.0fN  |  SF_min=%.2f  |  Score=%d/100',...
        round(S.rpm),v,S.belt.static_tension,fmax,sfmin,S.cache.score);
end

%% ── Results table ────────────────────────────────────────────────────────────
function update_table()
    if isempty(S.cache), return; end
    [~,ri]=min(abs(rpm_sw-S.rpm));
    d=cell(np,8);
    for k=1:np
        sf=S.cache.SF(k,ri); lf=S.cache.life(k);
        pk=S.cache.Pk(k,ri);
        status=iif(sf>=1.3&&lf>=200000,'✅ OK','⚠ CHK');
        d{k,1}=pnames{k};
        d{k,2}=sprintf('%.2f',pk);
        d{k,3}=sprintf('%.0f',S.cache.Tt(k,ri));
        d{k,4}=sprintf('%.0f',S.cache.Ts(k,ri));
        d{k,5}=sprintf('%.0f',S.cache.Fh(k,ri));
        d{k,6}=sprintf('%.2f',min(sf,9.99));
        d{k,7}=sprintf('%.0f',lf);
        d{k,8}=status;
    end
    S.tbl.Data=d;
end

%% ════════════════════════════════════════════════════════════════════════════
%  ANIMATION ENGINE
%% ════════════════════════════════════════════════════════════════════════════
function gh = init_graphics(ax, pl)
    cla(ax); hold(ax,'on');
    th=linspace(0,2*pi,96); npl=numel(pl);
    gh.th=th; gh.h_disc=gobjects(npl,1); gh.h_hub=gobjects(npl,1);
    gh.h_ring=gobjects(npl,1); gh.h_spk=cell(npl,1);
    gh.h_span=gobjects(npl,1); gh.h_arr=gobjects(npl,1);
    gh.h_atxt=gobjects(npl,1); gh.h_lbl=gobjects(npl,1);
    gh.h_mrk=gobjects(16,1);

    for k=1:npl
        px=pl(k).x; py=pl(k).y; r=pl(k).r; c=PC{k};
        kn=mod(k,npl)+1;
        gh.h_span(k)=plot(ax,[px pl(kn).x],[py pl(kn).y],...
            'Color',[AM 0.45],'LineWidth',4.5);
        gh.h_disc(k)=patch(ax,px+r*cos(th),py+r*sin(th),c*0.15,...
            'EdgeColor',c,'LineWidth',2.2,'FaceAlpha',0.92);
        gh.h_ring(k)=patch(ax,px+(r+7)*cos(th),py+(r+7)*sin(th),'none',...
            'EdgeColor',GR,'LineWidth',3);
        gh.h_hub(k) =patch(ax,px+r*.18*cos(th),py+r*.18*sin(th),...
            [0.04 0.06 0.10],'EdgeColor',[0.45 0.55 0.65],'LineWidth',1.5);
        gh.h_spk{k} =gobjects(4,1);
        for s=1:4
            a=(s-1)*pi/2;
            gh.h_spk{k}(s)=plot(ax,...
                [px+r*.18*cos(a) px+r*.85*cos(a)],...
                [py+r*.18*sin(a) py+r*.85*sin(a)],...
                'Color',c,'LineWidth',2.5);
        end
        gh.h_lbl(k)=text(ax,px,py+r+18,pnames{k},...
            'HorizontalAlignment','center','Color',c,...
            'FontSize',9,'FontWeight','bold');
        gh.h_arr(k)=quiver(ax,px,py,0,0,'Color',RD,...
            'LineWidth',2,'MaxHeadSize',0.55,'AutoScale','off');
        gh.h_atxt(k)=text(ax,px+4,py+4,'',...
            'Color',[0.98 0.65 0.65],'FontSize',8,'FontWeight','bold');
    end
    for m=1:16
        gh.h_mrk(m)=plot(ax,0,0,'s','Color',[1.0 0.82 0.26],...
            'MarkerSize',4.5,'MarkerFaceColor',[1.0 0.82 0.26]);
    end
    gh.ov_rpm  =text(ax,-408,528,'RPM: —','Color',AM,'FontSize',11,'FontWeight','bold');
    gh.ov_vel  =text(ax,-408,500,'v_belt: —','Color',VI,'FontSize',10);
    gh.ov_t0   =text(ax,-408,474,'T₀: —','Color',[0.38 0.64 0.98],'FontSize',10);
    gh.ov_score=text(ax,-408,440,'Score: —','Color',GR,'FontSize',12,'FontWeight','bold');
    gh.mrk_pos =linspace(0,1,16);
end

function anim_step()
    if ~isvalid(fig)||~S.running, return; end
    dt=1/25; pl=S.pulleys; npl=numel(pl);
    v=pl(1).r/1000*S.rpm*2*pi/60;

    % Rotate spokes
    for k=1:npl
        rk=pl(k).r/1000; om=v/rk;
        if ~pl(k).cw, om=-om; end
        S.ang(k)=S.ang(k)+om*dt;
        px=pl(k).x; py=pl(k).y; r=pl(k).r;
        for s=1:4
            a=(s-1)*pi/2+S.ang(k);
            set(S.gh.h_spk{k}(s),...
                'XData',[px+r*.18*cos(a) px+r*.85*cos(a)],...
                'YData',[py+r*.18*sin(a) py+r*.85*sin(a)]);
        end
    end

    % Move belt markers
    sx=[pl.x pl(1).x]; sy=[pl.y pl(1).y];
    cum=zeros(1,npl+1);
    for k=1:npl, cum(k+1)=cum(k)+hypot(sx(k+1)-sx(k),sy(k+1)-sy(k)); end
    bsp=v*dt/max(cum(end),1);
    S.gh.mrk_pos=mod(S.gh.mrk_pos+bsp,1);
    for m=1:16
        pm=S.gh.mrk_pos(m)*cum(end);
        sg=find(cum<=pm,1,'last'); sg=min(sg,npl);
        fr=(pm-cum(sg))/max(cum(sg+1)-cum(sg),1);
        set(S.gh.h_mrk(m),'XData',sx(sg)*(1-fr)+sx(sg+1)*fr,...
            'YData',sy(sg)*(1-fr)+sy(sg+1)*fr);
    end

    % Update hub-load arrows + slip rings
    if ~isempty(S.cache)
        [~,ri]=min(abs(rpm_sw-S.rpm));
        for k=1:npl
            F=S.cache.Fh(k,ri); sf=S.cache.SF(k,ri);
            kp=mod(k-2,npl)+1; kn=mod(k,npl)+1;
            dxp=pl(kp).x-pl(k).x; dyp=pl(kp).y-pl(k).y;
            dxn=pl(kn).x-pl(k).x; dyn=pl(kn).y-pl(k).y;
            Lp=max(hypot(dxp,dyp),1); Ln=max(hypot(dxn,dyn),1);
            Tt=S.cache.Tt(k,ri); Ts=S.cache.Ts(k,ri);
            Fx=Tt*dxp/Lp+Ts*dxn/Ln; Fy=Tt*dyp/Lp+Ts*dyn/Ln;
            sc=min(F/5000,1)*70+8;
            if F>10
                ex=Fx/max(F,1)*sc; ey=Fy/max(F,1)*sc;
                set(S.gh.h_arr(k),'XData',pl(k).x,'YData',pl(k).y,...
                    'UData',ex,'VData',ey);
                set(S.gh.h_atxt(k),'Position',...
                    [pl(k).x+ex+5,pl(k).y+ey+5],...
                    'String',sprintf('%.0fN',F));
            end
            rc=iif(sf>=1.3,GR,iif(sf>=1.0,AM,RD));
            S.gh.h_ring(k).EdgeColor=rc;
        end
        S.gh.ov_rpm.String  = sprintf('Engine: %d RPM',round(S.rpm));
        S.gh.ov_vel.String  = sprintf('v_{belt}: %.2f m/s',v);
        S.gh.ov_t0.String   = sprintf('T₀: %d N',round(S.belt.static_tension));
        sc2=S.cache.score;
        S.gh.ov_score.String= sprintf('Score: %d / 100',sc2);
        S.gh.ov_score.Color = iif(sc2>=80,GR,iif(sc2>=60,AM,RD));
    end
    drawnow limitrate;
end

function update_layout_graphics()
    pl=S.pulleys; npl=numel(pl); th=S.gh.th;
    for k=1:npl
        px=pl(k).x; py=pl(k).y; r=pl(k).r;
        kn=mod(k,npl)+1;
        S.gh.h_disc(k).XData=px+r*cos(th); S.gh.h_disc(k).YData=py+r*sin(th);
        S.gh.h_ring(k).XData=px+(r+7)*cos(th); S.gh.h_ring(k).YData=py+(r+7)*sin(th);
        S.gh.h_hub(k).XData =px+r*.18*cos(th); S.gh.h_hub(k).YData =py+r*.18*sin(th);
        S.gh.h_lbl(k).Position(1:2)=[px, py+r+18];
        S.gh.h_span(k).XData=[px pl(kn).x]; S.gh.h_span(k).YData=[py pl(kn).y];
    end
end

%% ════════════════════════════════════════════════════════════════════════════
%  CALLBACKS
%% ════════════════════════════════════════════════════════════════════════════
function cb_anim(~,~)
    S.running=~S.running;
    if S.running
        S.btn_anim.Text='⏹  Stop Animation';
        S.btn_anim.BackgroundColor=[0.22 0.06 0.06];
        if strcmp(S.tmr.Running,'off'), start(S.tmr); end
    else
        S.btn_anim.Text='▶  Start Animation';
        S.btn_anim.BackgroundColor=[0.08 0.20 0.12];
    end
end

function cb_validate(~,~)
    recompute();
    sc=S.cache.score;
    uialert(fig,...
        sprintf(['Score: %d/100\n%d of 8 checks passed\n\n'...
        'Min SF = %.2f\nMax F_hub = %.0f N\nBelt life = %.0f km\n\n%s'],...
        sc,S.cache.rpt.n_pass,...
        min(S.cache.SF(:,find(rpm_sw>=1199,1))),...
        max(S.cache.Fh(:,find(rpm_sw>=1199,1))),...
        min(S.cache.life),...
        strjoin(S.cache.sug,newline)),...
        'Design Validation','Icon',iif(sc>=80,'success',iif(sc>=60,'warning','error')));
end

function cb_optimise(~,~)
    try
        [~,~,~,opt]=FEAD_Validator(S.pulleys,S.belt,LT,...
            struct('ac',S.ac_on,'nightRun',S.night,'bas',S.bas,'temp_C',S.temp_C),1200);
        moved=false;
        if opt(6).x~=S.pulleys(6).x||opt(6).y~=S.pulleys(6).y
            S.pulleys(6).x=opt(6).x; S.pulleys(6).y=opt(6).y;
            S.ef_x{6}.Value=opt(6).x; S.ef_y{6}.Value=opt(6).y;
            moved=true;
        end
        update_layout_graphics(); recompute();
        if moved
            uialert(fig,sprintf('Tensioner moved to X=%.0f, Y=%.0f mm\nTotal hub load reduced.',...
                opt(6).x,opt(6).y),'Optimised','Icon','success');
        else
            uialert(fig,'Tensioner already at optimal position.','Optimise','Icon','info');
        end
    catch e
        uialert(fig,['Optimise failed: ' e.message],'Error','Icon','error');
    end
end

function cb_dw(~,~)
    if isempty(S.dw)||~isvalid(S.dw.fig), S.dw=FEAD_DataWindow(); end
    figure(S.dw.fig);
    rpt=S.cache.rpt; rpt.belt_life_km=S.cache.life; rpt.suggestions=S.cache.sug;
    FEAD_DataWindow_Update(S.dw,rpt,S.belt,S.rpm,WP,LT,S.pulleys);
end

function cb_simscape(~,~)
    assignin('base','pulleys',S.pulleys);
    assignin('base','belt',S.belt);
    try
        FEAD_TestRig_Builder(S.pulleys,S.belt,LT,...
            struct('ac',S.ac_on,'nightRun',S.night,'bas',S.bas,'temp_C',S.temp_C));
        uialert(fig,'FEAD_TestRig.slx built. Open in Simulink.','Simscape','Icon','success');
    catch e
        uialert(fig,['Build error: ' e.message],'Simscape','Icon','error');
    end
end

function cb_github(~,~)
    rpt=S.cache.rpt; rpt.belt_life_km=S.cache.life; rpt.suggestions=S.cache.sug;
    try
        push_to_github(S.pulleys,S.belt,rpt);
        uialert(fig,'Pushed to GitHub successfully.','GitHub','Icon','success');
    catch e
        uialert(fig,['GitHub push failed: ' e.message],'GitHub','Icon','error');
    end
end

function cb_reset(~,~)
    FEAD_params;
    S.pulleys=evalin('base','pulleys');
    S.belt.static_tension=evalin('base','belt.static_tension');
    for k=1:np
        S.ef_x{k}.Value=S.pulleys(k).x;
        S.ef_y{k}.Value=S.pulleys(k).y;
        S.ef_r{k}.Value=S.pulleys(k).r;
    end
    S.sl_ten.Value=S.belt.static_tension;
    S.lbl_ten.Text=sprintf('%d N',round(S.belt.static_tension));
    update_layout_graphics(); recompute();
end

function cb_import(~,~)
    [f,p]=uigetfile('*.json','Select web-tool JSON');
    if isnumeric(f), return; end
    try
        d=jsondecode(fileread(fullfile(p,f)));
        fn=fieldnames(d.pulleys);
        for ki=1:min(numel(fn),np)
            idx=find(strcmp(pnames,fn{ki}),1); if isempty(idx),continue;end
            pf=d.pulleys.(fn{ki});
            if isfield(pf,'x'),S.pulleys(idx).x=pf.x;S.ef_x{idx}.Value=pf.x;end
            if isfield(pf,'y'),S.pulleys(idx).y=pf.y;S.ef_y{idx}.Value=pf.y;end
            if isfield(pf,'r'),S.pulleys(idx).r=pf.r;S.ef_r{idx}.Value=pf.r;end
        end
        if isfield(d,'rpm'), S.rpm=d.rpm; S.sl_rpm.Value=S.rpm; end
        if isfield(d,'staticTension')
            S.belt.static_tension=d.staticTension;
            S.sl_ten.Value=d.staticTension;
        end
        update_layout_graphics(); recompute();
        uialert(fig,'Layout imported.','Import','Icon','success');
    catch e
        uialert(fig,['Import failed: ' e.message],'Import','Icon','error');
    end
end

function cb_belt(src,~)
    idx=find(strcmp({BLIB.name},src.Value),1);
    if isempty(idx),return;end
    prev_t0=S.belt.static_tension;
    S.belt=BLIB(idx);
    S.belt.static_tension=prev_t0;
    S.belt.length_m=S.belt.length_mm/1000;
    S.lbl_belt.Text=beltstr(S.belt);
    recompute();
end

function cb_rpm(src,~)
    S.rpm=round(src.Value/50)*50;
    S.sl_rpm.Value=S.rpm;
    S.lbl_rpm.Text=sprintf('%d RPM',S.rpm);
    update_badges(); update_strip(); update_table();
    drawnow limitrate;
end

function cb_tension(src,~)
    S.belt.static_tension=round(src.Value/10)*10;
    S.lbl_ten.Text=sprintf('%d N',S.belt.static_tension);
    recompute();
end

function cb_tenpos(~,evt)
    lbl_sel=evt.NewValue.Text;
    idx=find(strcmp({TPOS.label},lbl_sel),1);
    if isempty(idx),return;end
    S.ten_idx=idx;
    S.pulleys(6).x=TPOS(idx).x; S.pulleys(6).y=TPOS(idx).y;
    S.belt.static_tension=TPOS(idx).T;
    S.ef_x{6}.Value=TPOS(idx).x; S.ef_y{6}.Value=TPOS(idx).y;
    S.sl_ten.Value=TPOS(idx).T;
    S.lbl_ten.Text=sprintf('%d N',round(TPOS(idx).T));
    S.lbl_ten2.Text=sprintf('T₀ = %.0f N  (%s position, arm=%.1fmm)',...
        TPOS(idx).T, TPOS(idx).label, TPOS(idx).arm);
    update_layout_graphics(); recompute();
end

function cb_cond(field,val)
    S.(field)=val; recompute();
end

function cb_pedit(k,field,val)
    S.pulleys(k).(field)=val;
    update_layout_graphics(); recompute();
end

function mdown(~,~)
    cp=S.ax.CurrentPoint; if isempty(cp),return;end
    for k=1:np
        if hypot(cp(1,1)-S.pulleys(k).x,cp(1,2)-S.pulleys(k).y)<S.pulleys(k).r*1.5
            S.dragging=k; return;
        end
    end
end

function mmove(~,~)
    if S.dragging<1,return;end
    cp=S.ax.CurrentPoint; if isempty(cp),return;end
    k=S.dragging;
    S.pulleys(k).x=round(cp(1,1)); S.pulleys(k).y=round(cp(1,2));
    S.ef_x{k}.Value=S.pulleys(k).x; S.ef_y{k}.Value=S.pulleys(k).y;
    update_layout_graphics(); drawnow limitrate;
end

function mup(~,~)
    if S.dragging>0, S.dragging=-1; recompute(); end
end

function cb_close(~,~)
    if strcmp(S.tmr.Running,'on'),stop(S.tmr);end
    delete(S.tmr);
    if ~isempty(S.dw)&&isvalid(S.dw.fig),delete(S.dw.fig);end
    delete(fig);
end

end % FEAD_App

%% ── Module-level helpers (local functions, not nested) ───────────────────────
function s=beltstr(b)
    s=sprintf('L=%.0fmm | μ=%.2f | %d-rib | %s | T_max=%.0fN',...
        b.length_mm,b.mu,b.ribs,b.core,b.T_max);
end

function out=iif(cond,a,b)
    if cond,out=a;else,out=b;end
end
