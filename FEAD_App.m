%% FEAD_App.m  –  Complete FEAD Belt Drive Test Rig App
%  Full-featured MATLAB uifigure application with:
%   • Real-time animation (rotating pulleys + moving belt)
%   • Editable layout datum (drag pulleys OR numeric inputs)
%   • Belt selection from full catalog
%   • Operating condition checkboxes (AC on/off, Night Run, BAS, etc.)
%   • Tensioner position dropdown (FREE→LOAD)
%   • RPM + static tension sliders
%   • Live validation with design score
%   • Design suggestions & layout optimizer
%   • Separate data window (all results)
%   • Build Simscape model button
%   • Export to GitHub button
%   • Import from web tool (loads JSON from web interface)
%
%  Usage:  >> FEAD_params; FEAD_App
% ─────────────────────────────────────────────────────────────────────────────

function FEAD_App()

if ~exist('pulleys','var') || ~exist('belt','var')
    FEAD_params;
end

% Load belt library
belt_lib = FEAD_BeltLibrary();
belt_idx = 1;  % default = Gates MT620 AMD 8-rib

% Current state
S.pulleys    = evalin('base','pulleys');
S.belt       = belt_lib(belt_idx);
% Copy belt fields from params to lib struct
S.belt.static_tension = evalin('base','belt.static_tension');
S.belt.length_m       = evalin('base','belt.length_m');
S.load_table  = evalin('base','load_table');
S.wp          = evalin('base','wp');
S.conditions  = struct('ac',true,'nightRun',false,'bas',false,'temp_C',40);
S.rpm         = 1200;
S.ten_idx     = 4;  % MEAN
S.ten_pos     = evalin('base','ten_pos');
S.dragging    = -1;
S.anim_running= false;
S.dw          = [];   % data window handle

pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
np     = 6;

%% ── COLOURS ────────────────────────────────────────────────────────────────
BG  = [0.06 0.08 0.14];
AX  = [0.09 0.12 0.20];
TC  = [0.89 0.91 0.94];
AM  = [0.96 0.62 0.04];
VI  = [0.65 0.54 0.98];
GR  = [0.20 0.85 0.60];
RD  = [0.94 0.27 0.27];
pcolors = {AM,[0.55 0.36 0.96],VI,GR,[0.96 0.28 0.71],[0.38 0.64 0.98]};

%% ── MAIN FIGURE ─────────────────────────────────────────────────────────────
fig = uifigure('Name','FEAD Belt Drive Test Rig  –  Ashok Leyland H6',...
    'Position',[20 30 1840 980],...
    'Color',BG,'Resize','on',...
    'CloseRequestFcn',@on_close);

%% ═══ LEFT PANEL: Controls (width 330) ═══════════════════════════════════════
left = uipanel(fig,'Position',[5 5 330 970],...
    'BackgroundColor',AX,'BorderType','none');

yc = 940;  % cursor y position

% Header
uilabel(left,'Text','FEAD Test Rig Controls',...
    'Position',[5 yc 320 26],'FontSize',13,'FontWeight','bold',...
    'FontColor',AM,'BackgroundColor',AX,'HorizontalAlignment','center');
yc=yc-30;

% ── Belt Selection ───────────────────────────────────────────────────────────
uilabel(left,'Text','── Belt Selection ──',...
    'Position',[5 yc 320 20],'FontSize',10,'FontColor',VI,'BackgroundColor',AX);
yc=yc-25;
belt_names = arrayfun(@(b) b.name, belt_lib,'UniformOutput',false);
S.dd_belt = uidropdown(left,'Position',[5 yc 320 28],...
    'Items',belt_names,'Value',belt_names{1},...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(src,~) on_belt_change(src.Value));
yc=yc-34;

% Belt info label
S.lbl_belt_info = uilabel(left,'Position',[5 yc 320 18],...
    'Text',sprintf('L=%.0fmm  μ=%.2f  %drib  Aramid', ...
    S.belt.length_mm, S.belt.mu, S.belt.ribs),...
    'FontSize',9,'FontColor',[0.55 0.62 0.72],'BackgroundColor',AX);
yc=yc-28;

% ── Engine RPM ───────────────────────────────────────────────────────────────
uilabel(left,'Text','── Engine Speed ──',...
    'Position',[5 yc 320 20],'FontSize',10,'FontColor',VI,'BackgroundColor',AX);
yc=yc-25;
S.lbl_rpm = uilabel(left,'Position',[220 yc 100 22],...
    'Text','1200 RPM','FontColor',AM,'FontWeight','bold','BackgroundColor',AX);
S.sl_rpm = uislider(left,'Position',[5 yc+8 210 3],...
    'Limits',[400 2500],'Value',1200,...
    'MajorTicks',[500 1000 1500 2000 2500],...
    'ValueChangedFcn',@(src,~) on_rpm_change(src.Value));
yc=yc-38;

% ── Static Tension ───────────────────────────────────────────────────────────
uilabel(left,'Text','── Static Tension (N) ──',...
    'Position',[5 yc 320 20],'FontSize',10,'FontColor',VI,'BackgroundColor',AX);
yc=yc-25;
S.lbl_ten = uilabel(left,'Position',[220 yc 100 22],...
    'Text','480 N','FontColor',AM,'FontWeight','bold','BackgroundColor',AX);
S.sl_ten = uislider(left,'Position',[5 yc+8 210 3],...
    'Limits',[100 1000],'Value',480,...
    'MajorTicks',[200 400 600 800 1000],...
    'ValueChangedFcn',@(src,~) on_tension_change(src.Value));
yc=yc-38;

% ── Tensioner Position ───────────────────────────────────────────────────────
uilabel(left,'Text','── Tensioner Position ──',...
    'Position',[5 yc 320 20],'FontSize',10,'FontColor',VI,'BackgroundColor',AX);
yc=yc-25;
ten_labels = arrayfun(@(t) t.label, S.ten_pos,'UniformOutput',false);
S.dd_ten = uidropdown(left,'Position',[5 yc 320 28],...
    'Items',ten_labels,'Value','MEAN',...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(src,~) on_ten_pos(src.Value));
yc=yc-38;

% ── Operating Conditions ─────────────────────────────────────────────────────
uilabel(left,'Text','── Operating Conditions ──',...
    'Position',[5 yc 320 20],'FontSize',10,'FontColor',VI,'BackgroundColor',AX);
yc=yc-28;

S.cb_ac = uicheckbox(left,'Text','AC Compressor ON','Value',1,...
    'Position',[10 yc 200 22],'FontColor',TC,'BackgroundColor',AX,...
    'ValueChangedFcn',@(src,~) on_cond_change('ac',src.Value));
yc=yc-26;
S.cb_night = uicheckbox(left,'Text','Night Run (Alt max load)','Value',0,...
    'Position',[10 yc 220 22],'FontColor',TC,'BackgroundColor',AX,...
    'ValueChangedFcn',@(src,~) on_cond_change('nightRun',src.Value));
yc=yc-26;
S.cb_bas = uicheckbox(left,'Text','BAS (Belt-Alt-Starter)','Value',0,...
    'Position',[10 yc 220 22],'FontColor',TC,'BackgroundColor',AX,...
    'ValueChangedFcn',@(src,~) on_cond_change('bas',src.Value));
yc=yc-30;

% Temperature input
uilabel(left,'Text','Operating Temp (°C):',...
    'Position',[10 yc 160 22],'FontColor',TC,'BackgroundColor',AX);
S.ef_temp = uieditfield(left,'numeric','Value',40,'Limits',[-40 150],...
    'Position',[175 yc 80 22],...
    'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
    'ValueChangedFcn',@(src,~) on_cond_change('temp_C',src.Value));
yc=yc-38;

% ── Pulley Layout Table ───────────────────────────────────────────────────────
uilabel(left,'Text','── Pulley Layout Datum (edit to move) ──',...
    'Position',[5 yc 320 20],'FontSize',10,'FontColor',VI,'BackgroundColor',AX);
yc=yc-22;
uilabel(left,'Text','  Pulley   X(mm)  Y(mm)  R(mm)',...
    'Position',[5 yc 320 18],'FontSize',9,'FontColor',[0.45 0.55 0.65],'BackgroundColor',AX);
yc=yc-22;

S.ef_px = cell(np,1); S.ef_py = cell(np,1); S.ef_pr = cell(np,1);
for k = 1:np
    c = pcolors{k};
    uilabel(left,'Text',pnames{k},...
        'Position',[5 yc 40 22],'FontColor',c,'FontWeight','bold','BackgroundColor',AX);
    S.ef_px{k} = uieditfield(left,'numeric','Value',S.pulleys(k).x,...
        'Limits',[-600 600],'Position',[48 yc 74 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(src,~) on_pulley_edit(k,'x',src.Value));
    S.ef_py{k} = uieditfield(left,'numeric','Value',S.pulleys(k).y,...
        'Limits',[-100 700],'Position',[126 yc 74 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(src,~) on_pulley_edit(k,'y',src.Value));
    S.ef_pr{k} = uieditfield(left,'numeric','Value',S.pulleys(k).r,...
        'Limits',[10 150],'Position',[204 yc 74 22],...
        'BackgroundColor',[0.07 0.10 0.18],'FontColor',TC,...
        'ValueChangedFcn',@(src,~) on_pulley_edit(k,'r',src.Value));
    yc = yc - 25;
end

yc = yc - 8;

% ── Control Buttons ──────────────────────────────────────────────────────────
btn_w=155; btn_h=30; btn_gap=6;

S.btn_anim = uibutton(left,'push','Text','▶ Start Animation',...
    'Position',[5 yc btn_w btn_h],...
    'BackgroundColor',[0.08 0.20 0.12],'FontColor',GR,'FontWeight','bold',...
    'ButtonPushedFcn',@(~,~) toggle_animation());

S.btn_validate = uibutton(left,'push','Text','✔ Validate Design',...
    'Position',[165 yc btn_w btn_h],...
    'BackgroundColor',[0.08 0.12 0.24],'FontColor',VI,'FontWeight','bold',...
    'ButtonPushedFcn',@(~,~) run_validation());
yc=yc-btn_h-btn_gap;

S.btn_optimize = uibutton(left,'push','Text','⚙ Optimize Layout',...
    'Position',[5 yc btn_w btn_h],...
    'BackgroundColor',[0.12 0.08 0.24],'FontColor',VI,...
    'ButtonPushedFcn',@(~,~) run_optimization());

S.btn_datawin = uibutton(left,'push','Text','📊 Data Window',...
    'Position',[165 yc btn_w btn_h],...
    'BackgroundColor',[0.10 0.15 0.28],'FontColor',[0.38 0.64 0.98],...
    'ButtonPushedFcn',@(~,~) open_data_window());
yc=yc-btn_h-btn_gap;

S.btn_simscape = uibutton(left,'push','Text','🔧 Build Simscape',...
    'Position',[5 yc btn_w btn_h],...
    'BackgroundColor',[0.16 0.10 0.06],'FontColor',AM,...
    'ButtonPushedFcn',@(~,~) build_simscape());

S.btn_github = uibutton(left,'push','Text','⬆ Push to GitHub',...
    'Position',[165 yc btn_w btn_h],...
    'BackgroundColor',[0.06 0.14 0.10],'FontColor',GR,...
    'ButtonPushedFcn',@(~,~) push_github());
yc=yc-btn_h-btn_gap;

S.btn_reset = uibutton(left,'push','Text','↺ Reset to PDF Datum',...
    'Position',[5 yc btn_w btn_h],...
    'BackgroundColor',AX,'FontColor',TC,...
    'ButtonPushedFcn',@(~,~) reset_datum());

S.btn_import = uibutton(left,'push','Text','⬇ Import from Web',...
    'Position',[165 yc btn_w btn_h],...
    'BackgroundColor',AX,'FontColor',TC,...
    'ButtonPushedFcn',@(~,~) import_from_web());
yc=yc-btn_h-btn_gap;

% Design score badge
S.lbl_score = uilabel(left,'Position',[5 yc 320 34],...
    'Text','Design Score:  —',...
    'FontSize',14,'FontWeight','bold','FontColor',GR,...
    'HorizontalAlignment','center','BackgroundColor',AX);

%% ═══ CENTRE PANEL: Animation Canvas ═════════════════════════════════════════
centre = uipanel(fig,'Position',[340 5 840 970],...
    'BackgroundColor',BG,'BorderType','none');

ax_anim = uiaxes(centre,'Position',[5 150 830 810]);
ax_anim.Color     = [0.05 0.08 0.12];
ax_anim.XColor    = [0.3 0.4 0.5];
ax_anim.YColor    = [0.3 0.4 0.5];

% Quick-result badges below animation
y_badge = 5;
badge_lbl = {'CRK F_hub','FAN F_hub','IDR F_hub','ALT F_hub','AC F_hub','TEN F_hub'};
S.lbl_badges = cell(np,1);
for k = 1:np
    c = pcolors{k};
    S.lbl_badges{k} = uilabel(centre,...
        'Position',[(k-1)*138+5 y_badge 133 32],...
        'Text',sprintf('%s\n— N',pnames{k}),...
        'FontSize',9,'FontWeight','bold','FontColor',c,...
        'BackgroundColor',[0.08 0.12 0.20],'HorizontalAlignment','center');
end

%% ═══ RIGHT PANEL: Results Plots ══════════════════════════════════════════════
right = uipanel(fig,'Position',[1185 5 650 970],...
    'BackgroundColor',BG,'BorderType','none');

uilabel(right,'Text','Live Results & Validation',...
    'Position',[5 940 640 24],'FontSize',13,'FontWeight','bold',...
    'FontColor',AM,'HorizontalAlignment','center','BackgroundColor',BG);

% Tab group for right panel plots
tg_r = uitabgroup(right,'Position',[5 5 640 930]);
tab_hub   = uitab(tg_r,'Title','Hub Loads', 'BackgroundColor',AX);
tab_tens  = uitab(tg_r,'Title','Tensions',  'BackgroundColor',AX);
tab_sf    = uitab(tg_r,'Title','Slip SF',   'BackgroundColor',AX);
tab_life  = uitab(tg_r,'Title','Belt Life', 'BackgroundColor',AX);
tab_valid = uitab(tg_r,'Title','Validation','BackgroundColor',AX);

S.ax_hub  = uiaxes(tab_hub, 'Position',[5 5 625 870]);
S.ax_tens = uiaxes(tab_tens,'Position',[5 5 625 870]);
S.ax_sf   = uiaxes(tab_sf,  'Position',[5 5 625 870]);
S.ax_life = uiaxes(tab_life,'Position',[5 5 625 870]);
S.ax_val  = uiaxes(tab_valid,'Position',[5 180 625 700]);
S.txt_val = uitextarea(tab_valid,'Position',[5 5 625 170],...
    'Editable','off','BackgroundColor',[0.07 0.10 0.17],'FontColor',TC,'FontSize',10);

for a = {S.ax_hub, S.ax_tens, S.ax_sf, S.ax_life, S.ax_val}
    ax_i = a{1};
    ax_i.Color = [0.07 0.10 0.17];
    ax_i.XColor = TC; ax_i.YColor = TC;
    ax_i.GridColor = [0.15 0.22 0.32]; ax_i.GridAlpha = 0.6;
    ax_i.XGrid = 'on'; ax_i.YGrid = 'on';
end

%% ── Initialise animation ─────────────────────────────────────────────────────
S.anim = FEAD_Animation(ax_anim, S.pulleys, S.belt, S.load_table);

%% ── Set up animation timer ───────────────────────────────────────────────────
S.timer = timer('Name','FEADTimer','Period',1/25,...
    'ExecutionMode','fixedRate',...
    'TimerFcn',@(~,~) timer_step());

%% ── Mouse callbacks on animation axes ───────────────────────────────────────
fig.WindowButtonDownFcn   = @on_mouse_down;
fig.WindowButtonMotionFcn = @on_mouse_move;
fig.WindowButtonUpFcn     = @on_mouse_up;

%% ── Initial compute & plot ───────────────────────────────────────────────────
recompute_and_plot();

fprintf('FEAD App launched. Use controls on left panel.\n');

%% ═══════════════════════════════════════════════════════════════════════════
%  NESTED CALLBACK FUNCTIONS
%  All access S via closure — no guidata needed.
% ═══════════════════════════════════════════════════════════════════════════

    function recompute_and_plot()
        % Run validation
        [report, score, suggestions, opt_layout] = ...
            FEAD_Validator(S.pulleys, S.belt, S.load_table, S.conditions, S.rpm);
        report.suggestions = suggestions;
        S.last_report = report;
        S.last_opt    = opt_layout;

        % Update score badge
        if score >= 80,    sc=[0.20 0.85 0.60];
        elseif score >= 60,sc=[0.96 0.62 0.04];
        else,              sc=[0.94 0.27 0.27]; end
        S.lbl_score.Text       = sprintf('Design Score:  %d / 100', score);
        S.lbl_score.FontColor  = sc;

        % Update hub load badges
        for k2 = 1:np
            S.lbl_badges{k2}.Text = sprintf('%s\n%.0f N', pnames{k2}, report.F_hub(k2));
        end

        % Update plots
        update_hub_plot(report);
        update_tens_plot(report);
        update_sf_plot(report);
        update_life_plot(report);
        update_val_plot(report, suggestions);

        % Push to data window if open
        if ~isempty(S.dw) && isvalid(S.dw.fig)
            FEAD_DataWindow_Update(S.dw, report, S.belt, S.rpm, S.wp, S.load_table, S.pulleys);
        end
    end

    function update_hub_plot(report)
        rpm_sw = 300:50:2500;
        F = zeros(np, numel(rpm_sw));
        for ri = 1:numel(rpm_sw)
            [r2,~,~,~] = FEAD_Validator(S.pulleys, S.belt, S.load_table, S.conditions, rpm_sw(ri));
            F(:,ri) = r2.F_hub(:);
        end
        cla(S.ax_hub); hold(S.ax_hub,'on');
        for k = 1:np
            plot(S.ax_hub, rpm_sw, F(k,:),'LineWidth',2,'Color',pcolors{k},'DisplayName',pnames{k});
        end
        pdf_F = [2658.9,2866.4,1710.1,1678.1,985.8,608.5];
        for k = 1:np
            plot(S.ax_hub,1200,pdf_F(k),'o','MarkerSize',7,'Color',pcolors{k},'MarkerFaceColor',pcolors{k});
        end
        xline(S.ax_hub, S.rpm,'--','Color',[1 1 1 0.3]);
        xlabel(S.ax_hub,'RPM','Color',TC); ylabel(S.ax_hub,'F (N)','Color',TC);
        title(S.ax_hub,'Hub Loads vs RPM','Color',AM,'FontWeight','bold');
        legend(S.ax_hub,'TextColor',TC,'Color',[0.07 0.10 0.17],'Location','northwest');
    end

    function update_tens_plot(report)
        cla(S.ax_tens); hold(S.ax_tens,'on');
        rpm_sw = 300:50:2500;
        for k = 1:np
            Tt = zeros(1,numel(rpm_sw)); Ts = zeros(1,numel(rpm_sw));
            for ri = 1:numel(rpm_sw)
                [r2] = FEAD_Validator(S.pulleys, S.belt, S.load_table, S.conditions, rpm_sw(ri));
                Tt(ri) = r2.T_tight(k); Ts(ri) = r2.T_slack(k);
            end
            plot(S.ax_tens,rpm_sw,Tt,'LineWidth',2,'Color',pcolors{k},'DisplayName',[pnames{k},' tight']);
            plot(S.ax_tens,rpm_sw,Ts,'--','LineWidth',1,'Color',pcolors{k}*0.6);
        end
        yline(S.ax_tens,S.belt.T_max,'r--','LineWidth',2,'Label','T_{max}');
        xlabel(S.ax_tens,'RPM','Color',TC); ylabel(S.ax_tens,'Tension (N)','Color',TC);
        title(S.ax_tens,'Belt Tensions (solid=tight, dashed=slack)','Color',AM,'FontWeight','bold');
        legend(S.ax_tens,'TextColor',TC,'Color',[0.07 0.10 0.17],'Location','northwest');
    end

    function update_sf_plot(report)
        cla(S.ax_sf); hold(S.ax_sf,'on');
        rpm_sw = 300:50:2500;
        for k = 1:np
            SF_v = zeros(1,numel(rpm_sw));
            for ri = 1:numel(rpm_sw)
                [r2] = FEAD_Validator(S.pulleys, S.belt, S.load_table, S.conditions, rpm_sw(ri));
                SF_v(ri) = min(r2.SF(k),6);
            end
            plot(S.ax_sf,rpm_sw,SF_v,'LineWidth',2,'Color',pcolors{k},'DisplayName',pnames{k});
        end
        yline(S.ax_sf,1.0,'r--','LineWidth',2,'Label','SF=1 (slip)');
        yline(S.ax_sf,1.3,'Color',AM,'LineStyle','--','LineWidth',1.5,'Label','SF=1.3');
        ylim(S.ax_sf,[0 6]);
        xlabel(S.ax_sf,'RPM','Color',TC); ylabel(S.ax_sf,'Slip SF','Color',TC);
        title(S.ax_sf,'Capstan Slip Safety Factor vs RPM','Color',AM,'FontWeight','bold');
        legend(S.ax_sf,'TextColor',TC,'Color',[0.07 0.10 0.17],'Location','northeast');
    end

    function update_life_plot(report)
        cla(S.ax_life); hold(S.ax_life,'on');
        bar_data = report.belt_life_km / 1000;
        b = bar(S.ax_life, bar_data,'FaceColor','flat');
        for k = 1:np
            b.CData(k,:) = pcolors{k};
        end
        yline(S.ax_life,200,'--','Color',AM,'LineWidth',1.5,'Label','Min 200k km');
        set(S.ax_life,'XTickLabel',pnames);
        xlabel(S.ax_life,'Pulley','Color',TC);
        ylabel(S.ax_life,'Belt Life (×10³ km)','Color',TC);
        title(S.ax_life,'Belt Fatigue Life (WLTC Weighted)','Color',AM,'FontWeight','bold');
    end

    function update_val_plot(report, suggestions)
        checks  = report.checks;
        nc      = numel(checks);
        cla(S.ax_val); hold(S.ax_val,'on');
        labels  = cellfun(@(c)c.label, checks,'UniformOutput',false);
        scores_v= cellfun(@(c) double(c.pass)*100, checks);
        barh(S.ax_val, 1:nc, scores_v,'FaceColor','flat',...
            'FaceAlpha',0.8);
        S.ax_val.Colormap = [RD; AM; GR];
        for i = 1:nc
            clr = ternary(checks{i}.pass, GR, RD);
            text(S.ax_val, 5, i, checks{i}.msg,...
                'Color',clr,'FontSize',9,'VerticalAlignment','middle');
        end
        set(S.ax_val,'YTick',1:nc,'YTickLabel',labels,'YColor',TC,...
            'XLim',[0 110],'XColor',TC);
        title(S.ax_val,sprintf('Validation — %d/%d Passed  (Score: %d/100)',...
            report.n_pass, report.n_total, report.score),'Color',AM,'FontWeight','bold');
        S.txt_val.Value = suggestions;
    end

    function timer_step()
        if isvalid(fig) && S.anim_running
            FEAD_Animation_Step(S.anim, S.rpm, S.belt.static_tension, S.last_report);
        end
    end

    function toggle_animation()
        if S.anim_running
            stop(S.timer);
            S.anim_running = false;
            S.btn_anim.Text = '▶ Start Animation';
            S.btn_anim.BackgroundColor = [0.08 0.20 0.12];
        else
            S.anim_running = true;
            S.btn_anim.Text = '⏹ Stop Animation';
            S.btn_anim.BackgroundColor = [0.24 0.06 0.06];
            if strcmp(S.timer.Running,'off'), start(S.timer); end
        end
    end

    function run_validation()
        recompute_and_plot();
        uialert(fig, sprintf('Design Score: %d/100\n%d of %d checks passed.',...
            S.last_report.score, S.last_report.n_pass, S.last_report.n_total),...
            'Validation Complete','Icon','info');
    end

    function run_optimization()
        opt = S.last_opt;
        % Apply optimal tensioner position
        if opt(6).x ~= S.pulleys(6).x || opt(6).y ~= S.pulleys(6).y
            S.pulleys(6).x = opt(6).x;
            S.pulleys(6).y = opt(6).y;
            S.ef_px{6}.Value = opt(6).x;
            S.ef_py{6}.Value = opt(6).y;
            FEAD_Animation_UpdateLayout(S.anim, S.pulleys);
            recompute_and_plot();
            uialert(fig, sprintf('Tensioner moved to X=%.0f, Y=%.0f mm\nTotal hub load reduced.',...
                opt(6).x, opt(6).y),'Layout Optimised','Icon','success');
        else
            uialert(fig,'Current tensioner position is already optimal.','Optimisation','Icon','info');
        end
    end

    function open_data_window()
        if isempty(S.dw) || ~isvalid(S.dw.fig)
            S.dw = FEAD_DataWindow();
        else
            figure(S.dw.fig);
        end
        FEAD_DataWindow_Update(S.dw, S.last_report, S.belt, S.rpm, S.wp, S.load_table, S.pulleys);
    end

    function build_simscape()
        assignin('base','pulleys', S.pulleys);
        assignin('base','belt',    S.belt);
        FEAD_TestRig_Builder(S.pulleys, S.belt, S.load_table, S.conditions);
        uialert(fig,'FEAD_TestRig.slx built and opened in Simulink.','Simscape','Icon','success');
    end

    function push_github()
        push_to_github(S.pulleys, S.belt, S.last_report);
    end

    function reset_datum()
        FEAD_params;
        S.pulleys = evalin('base','pulleys');
        for k = 1:np
            S.ef_px{k}.Value = S.pulleys(k).x;
            S.ef_py{k}.Value = S.pulleys(k).y;
            S.ef_pr{k}.Value = S.pulleys(k).r;
        end
        FEAD_Animation_UpdateLayout(S.anim, S.pulleys);
        recompute_and_plot();
    end

    function import_from_web()
        % Try to read layout JSON exported from the website
        [f,p] = uigetfile('*.json','Select layout JSON from web tool');
        if f == 0, return; end
        try
            txt  = fileread(fullfile(p,f));
            data = jsondecode(txt);
            if isfield(data,'pulleys')
                for k = 1:min(numel(data.pulleys),np)
                    if isfield(data.pulleys(k),'x'), S.pulleys(k).x = data.pulleys(k).x; end
                    if isfield(data.pulleys(k),'y'), S.pulleys(k).y = data.pulleys(k).y; end
                    if isfield(data.pulleys(k),'r'), S.pulleys(k).r = data.pulleys(k).r; end
                    S.ef_px{k}.Value = S.pulleys(k).x;
                    S.ef_py{k}.Value = S.pulleys(k).y;
                    S.ef_pr{k}.Value = S.pulleys(k).r;
                end
            end
            if isfield(data,'rpm'),           S.rpm = data.rpm; S.sl_rpm.Value = S.rpm; end
            if isfield(data,'staticTension'), S.belt.static_tension = data.staticTension; S.sl_ten.Value = S.belt.static_tension; end
            FEAD_Animation_UpdateLayout(S.anim, S.pulleys);
            recompute_and_plot();
            uialert(fig,'Web layout imported successfully.','Import','Icon','success');
        catch e
            uialert(fig,['Import failed: ' e.message],'Import Error','Icon','error');
        end
    end

    function on_belt_change(val)
        idx = find(strcmp({belt_lib.name}, val),1);
        if isempty(idx), return; end
        S.belt = belt_lib(idx);
        S.belt.static_tension = evalin('base','belt.static_tension');
        S.belt.length_m       = S.belt.length_mm/1000;
        S.lbl_belt_info.Text  = sprintf('L=%.0fmm  μ=%.2f  %drib  %s',...
            S.belt.length_mm, S.belt.mu, S.belt.ribs, S.belt.core);
        recompute_and_plot();
    end

    function on_rpm_change(val)
        S.rpm = round(val/50)*50;
        S.sl_rpm.Value = S.rpm;
        S.lbl_rpm.Text = sprintf('%d RPM', S.rpm);
        recompute_and_plot();
    end

    function on_tension_change(val)
        S.belt.static_tension = round(val/10)*10;
        S.lbl_ten.Text = sprintf('%d N', S.belt.static_tension);
        recompute_and_plot();
    end

    function on_ten_pos(val)
        idx = find(strcmp({S.ten_pos.label}, val),1);
        if isempty(idx), return; end
        S.ten_idx = idx;
        S.pulleys(6).x = S.ten_pos(idx).x;
        S.pulleys(6).y = S.ten_pos(idx).y;
        S.belt.static_tension = S.ten_pos(idx).T;
        S.ef_px{6}.Value = S.pulleys(6).x;
        S.ef_py{6}.Value = S.pulleys(6).y;
        S.sl_ten.Value = S.belt.static_tension;
        S.lbl_ten.Text = sprintf('%d N', round(S.belt.static_tension));
        FEAD_Animation_UpdateLayout(S.anim, S.pulleys);
        recompute_and_plot();
    end

    function on_cond_change(field, val)
        S.conditions.(field) = val;
        recompute_and_plot();
    end

    function on_pulley_edit(k, field, val)
        S.pulleys(k).(field) = val;
        FEAD_Animation_UpdateLayout(S.anim, S.pulleys);
        recompute_and_plot();
    end

    function on_mouse_down(~,~)
        cp = ax_anim.CurrentPoint;
        if isempty(cp), return; end
        mx = cp(1,1); my = cp(1,2);
        for k = 1:np
            if hypot(mx-S.pulleys(k).x, my-S.pulleys(k).y) < S.pulleys(k).r*1.4
                S.dragging = k; return;
            end
        end
    end

    function on_mouse_move(~,~)
        if S.dragging < 1, return; end
        cp = ax_anim.CurrentPoint;
        if isempty(cp), return; end
        S.pulleys(S.dragging).x = round(cp(1,1));
        S.pulleys(S.dragging).y = round(cp(1,2));
        S.ef_px{S.dragging}.Value = S.pulleys(S.dragging).x;
        S.ef_py{S.dragging}.Value = S.pulleys(S.dragging).y;
        FEAD_Animation_UpdateLayout(S.anim, S.pulleys);
    end

    function on_mouse_up(~,~)
        if S.dragging > 0
            S.dragging = -1;
            recompute_and_plot();  % full recalculate on release
        end
    end

    function on_close(~,~)
        if strcmp(S.timer.Running,'on'), stop(S.timer); end
        delete(S.timer);
        if ~isempty(S.dw) && isvalid(S.dw.fig)
            delete(S.dw.fig);
        end
        delete(fig);
    end

end % FEAD_App

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
