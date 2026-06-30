%% FEAD_DataWindow.m  –  Standalone Live Data Display Window
%  Opens a separate figure window showing all computed FEAD values
%  updated in real time. Called from FEAD_App.m.
%
%  Usage:  dw = FEAD_DataWindow();
%          FEAD_DataWindow_Update(dw, report, belt, rpm);
% ─────────────────────────────────────────────────────────────────────────────

function dw = FEAD_DataWindow()
%FEAD_DATAWINDOW  Create the data display window.

BG = [0.06 0.08 0.14];
TC = [0.89 0.91 0.94];
AX = [0.09 0.12 0.20];

fig = uifigure('Name','FEAD Live Data – All Results',...
    'Position',[1320 60 700 940],...
    'Color',BG,...
    'Resize','on');

% ── Title bar ────────────────────────────────────────────────────────────────
uilabel(fig,'Text','FEAD Live Data Monitor',...
    'Position',[10 900 680 32],...
    'FontSize',16,'FontWeight','bold',...
    'FontColor',[0.96 0.62 0.04],...
    'HorizontalAlignment','center',...
    'BackgroundColor',BG);

uilabel(fig,'Text','H6 OEM Engine · MT620 AMD · Real-time Computation',...
    'Position',[10 876 680 20],...
    'FontSize',10,'FontColor',[0.55 0.62 0.72],...
    'HorizontalAlignment','center','BackgroundColor',BG);

% ── Tab group ────────────────────────────────────────────────────────────────
tg = uitabgroup(fig,'Position',[5 10 690 865]);

tab1 = uitab(tg,'Title','Hub Loads',    'BackgroundColor',AX);
tab2 = uitab(tg,'Title','Tensions',     'BackgroundColor',AX);
tab3 = uitab(tg,'Title','Validation',   'BackgroundColor',AX);
tab4 = uitab(tg,'Title','Fatigue/Life', 'BackgroundColor',AX);
tab5 = uitab(tg,'Title','Belt Data',    'BackgroundColor',AX);

% ─────────────────────────────────────────────────────────────────────────────
% TAB 1: Hub Loads table
% ─────────────────────────────────────────────────────────────────────────────
pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
col_names_hub = {'Pulley','F_hub (N)','Dir (°)','T_tight (N)','T_slack (N)','T_cent (N)'};
dw.tbl_hub = uitable(tab1,...
    'Position',[10 10 665 800],...
    'ColumnName',col_names_hub,...
    'ColumnWidth',{80 100 80 110 110 100},...
    'RowName',[],...
    'Data',cell(6,6),...
    'BackgroundColor',repmat([AX; AX*1.2],3,1),...
    'FontColor',TC,...
    'FontSize',11);

% ─────────────────────────────────────────────────────────────────────────────
% TAB 2: Tensions
% ─────────────────────────────────────────────────────────────────────────────
col_names_ten = {'Pulley','P (kW)','v_belt (m/s)','T_eff (N)','T_tight (N)','T_slack (N)','SF (slip)'};
dw.tbl_ten = uitable(tab2,...
    'Position',[10 10 665 800],...
    'ColumnName',col_names_ten,...
    'ColumnWidth',{70 80 100 90 100 100 80},...
    'RowName',[],...
    'Data',cell(6,7),...
    'BackgroundColor',repmat([AX; AX*1.2],3,1),...
    'FontColor',TC,'FontSize',11);

% ─────────────────────────────────────────────────────────────────────────────
% TAB 3: Validation
% ─────────────────────────────────────────────────────────────────────────────
dw.val_score_lbl = uilabel(tab3,...
    'Position',[10 780 665 55],...
    'Text','Design Score: — / 100',...
    'FontSize',22,'FontWeight','bold',...
    'FontColor',[0.20 0.85 0.60],...
    'HorizontalAlignment','center',...
    'BackgroundColor',AX);

col_names_val = {'Check','Status','Value','Limit','Notes'};
dw.tbl_val = uitable(tab3,...
    'Position',[10 400 665 375],...
    'ColumnName',col_names_val,...
    'ColumnWidth',{170 70 110 110 190},...
    'RowName',[],...
    'Data',cell(8,5),...
    'BackgroundColor',repmat([AX; AX*1.2],4,1),...
    'FontColor',TC,'FontSize',10);

% Suggestions text area
uilabel(tab3,'Text','Design Suggestions & Optimisation',...
    'Position',[10 370 665 24],...
    'FontSize',12,'FontWeight','bold','FontColor',[0.96 0.62 0.04],...
    'BackgroundColor',AX);
dw.txt_suggestions = uitextarea(tab3,...
    'Position',[10 10 665 355],...
    'FontSize',10,...
    'Value',{'Suggestions will appear here after validation...'},...
    'BackgroundColor',[0.07 0.10 0.17],...
    'FontColor',TC,'Editable','off');

% ─────────────────────────────────────────────────────────────────────────────
% TAB 4: Fatigue & Bearing Life
% ─────────────────────────────────────────────────────────────────────────────
col_names_fat = {'Pulley','T_tight (N)','Nf (cycles)','Life (km)','Miner D','Status'};
dw.tbl_fat = uitable(tab4,...
    'Position',[10 500 665 320],...
    'ColumnName',col_names_fat,...
    'ColumnWidth',{80 110 120 100 90 100},...
    'RowName',[],'Data',cell(6,6),...
    'BackgroundColor',repmat([AX; AX*1.2],3,1),...
    'FontColor',TC,'FontSize',11);

uilabel(tab4,'Text','Water Pump Bearing Life (ISO 281)',...
    'Position',[10 465 665 28],...
    'FontSize',12,'FontWeight','bold','FontColor',[0.38 0.64 0.98],...
    'BackgroundColor',AX);

col_names_wp = {'Bearing','Cr (N)','P_load (N)','L10 (h)','Ref L10 (h)','Status'};
dw.tbl_wp = uitable(tab4,...
    'Position',[10 260 665 200],...
    'ColumnName',col_names_wp,...
    'ColumnWidth',{100 90 100 100 110 100},...
    'RowName',[],'Data',cell(3,6),...
    'BackgroundColor',repmat([AX; AX*1.2],2,1),...
    'FontColor',TC,'FontSize',11);

uilabel(tab4,'Text','Frictional Power (AC vs Non-AC)',...
    'Position',[10 225 665 28],...
    'FontSize',12,'FontWeight','bold','FontColor',[0.96 0.28 0.71],...
    'BackgroundColor',AX);
dw.txt_power = uitextarea(tab4,...
    'Position',[10 10 665 210],...
    'FontSize',11,'Value',{'Loading...'},...
    'BackgroundColor',[0.07 0.10 0.17],'FontColor',TC,'Editable','off');

% ─────────────────────────────────────────────────────────────────────────────
% TAB 5: Belt data
% ─────────────────────────────────────────────────────────────────────────────
dw.txt_belt = uitextarea(tab5,...
    'Position',[10 10 665 820],...
    'FontSize',11,'Value',{'Belt properties will load here...'},...
    'BackgroundColor',[0.07 0.10 0.17],'FontColor',TC,'Editable','off');

dw.fig = fig;
end % FEAD_DataWindow


function FEAD_DataWindow_Update(dw, report, belt, rpm, wp, load_table, pulleys)
%FEAD_DATAWINDOW_UPDATE  Refresh all tables with latest computed values.

if ~isvalid(dw.fig), return; end  % window closed

pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
np     = 6;
v      = pulleys(1).r/1000 * rpm*2*pi/60;
if v < 0.01, v = 0.01; end

%% ── Tab 1: Hub loads ─────────────────────────────────────────────────────────
data_hub = cell(np,6);
for k = 1:np
    kp = mod(k-2,np)+1; kn = mod(k,np)+1;
    dx_p = pulleys(kp).x-pulleys(k).x; dy_p = pulleys(kp).y-pulleys(k).y;
    dx_n = pulleys(kn).x-pulleys(k).x; dy_n = pulleys(kn).y-pulleys(k).y;
    Lp = max(hypot(dx_p,dy_p),1); Ln = max(hypot(dx_n,dy_n),1);
    Fx = report.T_tight(k)*dx_p/Lp + report.T_slack(k)*dx_n/Ln;
    Fy = report.T_tight(k)*dy_p/Lp + report.T_slack(k)*dy_n/Ln;
    dir_k = atan2d(Fy,Fx);
    T_c   = belt.lin_mass * v^2;

    data_hub{k,1} = pnames{k};
    data_hub{k,2} = sprintf('%.0f', report.F_hub(k));
    data_hub{k,3} = sprintf('%.1f', dir_k);
    data_hub{k,4} = sprintf('%.0f', report.T_tight(k));
    data_hub{k,5} = sprintf('%.0f', report.T_slack(k));
    data_hub{k,6} = sprintf('%.1f', T_c);
end
dw.tbl_hub.Data = data_hub;

%% ── Tab 2: Tensions ─────────────────────────────────────────────────────────
data_ten = cell(np,7);
for k = 1:np
    P_k   = max(interp1(load_table.rpm, load_table.(pnames{k}), rpm,'linear','extrap'),0);
    T_eff = P_k*1000/v;
    data_ten{k,1} = pnames{k};
    data_ten{k,2} = sprintf('%.2f', P_k);
    data_ten{k,3} = sprintf('%.2f', v);
    data_ten{k,4} = sprintf('%.0f', T_eff);
    data_ten{k,5} = sprintf('%.0f', report.T_tight(k));
    data_ten{k,6} = sprintf('%.0f', report.T_slack(k));
    sf_k = report.SF(k);
    if sf_k >= 1.3, st='✅ OK'; elseif sf_k >= 1.0, st='⚠ MARG'; else, st='❌ SLIP'; end
    data_ten{k,7} = sprintf('%.2f  %s', min(sf_k,9.99), st);
end
dw.tbl_ten.Data = data_ten;

%% ── Tab 3: Validation ────────────────────────────────────────────────────────
checks = report.checks;
data_val = cell(numel(checks),5);
for i = 1:numel(checks)
    c = checks{i};
    data_val{i,1} = c.label;
    data_val{i,2} = ternary(c.pass,'✅ PASS','❌ FAIL');
    if isfield(c,'value')
        data_val{i,3} = sprintf('%.2f', c.value);
    elseif isfield(c,'values')
        data_val{i,3} = sprintf('%.1f', min(c.values));
    else
        data_val{i,3} = sprintf('%.2f', min(c.SF));
    end
    if isfield(c,'limit'), data_val{i,4} = sprintf('%.1f', c.limit);
    else,                  data_val{i,4} = '1.3'; end
    data_val{i,5} = c.msg;
end
dw.tbl_val.Data = data_val;
score_str = sprintf('Design Score:  %d / 100  —  %d of %d checks passed',...
    report.score, report.n_pass, report.n_total);
dw.val_score_lbl.Text = score_str;
if report.score >= 80, dw.val_score_lbl.FontColor = [0.20 0.85 0.60];
elseif report.score >= 60, dw.val_score_lbl.FontColor = [0.96 0.62 0.04];
else, dw.val_score_lbl.FontColor = [0.94 0.27 0.27]; end
dw.txt_suggestions.Value = report.suggestions;

%% ── Tab 4: Fatigue ──────────────────────────────────────────────────────────
wltc_rpm = [900 1200 1600 2000];
wltc_w   = [0.25 0.25 0.25 0.25];
data_fat = cell(np,6);
for k = 1:np
    D = 0;
    for wi=1:4
        vi  = max(pulleys(1).r/1000*wltc_rpm(wi)*2*pi/60,0.01);
        P_i = max(interp1(load_table.rpm,load_table.(pnames{k}),wltc_rpm(wi),'linear','extrap'),0);
        Tt  = belt.static_tension + P_i*1000/vi/2 + belt.lin_mass*vi^2;
        Nf  = belt.wohler_Nref*(belt.wohler_Tref/max(Tt,1))^belt.wohler_m;
        D   = D + wltc_w(wi)/Nf;
    end
    life_km = min(belt.length_m/1000/max(D,1e-15),500000);
    Nf_tot  = 1/max(D,1e-15);
    data_fat{k,1} = pnames{k};
    data_fat{k,2} = sprintf('%.0f', report.T_tight(k));
    data_fat{k,3} = sprintf('%.2e', Nf_tot);
    data_fat{k,4} = sprintf('%.0f', life_km);
    data_fat{k,5} = sprintf('%.3e', D);
    data_fat{k,6} = ternary(life_km>=200000,'✅ OK','⚠ LOW');
end
dw.tbl_fat.Data = data_fat;

% WP Bearing
F_ten = report.F_hub(6);
P_ball   = sqrt((F_ten*0.3)^2 + wp.F_radial^2);
P_roller = sqrt((F_ten*0.7)^2 + wp.F_radial^2);
n_wp     = rpm * wp.gear_ratio;
L10A  = (wp.ball.Cr  /max(P_ball,1))^wp.ball.p   * 1e6/(60*max(n_wp,1));
L10B  = (wp.roller.Cr/max(P_roller,1))^wp.roller.p * 1e6/(60*max(n_wp,1));
L10C  = ((L10A^(-9/8)+L10B^(-9/8))^(-8/9));
data_wp = {
    'Ball (C&U)',   sprintf('%.0f',wp.ball.Cr),   sprintf('%.0f',P_ball),   sprintf('%.0f',L10A), sprintf('%d',wp.ref.L10A),   ternary(L10A>=wp.ref.L10A*0.9,'✅','⚠');
    'Roller (C&U)', sprintf('%.0f',wp.roller.Cr), sprintf('%.0f',P_roller), sprintf('%.0f',L10B), sprintf('%d',wp.ref.L10B),   ternary(L10B>=wp.ref.L10B*0.9,'✅','⚠');
    'Composite',    '—',                           '—',                       sprintf('%.0f',L10C), sprintf('%d',wp.ref.L10comp),ternary(L10C>=wp.ref.L10comp*0.9,'✅','⚠')
};
dw.tbl_wp.Data = data_wp;

% Power summary
P_ac_on = sum(arrayfun(@(k) max(interp1(load_table.rpm,load_table.(pnames{k}),rpm,'linear','extrap'),0), 1:np));
P_ac_off= P_ac_on - max(interp1(load_table.rpm,load_table.AC,rpm,'linear','extrap'),0);
dw.txt_power.Value = {
    sprintf('  Engine RPM            :  %d RPM', round(rpm))
    sprintf('  Belt velocity         :  %.2f m/s', v)
    sprintf('  FEAD power (AC ON)    :  %.2f kW', P_ac_on)
    sprintf('  FEAD power (AC OFF)   :  %.2f kW', P_ac_off)
    sprintf('  AC overhead           :  %.2f kW  (%.1f%%)', P_ac_on-P_ac_off, 100*(P_ac_on-P_ac_off)/max(P_ac_on,0.01))
    sprintf('  WP ball L10A          :  %.0f h  (ref %d h)', L10A, wp.ref.L10A)
    sprintf('  WP roller L10B        :  %.0f h  (ref %d h)', L10B, wp.ref.L10B)
    sprintf('  WP composite L10      :  %.0f h  (ref %d h)', L10C, wp.ref.L10comp)
};

%% ── Tab 5: Belt data ────────────────────────────────────────────────────────
dw.txt_belt.Value = {
    sprintf('  Belt Model            :  %s', belt.name)
    sprintf('  Type                  :  %s', belt.type)
    sprintf('  Ribs                  :  %d', belt.ribs)
    sprintf('  Pitch                 :  %.2f mm', belt.pitch_mm)
    sprintf('  Length                :  %.1f mm', belt.length_mm)
    sprintf('  Linear mass           :  %.3f kg/m', belt.lin_mass)
    sprintf('  Friction coeff (μ)    :  %.3f', belt.mu)
    sprintf('  Axial stiffness (EA)  :  %.0f N', belt.EA)
    sprintf('  Core material         :  %s', belt.core)
    sprintf('  Cover material        :  %s', belt.cover)
    sprintf('  Standard              :  %s', belt.standard)
    sprintf('  Part number           :  %s', belt.part_no)
    '  '
    sprintf('  Wöhler exponent m     :  %d', belt.wohler_m)
    sprintf('  Wöhler N_ref          :  %.1e cycles', belt.wohler_Nref)
    sprintf('  Wöhler T_ref          :  %.0f N', belt.wohler_Tref)
    sprintf('  Max allowable tension :  %.0f N', belt.T_max)
    sprintf('  Max belt velocity     :  %.0f m/s', belt.v_max)
    sprintf('  Temp range            :  [%.0f, %.0f] °C', belt.temp_min, belt.temp_max)
    '  '
    sprintf('  Current v_belt        :  %.2f m/s  (%.1f%% of limit)', v, 100*v/belt.v_max)
    sprintf('  Current T_max         :  %.0f N  (%.1f%% of limit)', max(report.T_tight), 100*max(report.T_tight)/belt.T_max)
};

end % FEAD_DataWindow_Update

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
