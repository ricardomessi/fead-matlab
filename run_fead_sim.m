%% run_fead_sim.m  –  Run the FEAD Belt Drive simulation & auto-plot results
%  Computes hub loads, belt tensions, slip SF, fatigue life, WP bearing life
%  and frictional power — both numerically and with publication-quality plots.
%
%  Usage:  >> FEAD_params; run_fead_sim
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('pulleys','var'), FEAD_params; end

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('  FEAD Belt Drive Test Rig  –  H6 OEM Engine\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

%% ── Sweep RPM range ─────────────────────────────────────────────────────────
rpm_sweep = 300:50:2500;
n_rpm     = numel(rpm_sweep);
pnames    = {'CRK','FAN','IDR','ALT','AC','TEN'};
np        = numel(pnames);

% Pre-allocate result arrays
F_hub    = zeros(np, n_rpm);   % Hub load magnitudes [N]
dir_hub  = zeros(np, n_rpm);   % Hub load directions [deg]
T_tight  = zeros(np, n_rpm);   % Tight-side tensions [N]
T_slack  = zeros(np, n_rpm);   % Slack-side tensions [N]
T_cent   = zeros(np, n_rpm);   % Centrifugal tension [N]
SF_slip  = zeros(np, n_rpm);   % Slip safety factor
v_belt   = zeros(1, n_rpm);    % Belt velocity [m/s]
P_total  = zeros(1, n_rpm);    % Total FEAD power [kW]

% Wrap angles (from Reference PDF baseline)
wrap_deg = [166.5, 127.6, 108.4, 145.1, 105.7, 76.4];

fprintf('Computing over %d RPM points...\n', n_rpm);

for ri = 1:n_rpm
    rpm = rpm_sweep(ri);
    v   = pulleys(1).r/1000 * rpm*2*pi/60;
    if v < 0.01, v = 0.01; end
    v_belt(ri) = v;
    T_c = belt.lin_mass * v^2;

    P_sum = 0;
    for k = 1:np
        pn  = pnames{k};
        P_k = interp1(load_table.rpm, load_table.(pn), rpm, 'linear', 'extrap');
        P_k = max(P_k, 0);
        P_sum = P_sum + P_k;

        T_eff = P_k * 1000 / v;
        T_t   = belt.static_tension + T_eff/2 + T_c;
        T_s   = max(belt.static_tension - T_eff/2, 0) + T_c;

        T_tight(k,ri) = T_t;
        T_slack(k,ri) = T_s;
        T_cent(k,ri)  = T_c;

        % Hub load vector (simplified: in-span + out-span tensions)
        kp = mod(k-2, np) + 1;
        kn = mod(k,   np) + 1;
        dx_p = pulleys(kp).x - pulleys(k).x;
        dy_p = pulleys(kp).y - pulleys(k).y;
        dx_n = pulleys(kn).x - pulleys(k).x;
        dy_n = pulleys(kn).y - pulleys(k).y;
        Lp = max(hypot(dx_p,dy_p), 1);
        Ln = max(hypot(dx_n,dy_n), 1);
        Fx = T_t * dx_p/Lp + T_s * dx_n/Ln;
        Fy = T_t * dy_p/Lp + T_s * dy_n/Ln;
        F_hub(k,ri)   = hypot(Fx,Fy);
        dir_hub(k,ri) = atan2d(Fy,Fx);

        % Slip SF – Capstan
        mu_th = belt.mu * wrap_deg(k) * pi/180;
        if T_s > 0.1
            SF_slip(k,ri) = log(T_t/T_s) / mu_th;
        else
            SF_slip(k,ri) = 99;
        end
    end
    P_total(ri) = P_sum;
end

fprintf('Done.\n\n');

%% ── Print results table at MEAN tensioner position (1200 RPM) ──────────────
[~, idx1200] = min(abs(rpm_sweep - 1200));
fprintf('%-6s  %8s  %8s  %8s  %8s  %6s  %6s\n',...
    'Pulley','T_tight','T_slack','T_cent','F_hub','Dir°','SF');
fprintf('%s\n', repmat('-',1,68));
for k = 1:np
    fprintf('%-6s  %8.1f  %8.1f  %8.1f  %8.1f  %6.1f  %6.2f\n',...
        pnames{k}, T_tight(k,idx1200), T_slack(k,idx1200),...
        T_cent(k,idx1200), F_hub(k,idx1200),...
        dir_hub(k,idx1200), min(SF_slip(k,idx1200),9.99));
end

% Compare with PDF baseline
fprintf('\nVs. Reference PDF Baseline (1200 RPM, MEAN tensioner):\n');
pdf_F   = [2658.9, 2866.4, 1710.1, 1678.1, 985.8, 608.5];
pdf_dir = [96, 258, 77, 279, 49, 237];
for k = 1:np
    dF  = F_hub(k,idx1200) - pdf_F(k);
    fprintf('  %-6s  F_calc=%6.0fN  F_pdf=%6.0fN  ΔF=%+6.0fN  (%.1f%%)\n',...
        pnames{k}, F_hub(k,idx1200), pdf_F(k), dF, 100*dF/pdf_F(k));
end

%% ── Fatigue Life (Wöhler + Palmgren-Miner, WLTC duty cycle) ────────────────
wltc_w = [sim.dutyCycle.wltcLow sim.dutyCycle.wltcMed ...
           sim.dutyCycle.wltcHigh sim.dutyCycle.wltcXHigh] / 100;
wltc_rpm = [900, 1200, 1600, 2000];

fprintf('\n--- Belt Fatigue Life (Wöhler/Palmgren-Miner + WLTC) ---\n');
fprintf('%-6s  %10s  %12s  %12s\n','Pulley','T_tight(N)','Nf (cycles)','Life (km)');
fprintf('%s\n', repmat('-',1,50));
belt_life_km = zeros(np,1);
for k = 1:np
    D_miner = 0;
    for wi = 1:4
        rpm_i  = wltc_rpm(wi);
        v_i    = pulleys(1).r/1000 * rpm_i * 2*pi/60;
        P_i    = interp1(load_table.rpm, load_table.(pnames{k}), rpm_i,'linear','extrap');
        T_t_i  = belt.static_tension + P_i*1000/v_i/2 + belt.lin_mass*v_i^2;
        N_f_i  = belt.wohler_Nref * (belt.wohler_Tref / max(T_t_i,1))^belt.wohler_m;
        D_miner = D_miner + wltc_w(wi) / N_f_i;
    end
    if D_miner > 0
        life_km = min(belt.length_m/1000 / D_miner, 500000);
    else
        life_km = 500000;
    end
    belt_life_km(k) = life_km;
    fprintf('%-6s  %10.1f  %12.2e  %12.0f\n',...
        pnames{k}, T_tight(k,idx1200), 1/D_miner, life_km);
end
overall_life = min(belt_life_km);
fprintf('OVERALL BELT LIFE: %.0f km  (limited by: %s)\n',...
    overall_life, pnames{belt_life_km == overall_life});

%% ── Water Pump Bearing Life ─────────────────────────────────────────────────
F_ten    = F_hub(6, idx1200);   % TEN pulley hub force as WP belt load proxy
wp_rpm_v = rpm_sweep * wp.gear_ratio;
L10A_v   = zeros(1,n_rpm);
L10B_v   = zeros(1,n_rpm);

for ri = 1:n_rpm
    F_t   = F_hub(6,ri);
    P_b   = sqrt((F_t*0.3)^2 + wp.F_radial^2);
    P_r   = sqrt((F_t*0.7)^2 + wp.F_radial^2);
    n_wp  = rpm_sweep(ri) * wp.gear_ratio;
    L10A_v(ri) = (wp.ball.Cr  /max(P_b,1))^wp.ball.p   * 1e6/(60*max(n_wp,1));
    L10B_v(ri) = (wp.roller.Cr/max(P_r,1))^wp.roller.p * 1e6/(60*max(n_wp,1));
end
L10_comp = ((L10A_v.^(-9/8) + L10B_v.^(-9/8)).^(-8/9));

fprintf('\n--- WP Bearing Life @ 1200 RPM ---\n');
fprintf('  Ball  L10A: %.0f h  (C&U ref: %d h)\n',  L10A_v(idx1200),  wp.ref.L10A);
fprintf('  Roller L10B: %.0f h  (C&U ref: %d h)\n', L10B_v(idx1200),  wp.ref.L10B);
fprintf('  Composite:   %.0f h  (C&U ref: %d h)\n', L10_comp(idx1200),wp.ref.L10comp);

%% ── Frictional Power (Non-AC vs AC-ON) ─────────────────────────────────────
P_AC_on  = zeros(1,n_rpm);
P_AC_off = zeros(1,n_rpm);
for ri = 1:n_rpm
    rpm = rpm_sweep(ri);
    v   = max(v_belt(ri), 0.01);
    for k = 1:np
        P_k = interp1(load_table.rpm, load_table.(pnames{k}), rpm,'linear','extrap');
        P_AC_on(ri)  = P_AC_on(ri)  + max(P_k,0);
        if k ~= 5  % skip AC compressor
            P_AC_off(ri) = P_AC_off(ri) + max(P_k,0);
        end
    end
end
delta_P = P_AC_on - P_AC_off;

fprintf('\n--- Frictional Power (AC overhead) @ 1200 RPM ---\n');
fprintf('  FEAD No-AC: %.2f kW\n',   P_AC_off(idx1200));
fprintf('  FEAD AC-ON: %.2f kW\n',   P_AC_on(idx1200));
fprintf('  Delta:      %.2f kW\n',   delta_P(idx1200));
fprintf('  Belt friction overhead ≈ %.1f%%\n', 100*(delta_P(idx1200)/P_AC_on(idx1200)));

%% ═══════════════════════════════════════════════════════════════════════════
%  PUBLICATION-QUALITY PLOTS  (dark amber/violet theme)
% ═══════════════════════════════════════════════════════════════════════════
BG  = [0.07 0.09 0.15];
AX  = [0.12 0.18 0.28];
TC  = [0.89 0.91 0.94];
colors = {[0.96 0.62 0.04],[0.55 0.36 0.96],[0.65 0.54 0.98],...
          [0.20 0.85 0.60],[0.96 0.28 0.71],[0.38 0.64 0.98]};

fig_main = figure('Color',BG,'Position',[50 50 1600 900],...
    'Name','FEAD Simulation Results');

%% Plot 1: Hub Loads vs RPM
ax1 = subplot(2,3,1,'Parent',fig_main);
hold(ax1,'on'); set(ax1,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
for k = 1:np
    plot(ax1, rpm_sweep, F_hub(k,:),'LineWidth',2,'Color',colors{k},'DisplayName',pnames{k});
end
% Add PDF reference points
pdf_vals = [2658.9, 2866.4, 1710.1, 1678.1, 985.8, 608.5];
for k = 1:np
    plot(ax1,1200,pdf_vals(k),'o','MarkerSize',8,'Color',colors{k},'MarkerFaceColor',colors{k});
end
xlabel(ax1,'Engine Speed (RPM)','Color',TC); ylabel(ax1,'Hub Load F (N)','Color',TC);
title(ax1,'Hub Loads vs RPM','Color',[0.96 0.62 0.04],'FontWeight','bold');
legend(ax1,'TextColor',TC,'Color',AX,'Location','northwest');
grid(ax1,'on'); xline(ax1,1200,'--','Color',[1 1 1 0.3]);

%% Plot 2: Belt Tensions vs RPM
ax2 = subplot(2,3,2,'Parent',fig_main);
hold(ax2,'on'); set(ax2,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
for k = 1:np
    plot(ax2,rpm_sweep,T_tight(k,:),'LineWidth',2,'Color',colors{k},'DisplayName',[pnames{k} ' tight']);
    plot(ax2,rpm_sweep,T_slack(k,:),'--','LineWidth',1,'Color',colors{k}*0.65);
end
xlabel(ax2,'RPM','Color',TC); ylabel(ax2,'Tension (N)','Color',TC);
title(ax2,'Belt Tensions (solid=tight, dashed=slack)','Color',[0.96 0.62 0.04],'FontWeight','bold');
legend(ax2,'TextColor',TC,'Color',AX,'Location','northwest','NumColumns',2);
grid(ax2,'on');

%% Plot 3: Slip Safety Factor vs RPM
ax3 = subplot(2,3,3,'Parent',fig_main);
hold(ax3,'on'); set(ax3,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
for k = 1:np
    SF_plot = min(SF_slip(k,:), 6);
    plot(ax3,rpm_sweep,SF_plot,'LineWidth',2,'Color',colors{k},'DisplayName',pnames{k});
end
yline(ax3,1.0,'r--','LineWidth',2,'DisplayName','SF=1 (slip)');
yline(ax3,1.3,'Color',[0.96 0.62 0.04],'LineStyle','--','LineWidth',1.5,'DisplayName','SF=1.3 (safe)');
xlabel(ax3,'RPM','Color',TC); ylabel(ax3,'Slip Safety Factor','Color',TC);
title(ax3,'Capstan Slip SF vs RPM','Color',[0.96 0.62 0.04],'FontWeight','bold');
legend(ax3,'TextColor',TC,'Color',AX,'Location','northwest'); ylim(ax3,[0 6]);
grid(ax3,'on');

%% Plot 4: WP Bearing Life vs RPM
ax4 = subplot(2,3,4,'Parent',fig_main);
hold(ax4,'on'); set(ax4,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
L10A_plot = min(L10A_v, 99999);
L10B_plot = min(L10B_v, 99999);
L10C_plot = min(L10_comp,99999);
plot(ax4,rpm_sweep,L10A_plot,'Color',[0.38 0.64 0.98],'LineWidth',2,'DisplayName','Ball L10A');
plot(ax4,rpm_sweep,L10B_plot,'Color',[0.65 0.54 0.98],'LineWidth',2,'DisplayName','Roller L10B');
plot(ax4,rpm_sweep,L10C_plot,'Color',[0.20 0.85 0.60],'LineWidth',2.5,'DisplayName','Composite');
yline(ax4,wp.ref.L10A,   '--','Color',[0.38 0.64 0.98 0.6],'LineWidth',1,'DisplayName','Ref L10A');
yline(ax4,wp.ref.L10B,   '--','Color',[0.65 0.54 0.98 0.6],'LineWidth',1,'DisplayName','Ref L10B');
yline(ax4,wp.ref.L10comp,'--','Color',[0.20 0.85 0.60 0.6],'LineWidth',1,'DisplayName','Ref Comp');
xlabel(ax4,'RPM','Color',TC); ylabel(ax4,'L10 Life (hours)','Color',TC);
title(ax4,'WP Bearing Life (ISO 281) vs RPM','Color',[0.96 0.62 0.04],'FontWeight','bold');
legend(ax4,'TextColor',TC,'Color',AX,'Location','northeast');
grid(ax4,'on');

%% Plot 5: FEAD Power (AC vs No-AC)
ax5 = subplot(2,3,5,'Parent',fig_main);
hold(ax5,'on'); set(ax5,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
h_ac_on  = area(ax5,rpm_sweep,P_AC_on, 'FaceColor',[0.96 0.28 0.71],'EdgeColor',[0.96 0.28 0.71],'LineWidth',2,'DisplayName','AC ON');
h_ac_off = area(ax5,rpm_sweep,P_AC_off,'FaceColor',[0.55 0.36 0.96],'EdgeColor',[0.65 0.54 0.98],'LineWidth',2,'DisplayName','No AC');
h_ac_on.FaceAlpha  = 0.25;
h_ac_off.FaceAlpha = 0.25;
plot(ax5,rpm_sweep,delta_P,'Color',[0.94 0.27 0.27],'LineWidth',2,'DisplayName','ΔP (AC overhead)');
xlabel(ax5,'RPM','Color',TC); ylabel(ax5,'Power (kW)','Color',TC);
title(ax5,'FEAD Power: AC vs Non-AC','Color',[0.96 0.62 0.04],'FontWeight','bold');
legend(ax5,'TextColor',TC,'Color',AX,'Location','northwest');
grid(ax5,'on');

%% Plot 6: Belt Fatigue Life (bar)
ax6 = subplot(2,3,6,'Parent',fig_main);
set(ax6,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
bar_colors = cell2mat(reshape(colors,1,1,[]));
b = bar(ax6,belt_life_km/1000,'FaceColor','flat');
b.CData = cell2mat(cellfun(@(c)c,colors','UniformOutput',false));
set(ax6,'XTickLabel',pnames);
xlabel(ax6,'Pulley','Color',TC); ylabel(ax6,'Belt Life (×10³ km)','Color',TC);
title(ax6,'Belt Fatigue Life per Pulley (WLTC Weighted)','Color',[0.96 0.62 0.04],'FontWeight','bold');
yline(ax6,500,'--','Color',[0.96 0.62 0.04],'LineWidth',1.5,'DisplayName','500 000 km limit');
grid(ax6,'on');

% Super title
sgtitle(fig_main,'FEAD Test Rig — H6 OEM Engine — Simscape Results',...
    'Color',[0.96 0.62 0.04],'FontSize',16,'FontWeight','bold');

%% ── Save results to workspace ───────────────────────────────────────────────
fead_sim_results.rpm_sweep  = rpm_sweep;
fead_sim_results.F_hub      = F_hub;
fead_sim_results.dir_hub    = dir_hub;
fead_sim_results.T_tight    = T_tight;
fead_sim_results.T_slack    = T_slack;
fead_sim_results.SF_slip    = SF_slip;
fead_sim_results.v_belt     = v_belt;
fead_sim_results.P_total    = P_total;
fead_sim_results.belt_life_km = belt_life_km;
fead_sim_results.L10A       = L10A_v;
fead_sim_results.L10B       = L10B_v;
fead_sim_results.L10_comp   = L10_comp;
fead_sim_results.P_AC_on    = P_AC_on;
fead_sim_results.P_AC_off   = P_AC_off;
assignin('base','fead_sim_results', fead_sim_results);

fprintf('\nResults saved to workspace: ''fead_sim_results''\n');
fprintf('Overall belt life: %.0f km\n', min(belt_life_km));
fprintf('WP L10 composite @ 1200 RPM: %.0f h\n', L10_comp(idx1200));
