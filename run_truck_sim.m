%% run_truck_sim.m  –  H6 Truck System simulation & auto-plot results
%  Simulates 0→80 km/h acceleration, cruise, and braking with
%  full FEAD parasitic coupling, AMT gear shifts, and road load.
%
%  FIXED: ODE parameters passed via struct P — no workspace access issues.
%  Usage:  >> FEAD_params; run_truck_sim
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('engine','var'), FEAD_params; end

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('  H6 Truck System Simulation  –  Ashok Leyland H6\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

%% ── Pack ALL parameters into a single struct P ───────────────────────────────
%  This is the critical fix: local functions in MATLAB cannot access the
%  script workspace, so we bundle everything into P and pass it explicitly.

P.engine      = engine;
P.trans       = trans;
P.tyre        = tyre;
P.vehicle     = vehicle;
P.belt        = belt;
P.pulleys     = pulleys;
P.load_table  = load_table;
P.driveline   = driveline;
P.pnames      = {'CRK','FAN','IDR','ALT','AC','TEN'};
P.n_drive     = n_drive_wheels;

% Drive cycle (speed reference and road grade)
P.t_ref  = [0   5   15   35   70  100  115  120];
P.v_ref  = [0   0   20   80   80    0    0    0] / 3.6;   % m/s
P.gr_ref = [0   0    0    0    0    5    0    0];          % road grade deg

% PID gains
P.Kp = 1200;  P.Ki = 180;  P.Kd = 40;

% Derived constants (pre-computed to speed up ODE)
P.m        = vehicle.GVW_kg;
P.Cd       = vehicle.Cd;
P.A_f      = vehicle.frontal_area;
P.rho      = vehicle.rho_air;
P.Cr       = tyre.rolling_res;
P.R_w      = tyre.R_loaded_m;
P.Iz_w     = tyre.Iz * n_drive_wheels;
P.Iz_e     = engine.flywheel_Iz;
P.k_s      = driveline.propshaft_k;
P.c_s      = driveline.propshaft_c;
P.i_fd     = trans.final_drive;
P.ratios   = trans.ratios;

%% ── Initial conditions ───────────────────────────────────────────────────────
%  x = [v_veh(m/s); omega_eng(rad/s); omega_wheel(rad/s); theta_prop(rad);
%        int_err; prev_err]
x0 = [0;
      engine.idle_rpm * 2*pi/60;
      0;
      0;
      0;
      0];

%% ── Run ODE45 ────────────────────────────────────────────────────────────────
fprintf('Integrating ODE45 (%.0f s)...\n', sim.T_end_truck);
opts     = odeset('RelTol',1e-4,'AbsTol',1e-5,'MaxStep',0.05);
t_span   = [0 sim.T_end_truck];

% Anonymous function captures P — this is the correct MATLAB pattern
ode_func = @(t,x) truck_ode(t, x, P);
[T_sol, X_sol] = ode45(ode_func, t_span, x0, opts);

fprintf('Integration done: %d time points.\n\n', numel(T_sol));

%% ── Extract states ───────────────────────────────────────────────────────────
v_veh_sol     = max(X_sol(:,1), 0);            % m/s
omega_eng_sol = max(X_sol(:,2), 0);            % rad/s
rpm_eng_sol   = omega_eng_sol * 60/(2*pi);
omega_whl_sol = max(X_sol(:,3), 0);            % rad/s
v_kmh         = v_veh_sol * 3.6;

%% ── Reconstruct derived signals ──────────────────────────────────────────────
n_t      = numel(T_sol);
gear_v   = zeros(n_t,1);
thr_v    = zeros(n_t,1);
brk_v    = zeros(n_t,1);
T_eng_v  = zeros(n_t,1);
P_fead_v = zeros(n_t,1);

for i = 1:n_t
    rpm_i  = rpm_eng_sol(i);
    v_i    = v_veh_sol(i);
    t_i    = T_sol(i);
    v_tgt  = interp1(P.t_ref, P.v_ref, t_i,'linear','extrap');

    % Throttle / brake
    err_i  = v_tgt - v_i;
    thr_v(i) = max(0, min(1,  P.Kp*err_i/P.m));
    brk_v(i) = max(0, min(1, -P.Kp*err_i/(tyre.mu_peak*P.m*9.81)));

    % Gear
    gear_v(i) = gear_select(rpm_i);

    % Engine torque
    T_eng_v(i) = interp2(engine.torque_thr, engine.torque_rpm,...
        engine.torque_map', thr_v(i), rpm_i,'linear',0);

    % FEAD power
    v_b = max(pulleys(1).r/1000 * omega_eng_sol(i), 0.01);
    Pf  = 0;
    for k = 1:6
        Pf = Pf + max(interp1(load_table.rpm, load_table.(P.pnames{k}),...
            rpm_i,'linear','extrap'), 0);
    end
    P_fead_v(i) = Pf;
end

%% ── Print summary ────────────────────────────────────────────────────────────
[v_max, idx_vm] = max(v_kmh);
fprintf('--- Truck Simulation Summary ---\n');
fprintf('  Max speed:         %.1f km/h at t = %.1f s\n', v_max, T_sol(idx_vm));
idx60 = find(v_kmh >= 60, 1); if ~isempty(idx60), fprintf('  0→60 km/h:        %.1f s\n', T_sol(idx60)); end
idx80 = find(v_kmh >= 80, 1); if ~isempty(idx80), fprintf('  0→80 km/h:        %.1f s\n', T_sol(idx80)); end
fprintf('  Max engine RPM:    %.0f RPM\n', max(rpm_eng_sol));
fprintf('  Max gear reached:  %d\n', max(gear_v));
fprintf('  Avg FEAD loss:     %.1f kW\n', mean(P_fead_v));
fprintf('  Peak FEAD loss:    %.1f kW\n', max(P_fead_v));

%% ── Plot results ─────────────────────────────────────────────────────────────
BG=[0.07 0.09 0.15]; TC=[0.89 0.91 0.94]; AX=[0.10 0.14 0.22];
AM=[0.96 0.62 0.04]; VI=[0.65 0.54 0.98]; GR=[0.20 0.85 0.60];
RD=[0.94 0.27 0.27]; PK=[0.96 0.28 0.71];

fig_t = figure('Color',BG,'Position',[60 40 1560 860],...
    'Name','H6 Truck System – Simulation Results');

mkax = @(n) deal(subplot(3,3,n,'Parent',fig_t));

%% 1 – Speed
ax1 = mkax(1); set(ax1,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax1,'on');
v_ref_plot = interp1(P.t_ref, P.v_ref, T_sol,'linear','extrap')*3.6;
plot(ax1,T_sol,v_ref_plot,'--','Color',[TC 0.4],'LineWidth',1.5,'DisplayName','Reference');
plot(ax1,T_sol,v_kmh,'Color',AM,'LineWidth',2.5,'DisplayName','Actual');
xlabel(ax1,'Time (s)','Color',TC); ylabel(ax1,'Speed (km/h)','Color',TC);
title(ax1,'Vehicle Speed','Color',AM,'FontWeight','bold');
legend(ax1,'TextColor',TC,'Color',AX); grid(ax1,'on');

%% 2 – Engine RPM
ax2 = mkax(2); set(ax2,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
plot(T_sol,rpm_eng_sol,'Color',VI,'LineWidth',2);
yline(ax2,engine.rated_rpm,'--','Color',RD,'LineWidth',1.5,'Label','Rated RPM');
yline(ax2,engine.idle_rpm, ':','Color',[TC 0.3],'LineWidth',1,'Label','Idle');
xlabel(ax2,'Time (s)','Color',TC); ylabel(ax2,'RPM','Color',TC);
title(ax2,'Engine Speed','Color',AM,'FontWeight','bold'); grid(ax2,'on');

%% 3 – Gear
ax3 = mkax(3); set(ax3,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
stairs(T_sol,gear_v,'Color',GR,'LineWidth',2.5);
xlabel(ax3,'Time (s)','Color',TC); ylabel(ax3,'Gear','Color',TC);
title(ax3,'AMT Gear Position','Color',AM,'FontWeight','bold');
yticks(ax3,1:6); ylim(ax3,[0 7]); grid(ax3,'on');

%% 4 – Engine torque
ax4 = mkax(4); set(ax4,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
plot(T_sol,T_eng_v,'Color',AM,'LineWidth',2);
yline(ax4,engine.max_torque_Nm,'--','Color',RD,'LineWidth',1.5,'Label','Max Torque');
xlabel(ax4,'Time (s)','Color',TC); ylabel(ax4,'Torque (Nm)','Color',TC);
title(ax4,'Engine Output Torque','Color',AM,'FontWeight','bold'); grid(ax4,'on');

%% 5 – FEAD parasitic loss
ax5 = mkax(5); set(ax5,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
area(T_sol,P_fead_v,'FaceColor',[PK 0.25],'EdgeColor',PK,'LineWidth',2);
xlabel(ax5,'Time (s)','Color',TC); ylabel(ax5,'Power (kW)','Color',TC);
title(ax5,'FEAD Parasitic Loss','Color',AM,'FontWeight','bold'); grid(ax5,'on');

%% 6 – Throttle & Brake
ax6 = mkax(6); set(ax6,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax6,'on');
plot(T_sol,thr_v*100,'Color',GR,'LineWidth',2,'DisplayName','Throttle %');
plot(T_sol,brk_v*100,'Color',RD,'LineWidth',2,'DisplayName','Brake %');
xlabel(ax6,'Time (s)','Color',TC); ylabel(ax6,'%','Color',TC);
title(ax6,'Throttle / Brake Demand','Color',AM,'FontWeight','bold');
legend(ax6,'TextColor',TC,'Color',AX); grid(ax6,'on');

%% 7 – Wheel slip check
ax7 = mkax(7); set(ax7,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax7,'on');
v_whl_kmh = omega_whl_sol * tyre.R_loaded_m * 3.6;
plot(T_sol,v_whl_kmh,'Color',VI,'LineWidth',2,'DisplayName','Wheel v');
plot(T_sol,v_kmh,'--','Color',AM,'LineWidth',1.5,'DisplayName','Body v');
xlabel(ax7,'Time (s)','Color',TC); ylabel(ax7,'Speed (km/h)','Color',TC);
title(ax7,'Wheel vs Body Speed (Slip Check)','Color',AM,'FontWeight','bold');
legend(ax7,'TextColor',TC,'Color',AX); grid(ax7,'on');

%% 8 – Power budget
ax8 = mkax(8); set(ax8,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
P_eng_kW  = T_eng_v .* omega_eng_sol / 1000;
P_trac_kW = P_eng_kW - P_fead_v;
hold(ax8,'on');
area(T_sol,P_eng_kW, 'FaceColor',[AM 0.2],'EdgeColor',AM,'LineWidth',2,'DisplayName','Engine');
area(T_sol,max(P_trac_kW,0),'FaceColor',[GR 0.2],'EdgeColor',GR,'LineWidth',2,'DisplayName','Traction');
area(T_sol,P_fead_v, 'FaceColor',[PK 0.2],'EdgeColor',PK,'LineWidth',2,'DisplayName','FEAD loss');
xlabel(ax8,'Time (s)','Color',TC); ylabel(ax8,'Power (kW)','Color',TC);
title(ax8,'Power Budget','Color',AM,'FontWeight','bold');
legend(ax8,'TextColor',TC,'Color',AX); grid(ax8,'on');

%% 9 – Speed coloured by gear
ax9 = mkax(9); set(ax9,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax9,'on');
gear_colors = lines(6);
for g = 1:6
    mask = gear_v == g;
    if any(mask)
        scatter(ax9,T_sol(mask),v_kmh(mask),5,'filled',...
            'MarkerFaceColor',hsv2rgb([g/9 0.85 0.95]),'DisplayName',sprintf('G%d',g));
    end
end
xlabel(ax9,'Time (s)','Color',TC); ylabel(ax9,'Speed (km/h)','Color',TC);
title(ax9,'Speed Coloured by Gear','Color',AM,'FontWeight','bold');
legend(ax9,'TextColor',TC,'Color',AX,'NumColumns',3); grid(ax9,'on');

sgtitle(fig_t,'Ashok Leyland H6 – Full Truck Simulation (ODE45)',...
    'Color',AM,'FontSize',15,'FontWeight','bold');

%% ── Save results ─────────────────────────────────────────────────────────────
truck_results.T         = T_sol;
truck_results.v_kmh     = v_kmh;
truck_results.rpm       = rpm_eng_sol;
truck_results.gear      = gear_v;
truck_results.T_engine  = T_eng_v;
truck_results.P_fead_kW = P_fead_v;
truck_results.thr       = thr_v;
truck_results.brk       = brk_v;
assignin('base','truck_results', truck_results);
fprintf('\nResults saved to ''truck_results'' in workspace.\n');

%% ═══════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS  –  receive all params via struct P
% ═══════════════════════════════════════════════════════════════════════════

function dxdt = truck_ode(t, x, P)
%TRUCK_ODE  Full-vehicle longitudinal dynamics ODE.
%  State: x = [v_veh; omega_eng; omega_wheel; theta_prop; int_err; prev_err]

    v_veh    = x(1);
    om_eng   = max(x(2), P.engine.idle_rpm*2*pi/60);
    om_whl   = max(x(3), 0);
    th_prop  = x(4);
    int_err  = x(5);
    prev_err = x(6);

    % Reference speed & grade at this time
    v_tgt   = interp1(P.t_ref, P.v_ref,  t,'linear','extrap');
    grade_i = interp1(P.t_ref, P.gr_ref, t,'linear','extrap');

    % PID → throttle / brake
    err        = v_tgt - v_veh;
    int_err_n  = int_err + err * 0.05;
    derr       = (err - prev_err) / 0.05;
    pid_out    = P.Kp*err + P.Ki*int_err_n + P.Kd*derr;
    thr_i      = max(0, min(1,  pid_out / P.m));
    brk_i      = max(0, min(1, -pid_out / (P.tyre.mu_peak * P.m * 9.81)));

    % Gear selection
    rpm_i   = om_eng * 60 / (2*pi);
    g_i     = gear_select(rpm_i);
    i_total = P.ratios(g_i) * P.i_fd;

    % Engine torque from 2D map
    thr_clamped = max(0, min(1, thr_i));
    rpm_clamped = max(P.engine.torque_rpm(1), min(P.engine.torque_rpm(end), rpm_i));
    T_eng = interp2(P.engine.torque_thr, P.engine.torque_rpm,...
        P.engine.torque_map', thr_clamped, rpm_clamped, 'linear', 0);
    T_eng = max(T_eng, 0);

    % FEAD parasitic torque
    v_b = max(P.pulleys(1).r/1000 * om_eng, 0.01);
    P_f = 0;
    for k = 1:6
        p_k = max(interp1(P.load_table.rpm, P.load_table.(P.pnames{k}),...
            rpm_i, 'linear', 'extrap'), 0);
        P_f = P_f + p_k;
    end
    T_fead = P_f * 1000 / max(om_eng, 0.1);

    % Propshaft torque (torsional compliance)
    om_prop_ref = om_whl * i_total;
    T_shaft     = P.k_s * th_prop + P.c_s * (om_eng - om_prop_ref);

    % Engine EOM:  Iz_e * dom_eng/dt = T_eng - T_fead - T_friction - T_shaft/i
    T_friction = P.engine.friction_Nm;
    dom_eng    = (T_eng - T_fead - T_friction - T_shaft/max(i_total,0.01)) / P.Iz_e;

    % Wheel EOM: Pacejka longitudinal + brake
    kappa = (om_whl*P.R_w - max(v_veh,0.01)) / max(abs(v_veh),0.01);
    kappa = max(-1, min(1, kappa));
    Fz_w  = P.m * 9.81 / P.n_drive;
    Fx_1  = Fz_w * P.tyre.D * sin(P.tyre.C * ...
            atan(P.tyre.B*kappa - P.tyre.E*(P.tyre.B*kappa - atan(P.tyre.B*kappa))));
    Fx_tot = Fx_1 * P.n_drive;

    T_drive_whl = T_shaft * i_total * 0.96;   % drivetrain efficiency
    T_brake_whl = brk_i * P.tyre.mu_peak * P.m * 9.81 * P.R_w / P.n_drive * P.n_drive;
    dom_whl     = (T_drive_whl - Fx_tot*P.R_w - T_brake_whl) / P.Iz_w;

    % Propshaft angle rate
    dth_prop = om_eng - om_whl * i_total;

    % Chassis longitudinal EOM: m*dv/dt = Fx - F_aero - F_roll - F_grade - F_brake
    F_aero  = 0.5 * P.rho * P.Cd * P.A_f * v_veh^2;
    F_roll  = P.Cr * P.m * 9.81 * cos(grade_i * pi/180);
    F_grade = P.m * 9.81 * sin(grade_i * pi/180);
    F_brk_b = brk_i * P.tyre.mu_peak * P.m * 9.81;
    dv_veh  = (Fx_tot - F_aero - F_roll - F_grade - F_brk_b) / P.m;

    dxdt = [dv_veh; dom_eng; dom_whl; dth_prop; err; err];
end

function g = gear_select(rpm)
%GEAR_SELECT  Simple RPM-threshold AMT gear selection.
    if     rpm < 850,  g = 1;
    elseif rpm < 1100, g = 2;
    elseif rpm < 1350, g = 3;
    elseif rpm < 1650, g = 4;
    elseif rpm < 1950, g = 5;
    else,              g = 6;
    end
end
