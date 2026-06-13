%% run_truck_sim.m  –  Run the H6 Truck System simulation & auto-plot results
%  Simulates a 0→80 km/h acceleration, cruise, and braking event with
%  full FEAD parasitic coupling, AMT gear shifts, and road load.
%
%  Usage:  >> FEAD_params; run_truck_sim
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('engine','var'), FEAD_params; end

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('  H6 Truck System Simulation  –  Ashok Leyland H6\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

%% ── Run ODE-based truck model (no Simulink needed for quick run) ─────────────
%  State vector: x = [v_veh; omega_eng; omega_wheel; theta_prop; gear]
%
%  Full Simulink model available via:  build_truck_model; sim('H6_Truck_System')
%  This script implements the same physics as a fast MATLAB ODE integration.

%% ─── Parameters ─────────────────────────────────────────────────────────────
m     = vehicle.GVW_kg;
Cd    = vehicle.Cd;
A_f   = vehicle.frontal_area;
rho   = vehicle.rho_air;
Cr    = tyre.rolling_res;
R_w   = tyre.R_loaded_m;
Iz_w  = tyre.Iz * n_drive_wheels;
Iz_e  = engine.flywheel_Iz;
k_s   = driveline.propshaft_k;
c_s   = driveline.propshaft_c;

gear_ratios = trans.ratios;
i_fd        = trans.final_drive;

%% ─── Drive cycle reference ───────────────────────────────────────────────────
t_ref  = [0   5   15   35   70  100  115  120];
v_ref  = [0   0   20   80   80    0    0    0] / 3.6;   % m/s
gr_ref = [0   0    0    0    0    5    0    0];          % road grade deg

%% ─── PID Driver gains ────────────────────────────────────────────────────────
Kp = 1200; Ki = 180; Kd = 40;

%% ─── Initial conditions ──────────────────────────────────────────────────────
x0 = [0;                                 % v_veh  [m/s]
      engine.idle_rpm*2*pi/60;           % omega_eng [rad/s]
      0;                                 % omega_wheel [rad/s]
      0;                                 % theta_propshaft [rad]
      0;                                 % PID integral
      0];                                % PID derivative state

gear_state    = 1;   % current gear (not in ODE state — handled discretely)
t_lastshift   = -2;  % time of last shift

%% ─── ODE event logging ────────────────────────────────────────────────────────
T_log     = [];
X_log     = [];
gear_log  = [];
throttle_log = [];
brake_log    = [];

%% ─── Integration ─────────────────────────────────────────────────────────────
fprintf('Integrating truck ODE (%.0f s, dt=%.4f s)...\n', sim.T_end_truck, sim.dt);

dt    = sim.dt * 10;   % 1ms for speed (ODE45 adaptive)
t_now = 0;
x_now = x0;

% Use ode45 with events
opts  = odeset('RelTol',1e-4,'AbsTol',1e-6,'MaxStep',0.05);

t_span = [0 sim.T_end_truck];
[T_sol, X_sol] = ode45(@truck_ode, t_span, x0, opts);

% Extract logged data
v_veh_sol     = X_sol(:,1);            % m/s
omega_eng_sol = X_sol(:,2);            % rad/s
rpm_eng_sol   = omega_eng_sol * 60/(2*pi);
omega_whl_sol = X_sol(:,3);            % rad/s
theta_prop    = X_sol(:,4);            % rad

% Reconstruct gear, throttle, accel, torques
n_t    = numel(T_sol);
gear_v = zeros(n_t,1);
thr_v  = zeros(n_t,1);
brk_v  = zeros(n_t,1);
T_eng_v= zeros(n_t,1);
P_fead_v=zeros(n_t,1);
v_kmh  = v_veh_sol * 3.6;

for i = 1:n_t
    t_i   = T_sol(i);
    v_i   = v_veh_sol(i);
    om_i  = omega_eng_sol(i);
    rpm_i = om_i * 60/(2*pi);

    v_tgt = interp1(t_ref, v_ref, t_i, 'linear', 'extrap');
    err   = v_tgt - v_i;
    thr_i = max(0, min(1, Kp*err/m));   % simple P gain
    brk_i = max(0, min(1,-Kp*err/(tyre.mu_peak*m*9.81)));

    % Gear
    g_i = max(1, min(6, gear_from_rpm(rpm_i)));
    gear_v(i)  = g_i;
    thr_v(i)   = thr_i;
    brk_v(i)   = brk_i;

    % Engine torque
    T_eng_v(i) = interp2(engine.torque_thr, engine.torque_rpm,...
        engine.torque_map', thr_i, rpm_i, 'linear', 0);

    % FEAD loss
    v_b = max(pulleys(1).r/1000 * om_i, 0.01);
    P_f = 0;
    pnames_l = {'CRK','FAN','IDR','ALT','AC','TEN'};
    for k = 1:6
        P_f = P_f + max(interp1(load_table.rpm, load_table.(pnames_l{k}), rpm_i,'linear','extrap'),0);
    end
    P_fead_v(i) = P_f;
end

%% ─── Print summary ────────────────────────────────────────────────────────────
fprintf('\n--- Truck Simulation Summary ---\n');
[v_max, idx_vm] = max(v_kmh);
fprintf('  Max speed reached:  %.1f km/h at t=%.1f s\n', v_max, T_sol(idx_vm));
% 0-60 km/h time
idx_60 = find(v_kmh >= 60, 1, 'first');
if ~isempty(idx_60)
    fprintf('  0-60 km/h time:     %.1f s\n', T_sol(idx_60));
end
idx_80 = find(v_kmh >= 80, 1, 'first');
if ~isempty(idx_80)
    fprintf('  0-80 km/h time:     %.1f s\n', T_sol(idx_80));
end
fprintf('  Max engine RPM:     %.0f RPM\n', max(rpm_eng_sol));
fprintf('  Max gear reached:   %d\n', max(gear_v));
fprintf('  Avg FEAD loss:      %.1f kW\n', mean(P_fead_v));
fprintf('  Peak FEAD loss:     %.1f kW\n', max(P_fead_v));

%% ─── Plots ────────────────────────────────────────────────────────────────────
BG = [0.07 0.09 0.15];
TC = [0.89 0.91 0.94];
AX = [0.10 0.14 0.22];
AM = [0.96 0.62 0.04];
VI = [0.65 0.54 0.98];
GR = [0.20 0.85 0.60];
RD = [0.94 0.27 0.27];
PK = [0.96 0.28 0.71];

fig_t = figure('Color',BG,'Position',[80 60 1550 860],...
    'Name','H6 Truck Simulation Results');

%% 1 – Vehicle speed & reference
ax1 = subplot(3,3,1,'Parent',fig_t);
set(ax1,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax1,'on');
v_ref_plot = interp1(t_ref, v_ref, T_sol,'linear','extrap') * 3.6;
plot(ax1, T_sol, v_ref_plot, '--','Color',[TC 0.5],'LineWidth',1.5,'DisplayName','Reference');
plot(ax1, T_sol, v_kmh, 'Color',AM,'LineWidth',2.5,'DisplayName','Actual');
xlabel(ax1,'Time (s)','Color',TC); ylabel(ax1,'Speed (km/h)','Color',TC);
title(ax1,'Vehicle Speed','Color',AM,'FontWeight','bold');
legend(ax1,'TextColor',TC,'Color',AX); grid(ax1,'on');

%% 2 – Engine RPM
ax2 = subplot(3,3,2,'Parent',fig_t);
set(ax2,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
plot(T_sol, rpm_eng_sol, 'Color',VI,'LineWidth',2);
yline(ax2, engine.rated_rpm,'--','Color',RD,'LineWidth',1.5,'Label','Rated');
yline(ax2, engine.idle_rpm, '--','Color',[TC 0.4],'LineWidth',1,'Label','Idle');
xlabel(ax2,'Time (s)','Color',TC); ylabel(ax2,'RPM','Color',TC);
title(ax2,'Engine Speed','Color',AM,'FontWeight','bold'); grid(ax2,'on');

%% 3 – Gear
ax3 = subplot(3,3,3,'Parent',fig_t);
set(ax3,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
stairs(T_sol, gear_v,'Color',GR,'LineWidth',2.5);
ylabel(ax3,'Gear','Color',TC); xlabel(ax3,'Time (s)','Color',TC);
title(ax3,'AMT Gear Position','Color',AM,'FontWeight','bold');
yticks(ax3,1:6); ylim(ax3,[0 7]); grid(ax3,'on');

%% 4 – Engine torque
ax4 = subplot(3,3,4,'Parent',fig_t);
set(ax4,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
plot(T_sol, T_eng_v,'Color',AM,'LineWidth',2);
yline(ax4,engine.max_torque_Nm,'--','Color',RD,'LineWidth',1.5,'Label','Max Torque');
xlabel(ax4,'Time (s)','Color',TC); ylabel(ax4,'Torque (Nm)','Color',TC);
title(ax4,'Engine Output Torque','Color',AM,'FontWeight','bold'); grid(ax4,'on');

%% 5 – FEAD parasitic power
ax5 = subplot(3,3,5,'Parent',fig_t);
set(ax5,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
area(T_sol, P_fead_v,'FaceColor',[PK 0.25],'EdgeColor',PK,'LineWidth',2);
xlabel(ax5,'Time (s)','Color',TC); ylabel(ax5,'Power (kW)','Color',TC);
title(ax5,'FEAD Parasitic Loss','Color',AM,'FontWeight','bold'); grid(ax5,'on');

%% 6 – Throttle & Brake
ax6 = subplot(3,3,6,'Parent',fig_t);
set(ax6,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax6,'on');
plot(T_sol,thr_v*100,'Color',GR,'LineWidth',2,'DisplayName','Throttle %');
plot(T_sol,brk_v*100,'Color',RD,'LineWidth',2,'DisplayName','Brake %');
xlabel(ax6,'Time (s)','Color',TC); ylabel(ax6,'%','Color',TC);
title(ax6,'Throttle / Brake Demand','Color',AM,'FontWeight','bold');
legend(ax6,'TextColor',TC,'Color',AX); grid(ax6,'on');

%% 7 – Wheel speed vs engine (slip)
ax7 = subplot(3,3,7,'Parent',fig_t);
set(ax7,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax7,'on');
plot(T_sol, omega_whl_sol*R_w*3.6,'Color',VI,'LineWidth',2,'DisplayName','Wheel v (km/h)');
plot(T_sol, v_kmh,'Color',AM,'LineWidth',1.5,'LineStyle','--','DisplayName','Body v (km/h)');
xlabel(ax7,'Time (s)','Color',TC); ylabel(ax7,'Speed (km/h)','Color',TC);
title(ax7,'Wheel vs Body Speed (Slip Check)','Color',AM,'FontWeight','bold');
legend(ax7,'TextColor',TC,'Color',AX); grid(ax7,'on');

%% 8 – Power budget
ax8 = subplot(3,3,8,'Parent',fig_t);
set(ax8,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
P_eng_total = T_eng_v .* omega_eng_sol / 1000;  % kW
P_traction  = P_eng_total - P_fead_v;
hold(ax8,'on');
area(T_sol, P_eng_total,'FaceColor',[AM 0.2],'EdgeColor',AM,'LineWidth',2,'DisplayName','Engine');
area(T_sol, P_traction, 'FaceColor',[GR 0.2],'EdgeColor',GR,'LineWidth',2,'DisplayName','Traction');
area(T_sol, P_fead_v,   'FaceColor',[PK 0.2],'EdgeColor',PK,'LineWidth',2,'DisplayName','FEAD loss');
xlabel(ax8,'Time (s)','Color',TC); ylabel(ax8,'Power (kW)','Color',TC);
title(ax8,'Power Budget','Color',AM,'FontWeight','bold');
legend(ax8,'TextColor',TC,'Color',AX); grid(ax8,'on');

%% 9 – v-t with gear annotations
ax9 = subplot(3,3,9,'Parent',fig_t);
set(ax9,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5,'Box','on');
hold(ax9,'on');
for g = 1:6
    mask = gear_v == g;
    if any(mask)
        scatter(ax9,T_sol(mask),v_kmh(mask),6,'filled',...
            'MarkerFaceColor',hsv2rgb([g/8 0.9 0.95]),'DisplayName',sprintf('G%d',g));
    end
end
xlabel(ax9,'Time (s)','Color',TC); ylabel(ax9,'Speed (km/h)','Color',TC);
title(ax9,'Speed Trace coloured by Gear','Color',AM,'FontWeight','bold');
legend(ax9,'TextColor',TC,'Color',AX,'NumColumns',3); grid(ax9,'on');

sgtitle(fig_t,'H6 Truck System — Full Vehicle Simulation',...
    'Color',AM,'FontSize',15,'FontWeight','bold');

fprintf('\nPlots generated. Results in workspace: T_sol, X_sol, rpm_eng_sol, gear_v\n');

%% ─── LOCAL ODE FUNCTION ──────────────────────────────────────────────────────
function dxdt = truck_ode(t, x)
    v_veh   = x(1);
    om_eng  = x(2);
    om_whl  = x(3);
    th_prop = x(4);
    int_err = x(5);
    prev_err= x(6);

    % Road conditions
    grade_i = interp1(t_ref, gr_ref, t,'linear','extrap');
    v_tgt   = interp1(t_ref, v_ref,  t,'linear','extrap');
    err     = v_tgt - v_veh;

    % PID throttle/brake
    int_err_new = int_err + err * 0.05;
    derr        = (err - prev_err) / 0.05;
    pid_out     = Kp*err + Ki*int_err_new + Kd*derr;
    thr_i       = max(0, min(1,  pid_out / m));
    brk_i       = max(0, min(1, -pid_out / (tyre.mu_peak*m*9.81)));

    % Gear selection
    rpm_i   = max(om_eng * 60/(2*pi), 0);
    gear_i  = gear_from_rpm(rpm_i);
    i_total = gear_ratios(gear_i) * i_fd;

    % Engine torque
    T_eng = interp2(engine.torque_thr, engine.torque_rpm,...
        engine.torque_map', thr_i, rpm_i,'linear',0);

    % FEAD torque loss
    v_b = max(pulleys(1).r/1000 * om_eng, 0.01);
    P_f = 0;
    pnames_l = {'CRK','FAN','IDR','ALT','AC','TEN'};
    for kk = 1:6
        P_f = P_f + max(interp1(load_table.rpm, load_table.(pnames_l{kk}), rpm_i,'linear','extrap'),0);
    end
    T_fead = P_f*1000 / max(om_eng, 0.1);
    T_friction = engine.friction_Nm;

    % Propshaft torque
    om_prop_expected = om_whl * i_total;
    T_shaft = k_s * th_prop + c_s * (om_eng - om_prop_expected);

    % Engine rotational EOM
    dom_eng = (T_eng - T_fead - T_friction - T_shaft/i_total) / Iz_e;

    % Wheel rotational EOM (Pacejka)
    kappa_i = (om_whl*R_w - max(v_veh,0.001)) / max(v_veh,0.001);
    kappa_i = max(-1, min(1, kappa_i));
    Fz_w    = m*9.81/n_drive_wheels;
    F_x     = Fz_w * tyre.D * sin(tyre.C * atan(tyre.B*kappa_i - tyre.E*(tyre.B*kappa_i - atan(tyre.B*kappa_i))));
    F_x_tot = F_x * n_drive_wheels;
    T_wheel_in = T_shaft * i_total * 0.96;   % 4% drivetrain efficiency loss
    T_brake     = brk_i * tyre.mu_peak * m * 9.81 * R_w;
    dom_whl     = (T_wheel_in - F_x_tot*R_w - T_brake) / Iz_w;

    % Propshaft angle
    dth_prop = om_eng - om_whl * i_total;

    % Chassis (longitudinal)
    F_aero  = 0.5*rho*Cd*A_f*v_veh^2;
    F_roll  = Cr*m*9.81*cos(grade_i*pi/180);
    F_grade = m*9.81*sin(grade_i*pi/180);
    F_brake_body = brk_i * tyre.mu_peak * m * 9.81;
    dv_veh  = (F_x_tot - F_aero - F_roll - F_grade - F_brake_body) / m;

    dxdt = [dv_veh; dom_eng; dom_whl; dth_prop; err; err];
end

function g = gear_from_rpm(rpm)
    if     rpm < 850,  g = 1;
    elseif rpm < 1100, g = 2;
    elseif rpm < 1350, g = 3;
    elseif rpm < 1650, g = 4;
    elseif rpm < 1950, g = 5;
    else,              g = 6;
    end
end
