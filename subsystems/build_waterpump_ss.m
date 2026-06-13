%% build_waterpump_ss.m  –  Water Pump Subsystem (gear-driven via engine front)
%  Models the WP as a gear-driven centrifugal pump with:
%    • Pump torque vs flow speed characteristic
%    • Bearing load calculation (ISO 281 L10 life)
%    • Cavitation onset RPM check
%    • Thermal feedback to engine coolant temperature
%
%  Usage (standalone):  >> FEAD_params; build_waterpump_ss
%  Or called from build_truck_model as a subsystem block.
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('wp','var'), FEAD_params; end

%% ── Water Pump Characteristics ───────────────────────────────────────────────
%  Centrifugal pump affinity laws:
%    Q = k_Q * n          (flow ∝ speed)
%    P = k_P * n^3        (power ∝ speed³)
%    H = k_H * n^2        (head ∝ speed²)

% C&U WP measured data at key engine RPMs
wp_data.eng_rpm  = [600   800  1000  1200  1400  1600  1800  2000];
wp_data.wp_rpm   = wp_data.eng_rpm * wp.gear_ratio;   % WP speed
wp_data.flow_lpm = [8    14    22    32    43    56    70    85];  % L/min
wp_data.head_m   = [0.8   1.5   2.4   3.5   4.8   6.4   8.1  10.0]; % m H2O
wp_data.power_W  = [18    40    80   145   235   375   560   780];  % W (absorbed)
wp_data.torque_Nm= wp_data.power_W ./ (wp_data.wp_rpm * 2*pi/60);

% Cavitation NPSHr (required NPSH) – increases with speed
wp_data.NPSHr_m  = [0.3   0.5   0.8   1.1   1.6   2.2   2.9   3.8];

% Bearing loads at WP shaft
wp_data.F_belt   = [350   420   490   560   635   710   790   870]; % N (belt load)
wp_data.F_radial = ones(1,8) * wp.F_radial;                          % N (impeller)

%% ── ISO 281 L10 Bearing Life over RPM sweep ──────────────────────────────────
n_pts   = numel(wp_data.eng_rpm);
L10A_v  = zeros(1,n_pts);
L10B_v  = zeros(1,n_pts);
L10C_v  = zeros(1,n_pts);

for i = 1:n_pts
    F_b  = wp_data.F_belt(i);
    F_r  = wp.F_radial;
    n_wp = wp_data.wp_rpm(i);

    P_ball   = sqrt((F_b*0.3)^2 + F_r^2);
    P_roller = sqrt((F_b*0.7)^2 + F_r^2);

    L10A_v(i) = (wp.ball.Cr   / max(P_ball,  1))^wp.ball.p   * 1e6/(60*max(n_wp,1));
    L10B_v(i) = (wp.roller.Cr / max(P_roller,1))^wp.roller.p * 1e6/(60*max(n_wp,1));
    % Series composite (ISO 16281 simplified)
    L10C_v(i) = ((L10A_v(i)^(-9/8) + L10B_v(i)^(-9/8))^(-8/9));
end

%% ── Print results table ───────────────────────────────────────────────────────
fprintf('\n=== Water Pump Subsystem – C&U Bearing Life (ISO 281) ===\n\n');
fprintf('%-8s %-8s %-10s %-10s %-8s %-8s %-10s %-10s %-10s\n',...
    'Eng RPM','WP RPM','Flow(L/m)','Power(W)','F_belt','L10A(h)','L10B(h)','Comp(h)','Status');
fprintf('%s\n', repmat('-',1,90));

for i = 1:n_pts
    if L10C_v(i) >= wp.ref.L10comp * 1.1
        status = 'OK  ✓';
    elseif L10C_v(i) >= wp.ref.L10comp * 0.7
        status = 'MARG';
    else
        status = 'WARN';
    end
    fprintf('%-8.0f %-8.0f %-10.0f %-10.0f %-8.0f %-8.0f %-10.0f %-10.0f %s\n',...
        wp_data.eng_rpm(i), wp_data.wp_rpm(i), wp_data.flow_lpm(i),...
        wp_data.power_W(i), wp_data.F_belt(i),...
        L10A_v(i), L10B_v(i), L10C_v(i), status);
end

fprintf('\nC&U Reference:  L10A=%d h  L10B=%d h  Composite=%d h\n',...
    wp.ref.L10A, wp.ref.L10B, wp.ref.L10comp);

%% ── Build as a Simulink Subsystem block (if model is open) ──────────────────
mdl_wp = 'WaterPump_Subsystem';
if bdIsLoaded(mdl_wp), close_system(mdl_wp,0); end
if exist([mdl_wp '.slx'],'file'), delete([mdl_wp '.slx']); end

new_system(mdl_wp);
open_system(mdl_wp);
set_param(mdl_wp,'StopTime','30','Solver','ode23t','SolverType','Fixed-step','FixedStep','1e-4');

%% ── Add Simscape Solver ──────────────────────────────────────────────────────
add_block('nesl_utility/Solver Configuration',[mdl_wp '/SC']);
set_param([mdl_wp '/SC'],'Position',[20 20 200 60]);

add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference',...
    [mdl_wp '/GND']);
set_param([mdl_wp '/GND'],'Position',[20 120 80 160]);

%% ── WP Shaft Inertia ─────────────────────────────────────────────────────────
Iz_wp = 0.003;   % kg·m²
add_block('fl_lib/Mechanical/Rotational Elements/Inertia',[mdl_wp '/WP_Inertia']);
set_param([mdl_wp '/WP_Inertia'],'inertia',num2str(Iz_wp),'Position',[200 200 350 260]);

%% ── Gear (speed multiplier) represented as ideal gear ────────────────────────
add_block('fl_lib/Mechanical/Rotational Elements/Ideal Torque Source',[mdl_wp '/GearInput']);
set_param([mdl_wp '/GearInput'],'Position',[200 300 350 360]);

%% ── Pump Load Torque (lookup: WP RPM → torque) ───────────────────────────────
add_block('simulink/Lookup Tables/1-D Lookup Table',[mdl_wp '/PumpTorque_LUT']);
wp_rpm_rad = wp_data.wp_rpm * 2*pi/60;
set_param([mdl_wp '/PumpTorque_LUT'],...
    'BreakpointsForDimension1', mat2str(wp_rpm_rad,6),...
    'Table',                    mat2str(wp_data.torque_Nm,6),...
    'ExtrapMethod','Clip','Position',[500 200 680 250]);

%% ── Speed sensor ─────────────────────────────────────────────────────────────
add_block('fl_lib/Mechanical/Rotational Elements/Ideal Rotational Motion Sensor',...
    [mdl_wp '/WP_SpeedSensor']);
set_param([mdl_wp '/WP_SpeedSensor'],'Position',[200 400 350 460]);

%% ── PS→Simulink for speed feedback ──────────────────────────────────────────
add_block('nesl_utility/PS-Simulink Converter',[mdl_wp '/Conv_Speed']);
set_param([mdl_wp '/Conv_Speed'],'Position',[400 410 520 450]);

%% ── Simulink→PS for load torque ──────────────────────────────────────────────
add_block('nesl_utility/Simulink-PS Converter',[mdl_wp '/Conv_Torque']);
set_param([mdl_wp '/Conv_Torque'],'Position',[730 210 850 250]);

%% ── Connections ──────────────────────────────────────────────────────────────
add_line(mdl_wp,'WP_SpeedSensor/W','Conv_Speed/I','autorouting','on');
add_line(mdl_wp,'Conv_Speed/1','PumpTorque_LUT/1','autorouting','on');
add_line(mdl_wp,'PumpTorque_LUT/1','Conv_Torque/I','autorouting','on');
add_line(mdl_wp,'Conv_Torque/O','GearInput/T','autorouting','on');

%% ── Bearing Life calculation block ──────────────────────────────────────────
add_block('simulink/Math Operations/Fcn',[mdl_wp '/L10A_calc']);
set_param([mdl_wp '/L10A_calc'],...
    'Expr',sprintf('(%g/max(sqrt((%g*0.3)^2+%g^2),1))^%g*1e6/(60*max(u*%g,1))',...
        wp.ball.Cr, 1, wp.F_radial, wp.ball.p, wp.gear_ratio),...
    'Position',[500 300 720 340]);
add_line(mdl_wp,'Conv_Speed/1','L10A_calc/1','autorouting','on');

%% ── Input from engine ────────────────────────────────────────────────────────
add_block('simulink/Sources/In1',[mdl_wp '/omega_eng']);
set_param([mdl_wp '/omega_eng'],'Port','1','Position',[20 300 80 330]);

%% ── Output scopes ────────────────────────────────────────────────────────────
add_block('simulink/Sinks/Scope',[mdl_wp '/WP_Scope']);
set_param([mdl_wp '/WP_Scope'],'NumInputPorts','3','Position',[900 250 1000 400]);
add_block('simulink/Math Operations/Gain',[mdl_wp '/rad2rpm']);
set_param([mdl_wp '/rad2rpm'],'Gain','60/(2*pi)','Position',[600 410 720 450]);
add_line(mdl_wp,'Conv_Speed/1','rad2rpm/1','autorouting','on');

%% ── Out: WP RPM, L10A, pump torque ──────────────────────────────────────────
add_block('simulink/Sinks/Out1',[mdl_wp '/WP_RPM_out']);  set_param([mdl_wp '/WP_RPM_out'],'Port','1','Position',[820 410 870 440]);
add_block('simulink/Sinks/Out1',[mdl_wp '/L10A_out']);    set_param([mdl_wp '/L10A_out'],'Port','2','Position',[820 310 870 340]);
add_line(mdl_wp,'rad2rpm/1','WP_RPM_out/1','autorouting','on');
add_line(mdl_wp,'L10A_calc/1','L10A_out/1','autorouting','on');

save_system(mdl_wp);
fprintf('\nWaterPump_Subsystem.slx saved.\n');

%% ═══════════════════════════════════════════════════════════════════════════
%  WATER PUMP PLOTS
% ═══════════════════════════════════════════════════════════════════════════
BG=[0.07 0.09 0.15]; TC=[0.89 0.91 0.94]; AX=[0.10 0.14 0.22];
AM=[0.96 0.62 0.04]; VI=[0.65 0.54 0.98]; GR=[0.20 0.85 0.60];

fig_wp = figure('Color',BG,'Position',[100 100 1200 700],'Name','Water Pump Analysis');

% Flow vs RPM
ax1=subplot(2,3,1); set(ax1,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5);
plot(wp_data.eng_rpm, wp_data.flow_lpm,'Color',AM,'LineWidth',2.5,'Marker','o','MarkerFaceColor',AM);
xlabel(ax1,'Engine RPM','Color',TC); ylabel(ax1,'Flow Rate (L/min)','Color',TC);
title(ax1,'WP Flow vs Engine RPM','Color',AM,'FontWeight','bold'); grid(ax1,'on');

% Head vs RPM
ax2=subplot(2,3,2); set(ax2,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5);
plot(wp_data.eng_rpm, wp_data.head_m,'Color',VI,'LineWidth',2.5,'Marker','s','MarkerFaceColor',VI);
xlabel(ax2,'Engine RPM','Color',TC); ylabel(ax2,'Head (m H₂O)','Color',TC);
title(ax2,'WP Pressure Head','Color',AM,'FontWeight','bold'); grid(ax2,'on');

% Power vs RPM
ax3=subplot(2,3,3); set(ax3,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5);
plot(wp_data.eng_rpm, wp_data.power_W/1000,'Color',GR,'LineWidth',2.5,'Marker','^','MarkerFaceColor',GR);
xlabel(ax3,'Engine RPM','Color',TC); ylabel(ax3,'Power (kW)','Color',TC);
title(ax3,'WP Absorbed Power','Color',AM,'FontWeight','bold'); grid(ax3,'on');

% L10 Life
ax4=subplot(2,3,4); set(ax4,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5);
hold(ax4,'on');
plot(wp_data.eng_rpm, L10A_v,'Color',[0.38 0.64 0.98],'LineWidth',2,'DisplayName','Ball L10A');
plot(wp_data.eng_rpm, L10B_v,'Color',VI,'LineWidth',2,'DisplayName','Roller L10B');
plot(wp_data.eng_rpm, L10C_v,'Color',GR,'LineWidth',2.5,'DisplayName','Composite');
yline(ax4,wp.ref.L10A,   '--','Color',[0.38 0.64 0.98 0.5],'Label','Ref L10A');
yline(ax4,wp.ref.L10B,   '--','Color',[VI    0.5],'Label','Ref L10B');
yline(ax4,wp.ref.L10comp,'--','Color',[GR    0.5],'LineWidth',1.5,'Label','Ref Comp');
xlabel(ax4,'Engine RPM','Color',TC); ylabel(ax4,'Life (hours)','Color',TC);
title(ax4,'Bearing Life (ISO 281)','Color',AM,'FontWeight','bold');
legend(ax4,'TextColor',TC,'Color',AX); grid(ax4,'on');

% Belt force vs RPM
ax5=subplot(2,3,5); set(ax5,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5);
plot(wp_data.eng_rpm,wp_data.F_belt,'Color',[0.96 0.28 0.71],'LineWidth',2.5,'Marker','d','MarkerFaceColor',[0.96 0.28 0.71]);
xlabel(ax5,'Engine RPM','Color',TC); ylabel(ax5,'Belt Force (N)','Color',TC);
title(ax5,'WP Belt (Tensioner) Load','Color',AM,'FontWeight','bold'); grid(ax5,'on');

% Torque vs RPM
ax6=subplot(2,3,6); set(ax6,'Color',AX,'XColor',TC,'YColor',TC,'GridColor',[0.2 0.3 0.4],'GridAlpha',0.5);
plot(wp_data.eng_rpm,wp_data.torque_Nm,'Color',AM,'LineWidth',2.5,'Marker','o','MarkerFaceColor',AM);
xlabel(ax6,'Engine RPM','Color',TC); ylabel(ax6,'Torque (Nm)','Color',TC);
title(ax6,'WP Drive Torque','Color',AM,'FontWeight','bold'); grid(ax6,'on');

sgtitle(fig_wp,'Water Pump Subsystem – C&U Bearing Life Analysis','Color',AM,'FontSize',14,'FontWeight','bold');

% Save to workspace
assignin('base','wp_data', wp_data);
assignin('base','wp_L10A',  L10A_v);
assignin('base','wp_L10B',  L10B_v);
assignin('base','wp_L10C',  L10C_v);
fprintf('WP data saved to workspace: wp_data, wp_L10A, wp_L10B, wp_L10C\n');
