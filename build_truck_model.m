%% build_truck_model.m  –  Complete Ashok Leyland H6 Truck System
%  Builds a full-vehicle Simulink model including Engine, FEAD, Transmission,
%  Driveline, Wheels, Chassis, Brakes, and a PID Driver model.
%
%  REQUIRES:  Simulink, Simscape (Foundation Library)
%  Usage:
%    >> FEAD_params
%    >> build_truck_model       % creates H6_Truck_System.slx
%    >> sim('H6_Truck_System')
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('engine','var'), FEAD_params; end

mdl = 'H6_Truck_System';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end

new_system(mdl);
open_system(mdl);
set_param(mdl,...
    'SolverType','Fixed-step','FixedStep','1e-4',...
    'Solver','ode4','StopTime',num2str(sim.T_end_truck),...
    'SaveTime','on','SaveOutput','on',...
    'SignalLogging','on','SignalLoggingName','logsout');

%% ─────────────────────────────────────────────────────────────────────────
%  TOP-LEVEL SUBSYSTEMS  (placed as masked subsystem blocks)
% ─────────────────────────────────────────────────────────────────────────
%  Layout (canvas positions):
%  Driver → Engine → FEAD → Transmission → Driveline → Wheels → Chassis
%                                                                   ↓ v_veh
%                                                         (feedback to Driver)

ss_list = {'Driver','Engine','FEAD','Transmission','Driveline','Wheels','Chassis','Brakes'};
x_pos   = [50 350 650 950 1250 1550 1850 2150];
y_pos   = 300;

for i = 1:numel(ss_list)
    nm = ss_list{i};
    blk = [mdl '/' nm];
    add_block('simulink/Ports & Subsystems/Subsystem', blk);
    set_param(blk,'Position',[x_pos(i) y_pos x_pos(i)+200 y_pos+120]);
    set_param(blk,'BackgroundColor','lightBlue');
    set_param(blk,'ForegroundColor','black');
end

%% ─────────────────────────────────────────────────────────────────────────
%  DRIVER SUBSYSTEM  (PID speed controller)
% ─────────────────────────────────────────────────────────────────────────
build_driver_ss([mdl '/Driver'], sim);

%% ─────────────────────────────────────────────────────────────────────────
%  ENGINE SUBSYSTEM  (Torque map + friction + idle governor)
% ─────────────────────────────────────────────────────────────────────────
build_engine_ss([mdl '/Engine'], engine, sim);

%% ─────────────────────────────────────────────────────────────────────────
%  FEAD SUBSYSTEM  (Belt drive parasitic losses fed back to engine)
% ─────────────────────────────────────────────────────────────────────────
build_fead_ss([mdl '/FEAD'], pulleys, belt, load_table, ten, ten_idx, ten_pos);

%% ─────────────────────────────────────────────────────────────────────────
%  TRANSMISSION SUBSYSTEM  (AMT 6-speed)
% ─────────────────────────────────────────────────────────────────────────
build_transmission_ss([mdl '/Transmission'], trans, sim);

%% ─────────────────────────────────────────────────────────────────────────
%  DRIVELINE SUBSYSTEM  (propshaft + differential)
% ─────────────────────────────────────────────────────────────────────────
build_driveline_ss([mdl '/Driveline'], driveline);

%% ─────────────────────────────────────────────────────────────────────────
%  WHEELS SUBSYSTEM  (Pacejka tire × 6)
% ─────────────────────────────────────────────────────────────────────────
build_wheel_ss([mdl '/Wheels'], tyre, n_drive_wheels);

%% ─────────────────────────────────────────────────────────────────────────
%  CHASSIS SUBSYSTEM  (longitudinal + pitch DOF)
% ─────────────────────────────────────────────────────────────────────────
build_chassis_ss([mdl '/Chassis'], vehicle);

%% ─────────────────────────────────────────────────────────────────────────
%  BRAKES SUBSYSTEM
% ─────────────────────────────────────────────────────────────────────────
build_brakes_ss([mdl '/Brakes'], vehicle, tyre);

%% ─────────────────────────────────────────────────────────────────────────
%  TOP-LEVEL SIGNAL CONNECTIONS
% ─────────────────────────────────────────────────────────────────────────
% Driver → throttle, brake_demand → Engine, Brakes
add_line(mdl,'Driver/1','Engine/1','autorouting','on');   % throttle
add_line(mdl,'Driver/2','Brakes/1','autorouting','on');   % brake demand

% Engine → omega_crankshaft → FEAD + Transmission
add_line(mdl,'Engine/1','FEAD/1','autorouting','on');     % omega_crk
add_line(mdl,'Engine/1','Transmission/1','autorouting','on');

% FEAD → parasitic torque → Engine (negative feedback)
add_line(mdl,'FEAD/1','Engine/2','autorouting','on');     % T_fead_loss

% Transmission → omega_out → Driveline
add_line(mdl,'Transmission/1','Driveline/1','autorouting','on');
add_line(mdl,'Transmission/2','Driveline/2','autorouting','on'); % current_gear

% Driveline → T_axle, omega_axle → Wheels
add_line(mdl,'Driveline/1','Wheels/1','autorouting','on');
add_line(mdl,'Driveline/2','Wheels/2','autorouting','on');

% Wheels → F_traction, omega_wheel → Chassis
add_line(mdl,'Wheels/1','Chassis/1','autorouting','on');  % F_traction
add_line(mdl,'Wheels/2','Chassis/2','autorouting','on');  % aero state

% Brakes → F_brake → Chassis
add_line(mdl,'Brakes/2','Chassis/3','autorouting','on');

% Chassis → v_vehicle → Driver (feedback) + Wheels (slip calc)
add_line(mdl,'Chassis/1','Driver/1','autorouting','on');  % v_veh feedback
add_line(mdl,'Chassis/1','Wheels/3','autorouting','on');  % v_veh for slip
add_line(mdl,'Chassis/2','Engine/3','autorouting','on');  % engine_rpm feedback

%% ─────────────────────────────────────────────────────────────────────────
%  SPEED REFERENCE (From Workspace)
% ─────────────────────────────────────────────────────────────────────────
blk_vref = [mdl '/SpeedRef_kmh'];
add_block('simulink/Sources/From Workspace', blk_vref);
set_param(blk_vref,'Position',[50 500 200 540]);
% Drive cycle: 0→80 km/h in 30s, cruise, brake
t_vref = [0  10  30  60  90 100 120];
v_vref = [0   0  80  80  80   0   0];  % km/h
assignin('base','t_vref_data', timeseries(v_vref, t_vref));
set_param(blk_vref,'VariableName','t_vref_data','Interpolate','on');

blk_kmh2ms = [mdl '/kmh2ms'];
add_block('simulink/Math Operations/Gain', blk_kmh2ms);
set_param(blk_kmh2ms,'Gain','1/3.6','Position',[230 500 380 540]);
add_line(mdl,'SpeedRef_kmh/1','kmh2ms/1','autorouting','on');
add_line(mdl,'kmh2ms/1','Driver/2','autorouting','on');

%% ─────────────────────────────────────────────────────────────────────────
%  GRADE ANGLE INPUT
% ─────────────────────────────────────────────────────────────────────────
blk_grade = [mdl '/RoadGrade_deg'];
add_block('simulink/Sources/Constant', blk_grade);
set_param(blk_grade,'Value','0','Position',[50 580 180 610]);  % flat road default
add_line(mdl,'RoadGrade_deg/1','Chassis/4','autorouting','on');

%% ─────────────────────────────────────────────────────────────────────────
%  DASHBOARD SCOPE  (Vehicle speed, RPM, gear, FEAD load, fuel)
% ─────────────────────────────────────────────────────────────────────────
blk_dash = [mdl '/Dashboard'];
add_block('simulink/Sinks/Scope', blk_dash);
set_param(blk_dash,'Position',[2400 200 2550 500]);
set_param(blk_dash,'NumInputPorts','5',...
    'AxesTitles','v [km/h]\nEngine RPM\nGear\nFEAD Loss [kW]\nEngine Torque [Nm]');

% Vehicle speed (m/s → km/h)
blk_ms2km = [mdl '/ms2kmh'];
add_block('simulink/Math Operations/Gain', blk_ms2km);
set_param(blk_ms2km,'Gain','3.6','Position',[2200 200 2350 230]);
add_line(mdl,'Chassis/1','ms2kmh/1','autorouting','on');
add_line(mdl,'ms2kmh/1','Dashboard/1','autorouting','on');

add_line(mdl,'Engine/2','Dashboard/2','autorouting','on');   % RPM
add_line(mdl,'Transmission/3','Dashboard/3','autorouting','on'); % gear
add_line(mdl,'FEAD/2','Dashboard/4','autorouting','on');     % FEAD kW
add_line(mdl,'Engine/3','Dashboard/5','autorouting','on');   % Torque

%% ─────────────────────────────────────────────────────────────────────────
%  LOG TO WORKSPACE
% ─────────────────────────────────────────────────────────────────────────
blk_log = [mdl '/To Workspace'];
add_block('simulink/Sinks/To Workspace', blk_log);
set_param(blk_log,'Position',[2400 560 2580 600]);
set_param(blk_log,'VariableName','truck_results','SaveFormat','Array','MaxDataPoints','inf');

%% Save
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);
fprintf('\nH6_Truck_System.slx created and saved.\n');
fprintf('Run:  sim(''%s'')  to simulate.\n', mdl);
open_system(mdl);

%% ═════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM BUILDER FUNCTIONS  (called above)
%  Each builds the interior of a pre-created Subsystem block.
% ═════════════════════════════════════════════════════════════════════════

% ──────────────────────────────────────────────────────────────────────────
function build_driver_ss(blk, sim)
% PID speed controller → throttle (0-1) and brake demand (0-1)
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    % Inports: v_actual(1), v_reference(2)
    add_block('simulink/Sources/In1',[blk '/v_actual']); set_param([blk '/v_actual'],'Port','1','Position',[30 80 80 110]);
    add_block('simulink/Sources/In1',[blk '/v_ref']);    set_param([blk '/v_ref'],'Port','2','Position',[30 170 80 200]);

    % Speed error
    add_block('simulink/Math Operations/Sum',[blk '/err']);
    set_param([blk '/err'],'Inputs','+-','Position',[120 120 160 200]);
    add_line(blk,'v_ref/1','err/1','autorouting','on');
    add_line(blk,'v_actual/1','err/2','autorouting','on');

    % PID controller
    add_block('simulink/Continuous/PID Controller',[blk '/PID']);
    set_param([blk '/PID'],'P','0.8','I','0.3','D','0.05',...
        'N','20','Position',[200 120 350 200]);
    add_line(blk,'err/1','PID/1','autorouting','on');

    % Saturation → throttle [0,1]
    add_block('simulink/Discontinuities/Saturation',[blk '/Sat_thr']);
    set_param([blk '/Sat_thr'],'UpperLimit','1','LowerLimit','0','Position',[400 100 480 140]);
    add_line(blk,'PID/1','Sat_thr/1','autorouting','on');

    % Negative part → brake demand
    add_block('simulink/Math Operations/Gain',[blk '/neg']);
    set_param([blk '/neg'],'Gain','-1','Position',[400 180 460 220]);
    add_block('simulink/Discontinuities/Saturation',[blk '/Sat_brk']);
    set_param([blk '/Sat_brk'],'UpperLimit','1','LowerLimit','0','Position',[480 180 560 220]);
    add_line(blk,'PID/1','neg/1','autorouting','on');
    add_line(blk,'neg/1','Sat_brk/1','autorouting','on');

    % Outports
    add_block('simulink/Sinks/Out1',[blk '/throttle']); set_param([blk '/throttle'],'Port','1','Position',[580 110 630 140]);
    add_block('simulink/Sinks/Out1',[blk '/brake']);     set_param([blk '/brake'],'Port','2','Position',[580 190 630 220]);
    add_line(blk,'Sat_thr/1','throttle/1','autorouting','on');
    add_line(blk,'Sat_brk/1','brake/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_engine_ss(blk, engine, sim)
% Diesel engine: torque map (RPM×throttle) + flywheel inertia + friction
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    % Inports
    add_block('simulink/Sources/In1',[blk '/throttle']);  set_param([blk '/throttle'],'Port','1','Position',[30 60 80 90]);
    add_block('simulink/Sources/In1',[blk '/T_fead']);    set_param([blk '/T_fead'],'Port','2','Position',[30 160 80 190]);
    add_block('simulink/Sources/In1',[blk '/rpm_fb']);    set_param([blk '/rpm_fb'],'Port','3','Position',[30 260 80 290]);

    % 2D Lookup table (throttle × RPM → torque)
    add_block('simulink/Lookup Tables/2-D Lookup Table',[blk '/TorqueMap']);
    set_param([blk '/TorqueMap'],...
        'RowIndex',    mat2str(engine.torque_thr),...
        'ColumnIndex', mat2str(engine.torque_rpm),...
        'Table',       mat2str(engine.torque_map),...
        'Position',[150 60 350 160]);
    add_line(blk,'throttle/1','TorqueMap/1','autorouting','on');
    add_line(blk,'rpm_fb/1','TorqueMap/2','autorouting','on');

    % Engine friction (constant + speed-proportional)
    add_block('simulink/Math Operations/Gain',[blk '/fric_c']);
    set_param([blk '/fric_c'],'Gain',num2str(engine.friction_Nm*0.01),'Position',[150 270 280 300]);
    add_line(blk,'rpm_fb/1','fric_c/1','autorouting','on');

    add_block('simulink/Math Operations/Sum',[blk '/T_net']);
    set_param([blk '/T_net'],'Inputs','+--','Position',[400 100 450 200]);
    add_line(blk,'TorqueMap/1','T_net/1','autorouting','on');
    add_line(blk,'T_fead/1','T_net/2','autorouting','on');
    add_line(blk,'fric_c/1','T_net/3','autorouting','on');

    % Flywheel integrator:  omega = integral(T_net/Iz)
    add_block('simulink/Math Operations/Gain',[blk '/inv_Iz']);
    set_param([blk '/inv_Iz'],'Gain',num2str(1/engine.flywheel_Iz),'Position',[500 130 600 170]);
    add_block('simulink/Continuous/Integrator',[blk '/omega_int']);
    set_param([blk '/omega_int'],'InitialCondition',num2str(sim.rpm_init*2*pi/60),...
        'LowerSaturationLimit','0','UpperSaturationLimit','Inf','Position',[650 130 750 170]);
    add_line(blk,'T_net/1','inv_Iz/1','autorouting','on');
    add_line(blk,'inv_Iz/1','omega_int/1','autorouting','on');

    % omega → RPM
    add_block('simulink/Math Operations/Gain',[blk '/w2rpm']);
    set_param([blk '/w2rpm'],'Gain','60/(2*pi)','Position',[800 130 900 170]);
    add_line(blk,'omega_int/1','w2rpm/1','autorouting','on');

    % Outports: omega_crk(1), RPM(2), T_net(3)
    add_block('simulink/Sinks/Out1',[blk '/omega_crk']); set_param([blk '/omega_crk'],'Port','1','Position',[970 130 1020 160]);
    add_block('simulink/Sinks/Out1',[blk '/RPM_out']);   set_param([blk '/RPM_out'],'Port','2','Position',[970 175 1020 205]);
    add_block('simulink/Sinks/Out1',[blk '/T_out']);     set_param([blk '/T_out'],'Port','3','Position',[970 220 1020 250]);
    add_line(blk,'omega_int/1','omega_crk/1','autorouting','on');
    add_line(blk,'w2rpm/1','RPM_out/1','autorouting','on');
    add_line(blk,'T_net/1','T_out/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_fead_ss(blk, pulleys, belt, load_table, ten, ten_idx, ten_pos)
% FEAD parasitic power model: given omega_crk → total loss torque (kW)
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    add_block('simulink/Sources/In1',[blk '/omega_crk']); set_param([blk '/omega_crk'],'Port','1','Position',[30 130 80 160]);

    % Belt velocity
    r_CRK = pulleys(1).r/1000;
    add_block('simulink/Math Operations/Gain',[blk '/v_belt']);
    set_param([blk '/v_belt'],'Gain',num2str(r_CRK),'Position',[120 130 240 160]);
    add_line(blk,'omega_crk/1','v_belt/1','autorouting','on');

    % Centrifugal tension loss
    add_block('simulink/Math Operations/Math Function',[blk '/v2']);
    set_param([blk '/v2'],'Operator','square','Position',[290 130 390 160]);
    add_block('simulink/Math Operations/Gain',[blk '/T_c_loss']);
    set_param([blk '/T_c_loss'],'Gain',num2str(belt.lin_mass),'Position',[420 130 540 160]);
    add_line(blk,'v_belt/1','v2/1','autorouting','on');
    add_line(blk,'v2/1','T_c_loss/1','autorouting','on');

    % Sum accessory powers from lookup tables
    pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
    lut_blocks = {};
    for k = 1:numel(pnames)
        pn = pnames{k};
        b = [blk '/P_' pn];
        add_block('simulink/Lookup Tables/1-D Lookup Table', b);
        rpm_rad = load_table.rpm * 2*pi/60;
        P_arr   = load_table.(pn);
        set_param(b,'BreakpointsForDimension1',mat2str(rpm_rad,6),...
            'Table',mat2str(P_arr,6),'ExtrapMethod','Clip',...
            'Position',[120 (k-1)*60+220 280 (k-1)*60+250]);
        add_line(blk,'omega_crk/1',['P_' pn '/1'],'autorouting','on');
        lut_blocks{k} = b;
    end

    % Sum all accessory powers → total FEAD power (kW)
    add_block('simulink/Math Operations/Sum',[blk '/P_total']);
    set_param([blk '/P_total'],'Inputs','++++++','Position',[350 280 400 580]);
    for k = 1:numel(pnames)
        pn = pnames{k};
        add_line(blk,['P_' pn '/1'],sprintf('P_total/%d',k),'autorouting','on');
    end

    % Total FEAD torque loss = P_total*1000/omega
    add_block('simulink/Math Operations/Divide',[blk '/T_fead_total']);
    set_param([blk '/T_fead_total'],'Position',[500 400 620 450]);
    add_block('simulink/Math Operations/Gain',[blk '/kW2W']);
    set_param([blk '/kW2W'],'Gain','1000','Position',[430 400 490 430]);
    add_line(blk,'P_total/1','kW2W/1','autorouting','on');
    add_line(blk,'kW2W/1','T_fead_total/1','autorouting','on');
    add_line(blk,'omega_crk/1','T_fead_total/2','autorouting','on');

    % Outports: T_fead_loss(1), P_fead_kW(2)
    add_block('simulink/Sinks/Out1',[blk '/T_loss']); set_param([blk '/T_loss'],'Port','1','Position',[700 420 750 450]);
    add_block('simulink/Sinks/Out1',[blk '/P_kW']);   set_param([blk '/P_kW'],'Port','2','Position',[700 480 750 510]);
    add_line(blk,'T_fead_total/1','T_loss/1','autorouting','on');
    add_line(blk,'P_total/1','P_kW/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_transmission_ss(blk, trans, sim)
% AMT 6-speed: gear ratio lookup + shift logic + output speed
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    add_block('simulink/Sources/In1',[blk '/omega_eng']); set_param([blk '/omega_eng'],'Port','1','Position',[30 80 80 110]);

    % Current gear (MATLAB Function for shift logic)
    add_block('simulink/User-Defined Functions/MATLAB Function',[blk '/GearLogic']);
    set_param([blk '/GearLogic'],'Position',[150 60 380 220]);
    % Gear logic MATLAB code
    gear_code = sprintf([...
        'function [gear, ratio] = GearLogic(omega_eng)\n',...
        '%% AMT upshift/downshift logic based on engine speed\n',...
        'rpm = omega_eng * 60/(2*pi);\n',...
        'ratios = [%s];\n',...
        'if rpm > %g\n',...
        '    gear = min(6, gear + 1);\n',...
        'elseif rpm < %g\n',...
        '    gear = max(1, gear - 1);\n',...
        'end\n',...
        'ratio = ratios(gear);\n'],...
        num2str(trans.ratios,'%.2f '), trans.upshift_rpm, trans.downshift_rpm);
    set_param([blk '/GearLogic'],'MATLABFunctionLanguage','MATLAB');

    % Simpler: Use lookup table for ratio vs RPM
    add_block('simulink/Lookup Tables/1-D Lookup Table',[blk '/GearRatio']);
    gear_rpm_breaks = [0 700 900 1100 1400 1700 2000 2600];
    gear_ratios_val = [trans.ratios(1) trans.ratios(1) trans.ratios(2) trans.ratios(3) ...
                       trans.ratios(4) trans.ratios(5) trans.ratios(6) trans.ratios(6)];
    omega_breaks = gear_rpm_breaks * 2*pi/60;
    set_param([blk '/GearRatio'],...
        'BreakpointsForDimension1',mat2str(omega_breaks,6),...
        'Table',mat2str(gear_ratios_val,6),...
        'ExtrapMethod','Clip','Position',[150 80 320 130]);
    add_line(blk,'omega_eng/1','GearRatio/1','autorouting','on');

    % Output omega = omega_eng / (gear_ratio × final_drive)
    add_block('simulink/Math Operations/Divide',[blk '/omega_out_calc']);
    set_param([blk '/omega_out_calc'],'Position',[380 80 480 130]);
    add_block('simulink/Math Operations/Gain',[blk '/final_drive']);
    set_param([blk '/final_drive'],'Gain',num2str(trans.final_drive),'Position',[340 80 375 110]);
    add_line(blk,'omega_eng/1','omega_out_calc/1','autorouting','on');
    add_line(blk,'GearRatio/1','final_drive/1','autorouting','on');
    add_line(blk,'final_drive/1','omega_out_calc/2','autorouting','on');

    add_block('simulink/Sinks/Out1',[blk '/omega_out']); set_param([blk '/omega_out'],'Port','1','Position',[560 90 610 120]);
    add_block('simulink/Sinks/Out1',[blk '/ratio_out']); set_param([blk '/ratio_out'],'Port','2','Position',[560 140 610 170]);
    add_block('simulink/Sinks/Out1',[blk '/gear_num']);  set_param([blk '/gear_num'],'Port','3','Position',[560 190 610 220]);
    add_line(blk,'omega_out_calc/1','omega_out/1','autorouting','on');
    add_line(blk,'GearRatio/1','ratio_out/1','autorouting','on');
    add_line(blk,'GearRatio/1','gear_num/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_driveline_ss(blk, driveline)
% Propshaft torsion + differential (open, 50/50 split)
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    add_block('simulink/Sources/In1',[blk '/omega_trans']); set_param([blk '/omega_trans'],'Port','1','Position',[30 80 80 110]);
    add_block('simulink/Sources/In1',[blk '/omega_wheel']); set_param([blk '/omega_wheel'],'Port','2','Position',[30 180 80 210]);

    % Torsional compliance: T = k*(theta_in - theta_out) + c*(omega_in - omega_out)
    add_block('simulink/Math Operations/Sum',[blk '/d_omega']);
    set_param([blk '/d_omega'],'Inputs','+-','Position',[150 130 200 210]);
    add_line(blk,'omega_trans/1','d_omega/1','autorouting','on');
    add_line(blk,'omega_wheel/1','d_omega/2','autorouting','on');

    % theta_diff = integral(d_omega)
    add_block('simulink/Continuous/Integrator',[blk '/theta_diff']);
    set_param([blk '/theta_diff'],'InitialCondition','0','Position',[250 130 340 190]);
    add_line(blk,'d_omega/1','theta_diff/1','autorouting','on');

    % T_shaft = k*theta + c*d_omega
    add_block('simulink/Math Operations/Gain',[blk '/k_shaft']);
    set_param([blk '/k_shaft'],'Gain',num2str(driveline.propshaft_k),'Position',[400 100 500 140]);
    add_block('simulink/Math Operations/Gain',[blk '/c_shaft']);
    set_param([blk '/c_shaft'],'Gain',num2str(driveline.propshaft_c),'Position',[400 180 500 220]);
    add_line(blk,'theta_diff/1','k_shaft/1','autorouting','on');
    add_line(blk,'d_omega/1','c_shaft/1','autorouting','on');

    add_block('simulink/Math Operations/Sum',[blk '/T_shaft']);
    set_param([blk '/T_shaft'],'Inputs','++','Position',[550 130 600 210]);
    add_line(blk,'k_shaft/1','T_shaft/1','autorouting','on');
    add_line(blk,'c_shaft/1','T_shaft/2','autorouting','on');

    % Differential split (open diff – 50/50)
    add_block('simulink/Math Operations/Gain',[blk '/diff_split']);
    set_param([blk '/diff_split'],'Gain','0.5','Position',[650 150 750 190]);
    add_line(blk,'T_shaft/1','diff_split/1','autorouting','on');

    add_block('simulink/Sinks/Out1',[blk '/T_axle']);     set_param([blk '/T_axle'],'Port','1','Position',[820 150 870 180]);
    add_block('simulink/Sinks/Out1',[blk '/omega_axle']); set_param([blk '/omega_axle'],'Port','2','Position',[820 200 870 230]);
    add_line(blk,'diff_split/1','T_axle/1','autorouting','on');
    add_line(blk,'omega_trans/1','omega_axle/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_wheel_ss(blk, tyre, n_drive)
% Pacejka longitudinal tire model × n_drive wheels
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    add_block('simulink/Sources/In1',[blk '/T_axle']);    set_param([blk '/T_axle'],'Port','1','Position',[30 80 80 110]);
    add_block('simulink/Sources/In1',[blk '/omega_axle']);set_param([blk '/omega_axle'],'Port','2','Position',[30 180 80 210]);
    add_block('simulink/Sources/In1',[blk '/v_veh']);     set_param([blk '/v_veh'],'Port','3','Position',[30 280 80 310]);

    Fz = 9.81 * 16000 / 6;   % N  per tyre (GVW / n_wheels)
    R  = tyre.R_loaded_m;

    % Wheel rotational dynamics: Iz * alpha = T_axle - F_x * R
    add_block('simulink/Continuous/Integrator',[blk '/omega_wheel']);
    set_param([blk '/omega_wheel'],'InitialCondition','0','Position',[550 130 640 180]);

    % Longitudinal slip  kappa = (omega*R - v) / max(v, 0.01)
    add_block('simulink/Math Operations/Gain',[blk '/R_gain']);
    set_param([blk '/R_gain'],'Gain',num2str(R),'Position',[200 180 320 220]);
    add_line(blk,'omega_wheel/1','R_gain/1','autorouting','on');

    add_block('simulink/Math Operations/Sum',[blk '/slip_num']);
    set_param([blk '/slip_num'],'Inputs','+-','Position',[380 180 440 260]);
    add_block('simulink/Math Operations/Divide',[blk '/kappa']);
    set_param([blk '/kappa'],'Position',[480 190 560 250]);
    add_block('simulink/Math Operations/Fcn',[blk '/v_lim']);
    set_param([blk '/v_lim'],'Expr','max(u,0.01)','Position',[380 280 460 310]);
    add_line(blk,'R_gain/1','slip_num/1','autorouting','on');
    add_line(blk,'v_veh/1','slip_num/2','autorouting','on');
    add_line(blk,'slip_num/1','kappa/1','autorouting','on');
    add_line(blk,'v_lim/1','kappa/2','autorouting','on');
    add_line(blk,'v_veh/1','v_lim/1','autorouting','on');

    % Pacejka:  F_x = D*sin(C*atan(B*kappa - E*(B*kappa - atan(B*kappa))))
    B=tyre.B; C=tyre.C; D=tyre.D*Fz; E=tyre.E;
    pacejka_expr = sprintf('%g*sin(%g*atan(%g*u - %g*(%g*u - atan(%g*u))))', D,C,B,E,B,B);
    add_block('simulink/Math Operations/Fcn',[blk '/Pacejka']);
    set_param([blk '/Pacejka'],'Expr',pacejka_expr,'Position',[620 200 800 250]);
    add_line(blk,'kappa/1','Pacejka/1','autorouting','on');

    % Scale for n_drive wheels
    add_block('simulink/Math Operations/Gain',[blk '/n_wheels']);
    set_param([blk '/n_wheels'],'Gain',num2str(n_drive),'Position',[850 200 960 240]);
    add_line(blk,'Pacejka/1','n_wheels/1','autorouting','on');

    % Wheel rotational EOM:  omega_dot = (T_axle - F_x*R) / Iz_wheel
    Iz_w = tyre.Iz * n_drive;
    add_block('simulink/Math Operations/Gain',[blk '/Fx_R']);
    set_param([blk '/Fx_R'],'Gain',num2str(R),'Position',[850 280 960 320]);
    add_line(blk,'Pacejka/1','Fx_R/1','autorouting','on');

    add_block('simulink/Math Operations/Sum',[blk '/T_net_w']);
    set_param([blk '/T_net_w'],'Inputs','+-','Position',[1020 240 1080 320]);
    add_block('simulink/Math Operations/Gain',[blk '/inv_Iz_w']);
    set_param([blk '/inv_Iz_w'],'Gain',num2str(1/Iz_w),'Position',[1120 250 1230 300]);
    add_line(blk,'T_axle/1','T_net_w/1','autorouting','on');
    add_line(blk,'Fx_R/1','T_net_w/2','autorouting','on');
    add_line(blk,'T_net_w/1','inv_Iz_w/1','autorouting','on');
    add_line(blk,'inv_Iz_w/1','omega_wheel/1','autorouting','on');

    % Feed omega_wheel back
    add_line(blk,'omega_wheel/1','omega_wheel/state','autorouting','on');

    add_block('simulink/Sinks/Out1',[blk '/F_traction']); set_param([blk '/F_traction'],'Port','1','Position',[1050 200 1100 230]);
    add_block('simulink/Sinks/Out1',[blk '/omega_whl']);   set_param([blk '/omega_whl'],'Port','2','Position',[1050 280 1100 310]);
    add_line(blk,'n_wheels/1','F_traction/1','autorouting','on');
    add_line(blk,'omega_wheel/1','omega_whl/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_chassis_ss(blk, vehicle)
% Longitudinal vehicle dynamics: F=ma with aero + rolling resistance
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    add_block('simulink/Sources/In1',[blk '/F_traction']); set_param([blk '/F_traction'],'Port','1','Position',[30 80 80 110]);
    add_block('simulink/Sources/In1',[blk '/F_brake']);    set_param([blk '/F_brake'],'Port','2','Position',[30 180 80 210]);
    add_block('simulink/Sources/In1',[blk '/aero_state']); set_param([blk '/aero_state'],'Port','3','Position',[30 280 80 310]);
    add_block('simulink/Sources/In1',[blk '/grade_deg']);  set_param([blk '/grade_deg'],'Port','4','Position',[30 380 80 410]);

    m   = vehicle.GVW_kg;
    Cd  = vehicle.Cd;
    A   = vehicle.frontal_area;
    rho = vehicle.rho_air;
    Cr  = 0.007;   % rolling resistance

    % Aero drag = 0.5*rho*Cd*A*v^2
    add_block('simulink/Math Operations/Math Function',[blk '/v_sq']);
    set_param([blk '/v_sq'],'Operator','square','Position',[200 280 300 320]);
    add_block('simulink/Math Operations/Gain',[blk '/F_aero']);
    set_param([blk '/F_aero'],'Gain',num2str(0.5*rho*Cd*A),'Position',[340 280 480 320]);
    add_line(blk,'aero_state/1','v_sq/1','autorouting','on');
    add_line(blk,'v_sq/1','F_aero/1','autorouting','on');

    % Rolling resistance = Cr*m*g*cos(grade)
    add_block('simulink/Math Operations/Fcn',[blk '/F_roll']);
    set_param([blk '/F_roll'],'Expr',sprintf('%g*cos(u*pi/180)',Cr*m*9.81),'Position',[200 380 380 420]);
    add_line(blk,'grade_deg/1','F_roll/1','autorouting','on');

    % Grade resistance = m*g*sin(grade)
    add_block('simulink/Math Operations/Fcn',[blk '/F_grade']);
    set_param([blk '/F_grade'],'Expr',sprintf('%g*sin(u*pi/180)',m*9.81),'Position',[200 460 380 500]);
    add_line(blk,'grade_deg/1','F_grade/1','autorouting','on');

    % Net force = F_traction - F_brake - F_aero - F_roll - F_grade
    add_block('simulink/Math Operations/Sum',[blk '/F_net']);
    set_param([blk '/F_net'],'Inputs','+----','Position',[560 200 620 520]);
    add_line(blk,'F_traction/1','F_net/1','autorouting','on');
    add_line(blk,'F_brake/1','F_net/2','autorouting','on');
    add_line(blk,'F_aero/1','F_net/3','autorouting','on');
    add_line(blk,'F_roll/1','F_net/4','autorouting','on');
    add_line(blk,'F_grade/1','F_net/5','autorouting','on');

    % v = integral(F_net/m)
    add_block('simulink/Math Operations/Gain',[blk '/inv_m']);
    set_param([blk '/inv_m'],'Gain',num2str(1/m),'Position',[660 330 780 380]);
    add_block('simulink/Continuous/Integrator',[blk '/v_veh']);
    set_param([blk '/v_veh'],'InitialCondition','0','LowerSaturationLimit','0','Position',[820 320 920 390]);
    add_line(blk,'F_net/1','inv_m/1','autorouting','on');
    add_line(blk,'inv_m/1','v_veh/1','autorouting','on');

    % Distance = integral(v)
    add_block('simulink/Continuous/Integrator',[blk '/x_veh']);
    set_param([blk '/x_veh'],'InitialCondition','0','Position',[820 420 920 480]);
    add_line(blk,'v_veh/1','x_veh/1','autorouting','on');

    % Engine RPM estimate from v (rough: v/R_tyre * final_drive * avg_gear)
    add_block('simulink/Math Operations/Gain',[blk '/v2rpm']);
    set_param([blk '/v2rpm'],'Gain',num2str(60*4.1/(2*pi*0.513)),'Position',[820 250 920 290]);
    add_line(blk,'v_veh/1','v2rpm/1','autorouting','on');

    add_block('simulink/Sinks/Out1',[blk '/v_out']);   set_param([blk '/v_out'],'Port','1','Position',[1000 330 1050 360]);
    add_block('simulink/Sinks/Out1',[blk '/rpm_est']); set_param([blk '/rpm_est'],'Port','2','Position',[1000 260 1050 290]);
    add_line(blk,'v_veh/1','v_out/1','autorouting','on');
    add_line(blk,'v2rpm/1','rpm_est/1','autorouting','on');
    close_system(blk);
end

% ──────────────────────────────────────────────────────────────────────────
function build_brakes_ss(blk, vehicle, tyre)
% Hydraulic brake force model
    open_system(blk);
    delete_block([blk '/In1']); delete_block([blk '/Out1']);

    add_block('simulink/Sources/In1',[blk '/brake_demand']); set_param([blk '/brake_demand'],'Port','1','Position',[30 80 80 110]);

    % Max brake force = mu * m * g
    F_brake_max = tyre.mu_peak * vehicle.GVW_kg * 9.81;
    add_block('simulink/Math Operations/Gain',[blk '/F_brake_gain']);
    set_param([blk '/F_brake_gain'],'Gain',num2str(F_brake_max),'Position',[150 80 300 120]);
    add_block('simulink/Discontinuities/Saturation',[blk '/Sat_brake']);
    set_param([blk '/Sat_brake'],'UpperLimit',num2str(F_brake_max),'LowerLimit','0','Position',[350 80 480 120]);
    add_line(blk,'brake_demand/1','F_brake_gain/1','autorouting','on');
    add_line(blk,'F_brake_gain/1','Sat_brake/1','autorouting','on');

    add_block('simulink/Sinks/Out1',[blk '/F_brake']); set_param([blk '/F_brake'],'Port','1','Position',[560 80 610 110]);
    add_line(blk,'Sat_brake/1','F_brake/1','autorouting','on');
    close_system(blk);
end
