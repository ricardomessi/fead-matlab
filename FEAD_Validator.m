%% FEAD_Validator.m  –  Workflow Validation + Layout Optimization Engine
%  Validates belt drive design and suggests improvements.
%  Called by FEAD_App.m on every parameter change.
%
%  Usage:  [report, score, suggestions] = FEAD_Validator(pulleys, belt, conditions)
% ─────────────────────────────────────────────────────────────────────────────
function [report, score, suggestions, opt_layout] = FEAD_Validator(pulleys, belt, load_table, conditions, sim_rpm)

if nargin < 5, sim_rpm = 1200; end

pnames   = {'CRK','FAN','IDR','ALT','AC','TEN'};
np       = numel(pnames);
wrap_deg = [166.5 127.6 108.4 145.1 105.7 76.4];

%% ── Compute state at sim_rpm ─────────────────────────────────────────────────
v = pulleys(1).r/1000 * sim_rpm*2*pi/60;
if v < 0.01, v = 0.01; end
T_c = belt.lin_mass * v^2;

%% ── Check 1: Belt velocity ───────────────────────────────────────────────────
chk.v_belt.value  = v;
chk.v_belt.limit  = belt.v_max;
chk.v_belt.pass   = v <= belt.v_max;
chk.v_belt.label  = 'Belt velocity';
chk.v_belt.unit   = 'm/s';
chk.v_belt.msg    = sprintf('v = %.2f m/s  (limit %.0f m/s)', v, belt.v_max);

%% ── Check 2: Slip safety factor per pulley ───────────────────────────────────
SF = zeros(1,np);
T_tight_arr = zeros(1,np);
T_slack_arr = zeros(1,np);
for k = 1:np
    pn  = pnames{k};
    P_k = interp1(load_table.rpm, load_table.(pn), sim_rpm,'linear','extrap');
    P_k = max(P_k,0);
    % Check AC condition
    if strcmp(pn,'AC') && isfield(conditions,'ac') && ~conditions.ac
        P_k = 0;
    end
    T_eff = P_k*1000/v;
    T_t   = belt.static_tension + T_eff/2 + T_c;
    T_s   = max(belt.static_tension - T_eff/2, 0) + T_c;
    T_tight_arr(k) = T_t;
    T_slack_arr(k) = T_s;
    mu_th = belt.mu * wrap_deg(k)*pi/180;
    if T_s > 0.1
        SF(k) = log(T_t/T_s)/mu_th;
    else
        SF(k) = 99;
    end
end
chk.slip.SF       = SF;
chk.slip.pass     = all(SF >= 1.3);
chk.slip.critical = pnames(SF < 1.3);
chk.slip.label    = 'Slip safety (SF≥1.3)';
chk.slip.msg      = sprintf('Min SF = %.2f on %s', min(SF), pnames{SF==min(SF)});

%% ── Check 3: Max tension vs belt rating ──────────────────────────────────────
T_max_calc = max(T_tight_arr);
chk.tension.value  = T_max_calc;
chk.tension.limit  = belt.T_max;
chk.tension.pass   = T_max_calc <= belt.T_max;
chk.tension.label  = 'Max belt tension';
chk.tension.unit   = 'N';
chk.tension.msg    = sprintf('T_max = %.0f N  (limit %.0f N)', T_max_calc, belt.T_max);

%% ── Check 4: Minimum wrap angles ────────────────────────────────────────────
min_wrap   = 60;   % deg
wrap_calc  = zeros(1,np);
for k = 1:np
    kp = mod(k-2,np)+1; kn = mod(k,np)+1;
    d1 = [pulleys(kp).x-pulleys(k).x, pulleys(kp).y-pulleys(k).y];
    d2 = [pulleys(kn).x-pulleys(k).x, pulleys(kn).y-pulleys(k).y];
    L1 = norm(d1); L2 = norm(d2);
    if L1>0 && L2>0
        cosang = dot(d1,d2)/(L1*L2);
        wrap_calc(k) = acosd(max(-1,min(1,cosang)));
    end
end
chk.wrap.values   = wrap_calc;
chk.wrap.limit    = min_wrap;
chk.wrap.pass     = all(wrap_calc >= min_wrap);
chk.wrap.critical = pnames(wrap_calc < min_wrap);
chk.wrap.label    = 'Min wrap angle (≥60°)';
chk.wrap.msg      = sprintf('Min wrap = %.1f° on %s', min(wrap_calc), pnames{wrap_calc==min(wrap_calc)});

%% ── Check 5: Belt span lengths (no extremely short/long spans) ───────────────
span_len = zeros(1,np);
for k = 1:np
    kn = mod(k,np)+1;
    span_len(k) = hypot(pulleys(kn).x-pulleys(k).x, pulleys(kn).y-pulleys(k).y);
end
chk.spans.values  = span_len;
chk.spans.pass    = all(span_len >= 30) && all(span_len <= 600);
chk.spans.label   = 'Span lengths (30–600mm)';
chk.spans.msg     = sprintf('Min span = %.0f mm, Max = %.0f mm', min(span_len), max(span_len));

%% ── Check 6: Hub load vs bearing capacity (typical limit 5000 N) ─────────────
F_hub_arr = zeros(1,np);
for k = 1:np
    kp = mod(k-2,np)+1; kn = mod(k,np)+1;
    dx_p = pulleys(kp).x-pulleys(k).x; dy_p = pulleys(kp).y-pulleys(k).y;
    dx_n = pulleys(kn).x-pulleys(k).x; dy_n = pulleys(kn).y-pulleys(k).y;
    Lp = max(hypot(dx_p,dy_p),1); Ln = max(hypot(dx_n,dy_n),1);
    Fx = T_tight_arr(k)*dx_p/Lp + T_slack_arr(k)*dx_n/Ln;
    Fy = T_tight_arr(k)*dy_p/Lp + T_slack_arr(k)*dy_n/Ln;
    F_hub_arr(k) = hypot(Fx,Fy);
end
F_hub_limit = 5000;
chk.hub.values = F_hub_arr;
chk.hub.limit  = F_hub_limit;
chk.hub.pass   = all(F_hub_arr <= F_hub_limit);
chk.hub.label  = 'Hub loads (≤5000 N)';
chk.hub.msg    = sprintf('Max hub load = %.0f N on %s', max(F_hub_arr), pnames{F_hub_arr==max(F_hub_arr)});

%% ── Check 7: Belt fatigue life (WLTC, min 200000 km) ────────────────────────
wltc_rpm = [900 1200 1600 2000];
wltc_w   = [0.25 0.25 0.25 0.25];
life_min_req = 200000; % km
belt_life = zeros(1,np);
for k = 1:np
    D = 0;
    for wi = 1:4
        vi   = max(pulleys(1).r/1000 * wltc_rpm(wi)*2*pi/60, 0.01);
        P_i  = max(interp1(load_table.rpm, load_table.(pnames{k}), wltc_rpm(wi),'linear','extrap'),0);
        Tt_i = belt.static_tension + P_i*1000/vi/2 + belt.lin_mass*vi^2;
        Nf_i = belt.wohler_Nref * (belt.wohler_Tref/max(Tt_i,1))^belt.wohler_m;
        D    = D + wltc_w(wi)/Nf_i;
    end
    belt_life(k) = min(belt.length_m/1000/max(D,1e-15), 500000);
end
overall_life = min(belt_life);
chk.fatigue.life_km = belt_life;
chk.fatigue.overall = overall_life;
chk.fatigue.pass    = overall_life >= life_min_req;
chk.fatigue.label   = 'Belt life (≥200 000 km)';
chk.fatigue.msg     = sprintf('Overall life = %.0f km (limit %.0f km)', overall_life, life_min_req);

%% ── Check 8: Temperature range ──────────────────────────────────────────────
op_temp = conditions.temp_C;
chk.temp.value = op_temp;
chk.temp.pass  = op_temp >= belt.temp_min && op_temp <= belt.temp_max;
chk.temp.label = 'Operating temperature';
chk.temp.msg   = sprintf('T_op = %.0f°C (range [%.0f, %.0f]°C)', op_temp, belt.temp_min, belt.temp_max);

%% ── Build report struct ─────────────────────────────────────────────────────
checks = {chk.v_belt, chk.slip, chk.tension, chk.wrap, chk.spans, chk.hub, chk.fatigue, chk.temp};
n_pass = sum(cellfun(@(c) c.pass, checks));
score  = round(100 * n_pass / numel(checks));

report.checks    = checks;
report.score     = score;
report.n_pass    = n_pass;
report.n_total   = numel(checks);
report.v_belt    = v;
report.SF        = SF;
report.T_tight   = T_tight_arr;
report.T_slack   = T_slack_arr;
report.F_hub     = F_hub_arr;
report.belt_life = belt_life;
report.span_len  = span_len;
report.wrap_deg  = wrap_calc;

%% ── Generate design suggestions ─────────────────────────────────────────────
suggestions = {};
idx = 1;

if ~chk.slip.pass
    suggestions{idx} = sprintf('⚠ SLIP RISK on %s: Increase tensioner preload or reduce AC compressor load.',...
        strjoin(chk.slip.critical,', ')); idx=idx+1;
end
if ~chk.tension.pass
    suggestions{idx} = sprintf('⚠ TENSION EXCEEDED: Switch to %d-rib belt or increase belt cross-section.',...
        belt.ribs+2); idx=idx+1;
end
if ~chk.wrap.pass
    suggestions{idx} = '⚠ LOW WRAP ANGLE: Move idler closer to adjacent pulley or add a second idler.'; idx=idx+1;
end
if ~chk.hub.pass
    suggestions{idx} = sprintf('⚠ HIGH HUB LOAD on %s: Move pulley to reduce span angle or add bearing support.',...
        pnames{F_hub_arr==max(F_hub_arr)}); idx=idx+1;
end
if ~chk.fatigue.pass
    suggestions{idx} = sprintf('⚠ LOW BELT LIFE (%.0f km): Reduce static tension or use Aramid-core belt.', overall_life); idx=idx+1;
end
if ~chk.v_belt.pass
    suggestions{idx} = sprintf('⚠ BELT SPEED EXCEEDED (%.1f > %.0f m/s): Reduce pulley diameter or engine RPM limit.', v, belt.v_max); idx=idx+1;
end
if ~chk.temp.pass
    suggestions{idx} = sprintf('⚠ TEMPERATURE OUT OF RANGE: Use EPDM belt rated to %.0f°C.', op_temp+20); idx=idx+1;
end
if isempty(suggestions)
    suggestions{1} = '✅ All checks passed. Design is validated for current operating conditions.';
end

% Optimal tensioner position suggestion
[~,best_ten] = min(abs(SF - 1.8));
if SF(6) < 1.5
    suggestions{end+1} = '💡 SUGGESTION: Move tensioner to MIN position to increase slack-side tension.';
elseif SF(6) > 3.0
    suggestions{end+1} = '💡 SUGGESTION: Move tensioner to FREE position to reduce bearing loads.';
end

% Belt upgrade suggestion
if overall_life < 300000 && strcmp(belt.core,'Polyester')
    suggestions{end+1} = '💡 UPGRADE: Switch to Aramid-core belt (MT620 AMD) — 2-3× fatigue life improvement.';
end

%% ── Layout optimization (greedy hub-load minimizer) ─────────────────────────
opt_layout = pulleys;  % start from current layout
% Simple gradient-free: try ±5mm shifts for tensioner (index 6) only
best_F_sum = sum(F_hub_arr);
best_ten_x = pulleys(6).x;
best_ten_y = pulleys(6).y;

for dx = -20:5:20
    for dy = -20:5:20
        trial = pulleys;
        trial(6).x = pulleys(6).x + dx;
        trial(6).y = pulleys(6).y + dy;
        F_trial = compute_hub_sum(trial, belt, load_table, sim_rpm, pnames, np);
        if F_trial < best_F_sum
            best_F_sum = F_trial;
            best_ten_x = trial(6).x;
            best_ten_y = trial(6).y;
        end
    end
end
opt_layout(6).x = best_ten_x;
opt_layout(6).y = best_ten_y;

if best_ten_x ~= pulleys(6).x || best_ten_y ~= pulleys(6).y
    suggestions{end+1} = sprintf('💡 OPTIMAL TENSIONER: Move to X=%.0f, Y=%.0f mm (reduces total hub load by %.0f N)',...
        best_ten_x, best_ten_y, sum(F_hub_arr)-best_F_sum);
end

end % main function

%% ─────────────────────────────────────────────────────────────────────────────
function F_sum = compute_hub_sum(pulleys, belt, load_table, rpm, pnames, np)
    v = max(pulleys(1).r/1000 * rpm*2*pi/60, 0.01);
    T_c = belt.lin_mass*v^2;
    T_t = zeros(1,np); T_s = zeros(1,np);
    for k = 1:np
        P_k = max(interp1(load_table.rpm, load_table.(pnames{k}), rpm,'linear','extrap'),0);
        T_eff = P_k*1000/v;
        T_t(k) = belt.static_tension + T_eff/2 + T_c;
        T_s(k) = max(belt.static_tension - T_eff/2,0) + T_c;
    end
    F_sum = 0;
    for k = 1:np
        kp = mod(k-2,np)+1; kn = mod(k,np)+1;
        dx_p = pulleys(kp).x-pulleys(k).x; dy_p = pulleys(kp).y-pulleys(k).y;
        dx_n = pulleys(kn).x-pulleys(k).x; dy_n = pulleys(kn).y-pulleys(k).y;
        Lp = max(hypot(dx_p,dy_p),1); Ln = max(hypot(dx_n,dy_n),1);
        Fx = T_t(k)*dx_p/Lp + T_s(k)*dx_n/Ln;
        Fy = T_t(k)*dy_p/Lp + T_s(k)*dy_n/Ln;
        F_sum = F_sum + hypot(Fx,Fy);
    end
end
