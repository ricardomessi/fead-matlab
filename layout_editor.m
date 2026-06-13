%% layout_editor.m  –  Interactive MATLAB GUI for FEAD Pulley Layout Editor
%  Drag-and-drop pulley positions, edit radius/coordinates numerically,
%  view live hub-load vectors and belt spans. Updates workspace variables.
%
%  Usage:  >> FEAD_params; layout_editor
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('pulleys','var'), FEAD_params; end

%% ── Create figure ──────────────────────────────────────────────────────────
fig = uifigure('Name','FEAD Layout Editor – Ashok Leyland H6',...
    'Position',[100 80 1300 780],...
    'Color',[0.07 0.09 0.15]);

%% ── Left panel: canvas (axes) ──────────────────────────────────────────────
ax = uiaxes(fig,'Position',[10 200 860 560],...
    'Color',[0.05 0.08 0.13],...
    'XColor',[0.3 0.4 0.5],'YColor',[0.3 0.4 0.5],...
    'GridColor',[0.12 0.18 0.25],'GridAlpha',1,...
    'XGrid','on','YGrid','on');
ax.Title.String = 'FEAD Belt Drive – Drag Pulleys to Edit Layout';
ax.Title.Color  = [0.96 0.62 0.04];
ax.XLabel.String = 'X [mm]'; ax.XLabel.Color = [0.58 0.65 0.75];
ax.YLabel.String = 'Y [mm]'; ax.YLabel.Color = [0.58 0.65 0.75];
hold(ax,'on');

%% ── Right panel: coordinate table ─────────────────────────────────────────
pnl = uipanel(fig,'Position',[880 200 410 560],...
    'BackgroundColor',[0.08 0.11 0.18],...
    'BorderType','none','Title','Pulley Coordinates & Properties',...
    'ForegroundColor',[0.96 0.62 0.04],'FontSize',13,'FontWeight','bold');

% Column headers
col_names = {'Name','X (mm)','Y (mm)','R (mm)','SR','CW'};
col_w     = [70 80 80 80 60 50];
x0 = 10; y0_hdr = 520;
for c = 1:numel(col_names)
    uilabel(pnl,'Text',col_names{c},...
        'Position',[x0 y0_hdr col_w(c) 24],...
        'FontWeight','bold','FontColor',[0.58 0.65 0.75],'FontSize',10);
    x0 = x0 + col_w(c) + 4;
end

%% Pulley colors matching the advanced web suite
pcolors = {'#f59e0b','#8b5cf6','#a78bfa','#34d399','#f472b6','#60a5fa'};
pnames  = {'CRK','FAN','IDR','ALT','AC','TEN'};

%% ── Create editable fields per pulley ─────────────────────────────────────
ef_x = cell(N_pulleys,1); ef_y = cell(N_pulleys,1);
ef_r = cell(N_pulleys,1); ef_sr = cell(N_pulleys,1);

for k = 1:N_pulleys
    y_row = y0_hdr - k*52;
    x0 = 10;
    hex = pcolors{k};
    rgb = hex2rgb(hex);

    % Name label (colored)
    uilabel(pnl,'Text',pnames{k},...
        'Position',[x0 y_row col_w(1) 30],...
        'FontWeight','bold','FontColor',rgb,'FontSize',12);
    x0 = x0 + col_w(1) + 4;

    % X edit field
    ef_x{k} = uieditfield(pnl,'numeric',...
        'Value',pulleys(k).x,'Limits',[-600 600],...
        'Position',[x0 y_row col_w(2) 30],...
        'BackgroundColor',[0.06 0.10 0.18],'FontColor',[0.89 0.91 0.94],'FontSize',10);
    x0 = x0 + col_w(2) + 4;

    % Y edit field
    ef_y{k} = uieditfield(pnl,'numeric',...
        'Value',pulleys(k).y,'Limits',[-100 700],...
        'Position',[x0 y_row col_w(3) 30],...
        'BackgroundColor',[0.06 0.10 0.18],'FontColor',[0.89 0.91 0.94],'FontSize',10);
    x0 = x0 + col_w(3) + 4;

    % R edit field
    ef_r{k} = uieditfield(pnl,'numeric',...
        'Value',pulleys(k).r,'Limits',[10 150],...
        'Position',[x0 y_row col_w(4) 30],...
        'BackgroundColor',[0.06 0.10 0.18],'FontColor',[0.89 0.91 0.94],'FontSize',10);
    x0 = x0 + col_w(4) + 4;

    % SR (read-only)
    uilabel(pnl,'Text',num2str(pulleys(k).sr,'%.3f'),...
        'Position',[x0 y_row col_w(5) 30],...
        'FontColor',[0.55 0.60 0.70],'FontSize',10);
    x0 = x0 + col_w(5) + 4;

    % CW/CCW label
    uilabel(pnl,'Text',ternary(pulleys(k).cw,'CW','CCW'),...
        'Position',[x0 y_row col_w(6) 30],...
        'FontColor',ternary(pulleys(k).cw,[0.20 0.90 0.65],[0.96 0.35 0.45]),'FontSize',10);

    % Wire callbacks (value changed → recompute + redraw)
    ef_x{k}.ValueChangedFcn = @(src,evt) on_coord_change(k,'x',src.Value);
    ef_y{k}.ValueChangedFcn = @(src,evt) on_coord_change(k,'y',src.Value);
    ef_r{k}.ValueChangedFcn = @(src,evt) on_coord_change(k,'r',src.Value);
end

%% ── Bottom bar: results ────────────────────────────────────────────────────
result_pnl = uipanel(fig,'Position',[10 10 1280 180],...
    'BackgroundColor',[0.06 0.09 0.16],'BorderType','none',...
    'Title','Live Results  (@ current RPM)','ForegroundColor',[0.96 0.62 0.04],...
    'FontSize',12,'FontWeight','bold');

% RPM slider
uilabel(result_pnl,'Text','Engine RPM','Position',[10 130 120 24],...
    'FontColor',[0.58 0.65 0.75],'FontSize',10);
rpm_slider = uislider(result_pnl,'Position',[130 148 300 3],...
    'Limits',[500 2500],'Value',1200,...
    'MajorTicks',[500 1000 1500 2000 2500]);
rpm_label  = uilabel(result_pnl,'Text','1200 RPM','Position',[450 135 100 24],...
    'FontColor',[0.96 0.62 0.04],'FontWeight','bold','FontSize',11);

% Tension slider
uilabel(result_pnl,'Text','Base Tension (N)','Position',[600 130 140 24],...
    'FontColor',[0.58 0.65 0.75],'FontSize',10);
ten_slider = uislider(result_pnl,'Position',[750 148 250 3],...
    'Limits',[200 1000],'Value',480,...
    'MajorTicks',[200 400 600 800 1000]);
ten_label  = uilabel(result_pnl,'Text','480 N','Position',[1015 135 100 24],...
    'FontColor',[0.96 0.62 0.04],'FontWeight','bold','FontSize',11);

% Results labels (hub forces)
res_labels = cell(N_pulleys,1);
for k = 1:N_pulleys
    hex = pcolors{k};
    rgb = hex2rgb(hex);
    res_labels{k} = uilabel(result_pnl,...
        'Text',sprintf('%s: F=— N  Dir=—°',pnames{k}),...
        'Position',[(k-1)*210+10 90 200 28],...
        'FontColor',rgb,'FontWeight','bold','FontSize',10);
end

% Belt life label
life_lbl = uilabel(result_pnl,'Text','Belt Life: —  km',...
    'Position',[10 55 400 28],'FontColor',[0.96 0.62 0.04],...
    'FontWeight','bold','FontSize',12);
bearing_lbl = uilabel(result_pnl,'Text','WP Bearing L10: —  h',...
    'Position',[450 55 400 28],'FontColor',[0.55 0.85 0.65],...
    'FontWeight','bold','FontSize',12);
slip_lbl = uilabel(result_pnl,'Text','Slip status: —',...
    'Position',[900 55 380 28],'FontColor',[0.96 0.62 0.04],...
    'FontSize',11);

% Export buttons
uibutton(result_pnl,'push','Text','↑ Save to Workspace',...
    'Position',[10 10 170 32],...
    'BackgroundColor',[0.12 0.18 0.30],'FontColor',[0.89 0.91 0.94],...
    'ButtonPushedFcn',@(~,~) save_to_ws());

uibutton(result_pnl,'push','Text','Reset to PDF Datum',...
    'Position',[200 10 170 32],...
    'BackgroundColor',[0.12 0.18 0.30],'FontColor',[0.89 0.91 0.94],...
    'ButtonPushedFcn',@(~,~) reset_to_defaults());

uibutton(result_pnl,'push','Text','▶ Run Simulation',...
    'Position',[390 10 170 32],...
    'BackgroundColor',[0.08 0.24 0.14],'FontColor',[0.20 0.95 0.60],...
    'FontWeight','bold','ButtonPushedFcn',@(~,~) run_sim());

%% ── Slider callbacks ───────────────────────────────────────────────────────
rpm_slider.ValueChangedFcn = @(src,~) on_rpm_change(src.Value);
ten_slider.ValueChangedFcn = @(src,~) on_ten_change(src.Value);

%% ── Internal state ─────────────────────────────────────────────────────────
curr_rpm = 1200;
curr_ten = 480;
dragging = -1;
pulley_handles = cell(N_pulleys,1);
span_handles   = gobjects(N_pulleys,1);
arrow_handles  = cell(N_pulleys,1);

%% ── Initial draw ───────────────────────────────────────────────────────────
redraw();

%% ── Mouse callbacks on axes ────────────────────────────────────────────────
set(fig,'WindowButtonDownFcn',  @on_mouse_down);
set(fig,'WindowButtonMotionFcn',@on_mouse_move);
set(fig,'WindowButtonUpFcn',    @on_mouse_up);

%% ═══════════════════════════════════════════════════════════════════════════
%  NESTED FUNCTIONS
% ═══════════════════════════════════════════════════════════════════════════

    function redraw()
        cla(ax);
        hold(ax,'on');

        % Draw belt spans (dashed amber)
        for k = 1:N_pulleys
            k2 = mod(k,N_pulleys)+1;
            x1=pulleys(k).x; y1=pulleys(k).y;
            x2=pulleys(k2).x; y2=pulleys(k2).y;
            plot(ax,[x1 x2],[y1 y2],'--','Color',[0.96 0.62 0.04 0.7],'LineWidth',1.8);
        end

        % Draw pulleys
        theta_vec = linspace(0,2*pi,80);
        for k = 1:N_pulleys
            p = pulleys(k);
            rgb = hex2rgb(pcolors{k});
            cx = p.x + p.r*cos(theta_vec);
            cy = p.y + p.r*sin(theta_vec);
            fill(ax,cx,cy,rgb*0.25,'EdgeColor',rgb,'LineWidth',2);
            % Hub circle
            cx2 = p.x + p.r*0.18*cos(theta_vec);
            cy2 = p.y + p.r*0.18*sin(theta_vec);
            fill(ax,cx2,cy2,[0.05 0.08 0.12],'EdgeColor',[1 1 1 0.5],'LineWidth',1);
            % Label
            text(ax,p.x,p.y+p.r+14,pnames{k},...
                'HorizontalAlignment','center','Color',rgb,...
                'FontSize',9,'FontWeight','bold');
        end

        % Draw hub load arrows
        res = compute_hub_loads(curr_rpm, curr_ten);
        for k = 1:N_pulleys
            if res(k).F > 1
                p   = pulleys(k);
                ang = atan2(res(k).Fy, res(k).Fx);
                scale = min(res(k).F/3500,1)*80 + 15;
                ex  = p.x + scale*cos(ang);
                ey  = p.y + scale*sin(ang);
                quiver(ax,p.x,p.y,ex-p.x,ey-p.y,...
                    'Color',[0.94 0.27 0.27],'LineWidth',2,...
                    'MaxHeadSize',0.5,'AutoScale','off');
                text(ax,ex+5,ey+5,sprintf('%.0fN',res(k).F),...
                    'Color',[0.97 0.60 0.60],'FontSize',8);
            end
        end

        ax.XLim = [-400 200];
        ax.YLim = [-100 550];
        axis(ax,'equal');
        drawnow;

        % Update result labels
        update_labels(res);
    end

    function res = compute_hub_loads(rpm, T_static)
        v = pulleys(1).r/1000 * rpm*2*pi/60;
        if v < 0.01, v = 0.01; end
        T_c = belt.lin_mass * v^2;
        res = struct('F',0,'Fx',0,'Fy',0,'dir',0,'SF',0);
        res = repmat(res,1,N_pulleys);
        for k = 1:N_pulleys
            P_k = interp1(load_table.rpm, load_table.(pnames{k}), rpm,'linear','extrap');
            T_eff = P_k*1000/v;
            T_tight = T_static + T_eff/2 + T_c;
            T_slack = max(T_static - T_eff/2, 0) + T_c;
            % Vector sum of adjacent span tensions
            kp = mod(k-2,N_pulleys)+1;
            kn = mod(k,N_pulleys)+1;
            dx_p = pulleys(kp).x - pulleys(k).x;
            dy_p = pulleys(kp).y - pulleys(k).y;
            dx_n = pulleys(kn).x - pulleys(k).x;
            dy_n = pulleys(kn).y - pulleys(k).y;
            Lp = hypot(dx_p,dy_p); Ln = hypot(dx_n,dy_n);
            if Lp < 1, Lp=1; end; if Ln < 1, Ln=1; end
            Fx = T_tight*dx_p/Lp + T_slack*dx_n/Ln;
            Fy = T_tight*dy_p/Lp + T_slack*dy_n/Ln;
            res(k).F   = hypot(Fx,Fy);
            res(k).Fx  = Fx;
            res(k).Fy  = Fy;
            res(k).dir = atan2d(Fy,Fx);
            % Slip SF
            wrap_arr = [166.5 127.6 108.4 145.1 105.7 76.4];
            mu_th = belt.mu * wrap_arr(k) * pi/180;
            if T_slack > 0.1
                res(k).SF = log(T_tight/T_slack)/mu_th;
            else
                res(k).SF = 99;
            end
        end
    end

    function update_labels(res)
        for k = 1:N_pulleys
            res_labels{k}.Text = sprintf('%s: F=%.0f N  Dir=%.0f°',pnames{k},res(k).F,res(k).dir);
        end
        % Belt life (simplified Wöhler)
        T_max = max(arrayfun(@(r)r.F,res));
        N_f   = belt.wohler_Nref * (belt.wohler_Tref/max(T_max,1))^belt.wohler_m;
        life_km = min(N_f * belt.length_m / 1000, 500000);
        life_lbl.Text = sprintf('Belt Life (CRK): %.0f km', life_km);

        % WP Bearing (TEN force as proxy)
        F_ten = res(6).F;
        P_ball = sqrt((F_ten*0.3)^2 + wp.F_radial^2);
        wp_rpm = curr_rpm * wp.gear_ratio;
        L10A   = (wp.ball.Cr/max(P_ball,1))^wp.ball.p * 1e6/(60*wp_rpm);
        bearing_lbl.Text = sprintf('WP Ball L10A: %.0f h  (ref 17820 h)', L10A);

        % Slip status
        SFs = arrayfun(@(r)r.SF,res);
        statuses = arrayfun(@(s)ternary(s<1,'SLIP!',ternary(s<1.3,'MARG','OK')),SFs,'UniformOutput',false);
        slip_str = strjoin(arrayfun(@(k)sprintf('%s:%s',pnames{k},statuses{k}),(1:6),'UniformOutput',false),'  ');
        slip_lbl.Text = ['Slip: ' slip_str];
    end

    function on_coord_change(k, field, val)
        pulleys(k).(field) = val;
        redraw();
    end

    function on_rpm_change(val)
        curr_rpm = round(val/50)*50;
        rpm_slider.Value = curr_rpm;
        rpm_label.Text = sprintf('%d RPM', curr_rpm);
        redraw();
    end

    function on_ten_change(val)
        curr_ten = round(val/10)*10;
        ten_label.Text = sprintf('%d N', curr_ten);
        redraw();
    end

    function on_mouse_down(~,~)
        cp = ax.CurrentPoint;
        mx = cp(1,1); my = cp(1,2);
        for k = 1:N_pulleys
            if hypot(mx-pulleys(k).x, my-pulleys(k).y) < pulleys(k).r * 1.3
                dragging = k;
                return;
            end
        end
    end

    function on_mouse_move(~,~)
        if dragging < 1, return; end
        cp = ax.CurrentPoint;
        pulleys(dragging).x = round(cp(1,1));
        pulleys(dragging).y = round(cp(1,2));
        ef_x{dragging}.Value = pulleys(dragging).x;
        ef_y{dragging}.Value = pulleys(dragging).y;
        redraw();
    end

    function on_mouse_up(~,~)
        dragging = -1;
    end

    function save_to_ws()
        assignin('base','pulleys',pulleys);
        msgbox('Pulley layout saved to workspace variable ''pulleys''.','Saved','help');
    end

    function reset_to_defaults()
        FEAD_params;
        for k = 1:N_pulleys
            ef_x{k}.Value = pulleys(k).x;
            ef_y{k}.Value = pulleys(k).y;
            ef_r{k}.Value = pulleys(k).r;
        end
        redraw();
    end

    function run_sim()
        save_to_ws();
        build_FEAD_model;
        run_fead_sim;
    end
end

%% ── Utility ────────────────────────────────────────────────────────────────
function rgb = hex2rgb(hex)
    hex = strrep(hex,'#','');
    rgb = double([hex2dec(hex(1:2)) hex2dec(hex(3:4)) hex2dec(hex(5:6))])/255;
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
