%% FEAD_Animation.m  –  Real-time FEAD Belt Drive Animation Engine
%  Creates a live animated 2D model of the belt drive with:
%   • Rotating pulleys (spokes + hub + ribs)
%   • Moving belt segments (travelling wave effect)
%   • Live hub-load force arrows (update with RPM/tension)
%   • Colour-coded slip status rings
%   • RPM, tension, and belt velocity readouts
%
%  Usage:  anim = FEAD_Animation(ax, pulleys, belt, load_table)
%          FEAD_Animation_Step(anim, rpm, tension, conditions)   % call in timer
%          FEAD_Animation_Stop(anim)
% ─────────────────────────────────────────────────────────────────────────────

function anim = FEAD_Animation(ax, pulleys, belt, load_table)
%FEAD_ANIMATION  Initialise animation handles on axes ax.

cla(ax); hold(ax,'on'); axis(ax,'equal');
ax.Color        = [0.05 0.08 0.12];
ax.XColor       = [0.35 0.45 0.55];
ax.YColor       = [0.35 0.45 0.55];
ax.GridColor    = [0.10 0.16 0.22];
ax.GridAlpha    = 1;
ax.XGrid        = 'on'; ax.YGrid = 'on';
ax.XLim         = [-420 230];
ax.YLim         = [-120 560];
ax.XLabel.String = 'X (mm)'; ax.XLabel.Color = [0.55 0.62 0.72];
ax.YLabel.String = 'Y (mm)'; ax.YLabel.Color = [0.55 0.62 0.72];
ax.Title.String = 'FEAD Test Rig – Live Animation';
ax.Title.Color  = [0.96 0.62 0.04];
ax.Title.FontSize = 12;

np      = numel(pulleys);
pnames  = {'CRK','FAN','IDR','ALT','AC','TEN'};
pcolors = {[0.96 0.62 0.04],[0.55 0.36 0.96],[0.65 0.54 0.98],...
           [0.20 0.85 0.60],[0.96 0.28 0.71],[0.38 0.64 0.98]};

theta_fine = linspace(0,2*pi,128);

%% ── Draw belt spans (static background) ────────────────────────────────────
anim.h_span = gobjects(np,1);
for k = 1:np
    kn = mod(k,np)+1;
    x1 = pulleys(k).x; y1 = pulleys(k).y;
    x2 = pulleys(kn).x; y2 = pulleys(kn).y;
    anim.h_span(k) = plot(ax,[x1 x2],[y1 y2],...
        'Color',[0.85 0.60 0.05 0.6],'LineWidth',3.5);
end

%% ── Draw belt markers (moving dots to show belt travel) ──────────────────────
n_markers = 12;
anim.h_markers = gobjects(n_markers,1);
for m = 1:n_markers
    anim.h_markers(m) = plot(ax, 0, 0, 's',...
        'Color',[1.0 0.80 0.20],'MarkerSize',5,'MarkerFaceColor',[1.0 0.80 0.20]);
end

%% ── Draw pulleys (filled discs with spokes) ─────────────────────────────────
anim.h_disc    = gobjects(np,1);
anim.h_hub     = gobjects(np,1);
anim.h_spokes  = cell(np,1);
anim.h_ring    = gobjects(np,1);   % slip status ring
anim.h_label   = gobjects(np,1);
anim.h_arrow   = gobjects(np,1);   % hub load arrows
anim.h_atext   = gobjects(np,1);   % arrow labels

for k = 1:np
    px = pulleys(k).x;
    py = pulleys(k).y;
    r  = pulleys(k).r;
    c  = pcolors{k};

    % Outer disc (filled)
    xd = px + r*cos(theta_fine);
    yd = py + r*sin(theta_fine);
    anim.h_disc(k) = patch(ax, xd, yd, c*0.18,...
        'EdgeColor',c,'LineWidth',2,'FaceAlpha',0.9);

    % Status ring (outside disc – changes colour based on SF)
    xr = px + (r+5)*cos(theta_fine);
    yr = py + (r+5)*sin(theta_fine);
    anim.h_ring(k) = patch(ax, xr, yr, 'none',...
        'EdgeColor',[0.20 0.85 0.60],'LineWidth',3,'FaceAlpha',0);

    % Hub circle
    xh = px + r*0.18*cos(theta_fine);
    yh = py + r*0.18*sin(theta_fine);
    anim.h_hub(k) = patch(ax, xh, yh, [0.04 0.06 0.10],...
        'EdgeColor',[0.5 0.6 0.7],'LineWidth',1.5);

    % Spokes (4 lines, will rotate)
    n_spokes = 4;
    anim.h_spokes{k} = gobjects(n_spokes,1);
    for s = 1:n_spokes
        ang = (s-1)*pi/2;
        anim.h_spokes{k}(s) = plot(ax,...
            [px+r*0.18*cos(ang), px+r*0.85*cos(ang)],...
            [py+r*0.18*sin(ang), py+r*0.85*sin(ang)],...
            'Color',c,'LineWidth',2.5);
    end

    % Label
    anim.h_label(k) = text(ax, px, py+r+18, pnames{k},...
        'HorizontalAlignment','center','Color',c,...
        'FontSize',9,'FontWeight','bold');

    % Hub load arrow (quiver starts at pulley centre, initially zero)
    anim.h_arrow(k) = quiver(ax, px, py, 0, 0,...
        'Color',[0.94 0.27 0.27],'LineWidth',2,...
        'MaxHeadSize',0.6,'AutoScale','off');
    anim.h_atext(k) = text(ax, px+5, py+5, '',...
        'Color',[0.97 0.60 0.60],'FontSize',8,'FontWeight','bold');
end

%% ── Status text overlays ────────────────────────────────────────────────────
anim.h_rpm_txt  = text(ax,-400,510, 'RPM: —',...
    'Color',[0.96 0.62 0.04],'FontSize',11,'FontWeight','bold');
anim.h_vel_txt  = text(ax,-400,480, 'Belt v: —',...
    'Color',[0.65 0.54 0.98],'FontSize',10);
anim.h_ten_txt  = text(ax,-400,450, 'T_static: —',...
    'Color',[0.38 0.64 0.98],'FontSize',10);
anim.h_score_txt= text(ax,-400,415, 'Design Score: —',...
    'Color',[0.20 0.85 0.60],'FontSize',11,'FontWeight','bold');

%% ── Store references ────────────────────────────────────────────────────────
anim.pulleys    = pulleys;
anim.belt       = belt;
anim.load_table = load_table;
anim.pcolors    = pcolors;
anim.pnames     = pnames;
anim.ax         = ax;
anim.theta_fine = theta_fine;
anim.ang_offset = zeros(1,np);  % current rotation angle per pulley
anim.marker_pos = linspace(0,1,n_markers);  % normalised belt position [0,1]
anim.n_markers  = n_markers;
anim.time       = 0;

end % FEAD_Animation


function FEAD_Animation_Step(anim, rpm, tension, report)
%FEAD_ANIMATION_STEP  Advance animation by one frame (call from timer ~30Hz).

ax        = anim.ax;
pulleys   = anim.pulleys;
pcolors   = anim.pcolors;
np        = numel(pulleys);
dt        = 1/30;  % assume 30 Hz

% Belt linear velocity
v_belt = pulleys(1).r/1000 * rpm*2*pi/60;

% Angular velocity per pulley  ω_k = v/r_k  (all driven by same belt)
for k = 1:np
    r_k = pulleys(k).r/1000;  % m
    if pulleys(k).cw
        omega_k = v_belt / r_k;
    else
        omega_k = -v_belt / r_k;
    end
    anim.ang_offset(k) = anim.ang_offset(k) + omega_k*dt;

    % Update spoke positions (rotate around pulley centre)
    px = pulleys(k).x; py = pulleys(k).y; r = pulleys(k).r;
    n_spokes = numel(anim.h_spokes{k});
    for s = 1:n_spokes
        base_ang = (s-1)*pi/2 + anim.ang_offset(k);
        set(anim.h_spokes{k}(s),...
            'XData',[px+r*0.18*cos(base_ang), px+r*0.85*cos(base_ang)],...
            'YData',[py+r*0.18*sin(base_ang), py+r*0.85*sin(base_ang)]);
    end
end

% Advance belt markers along path
belt_speed_norm = v_belt * dt / (sum(anim.belt.length_m));
anim.marker_pos = mod(anim.marker_pos + belt_speed_norm, 1);

% Map marker positions to (x,y) along belt path
spans_x = [pulleys.x]; spans_y = [pulleys.y];
spans_x(end+1) = pulleys(1).x; spans_y(end+1) = pulleys(1).y;
cum_len = zeros(1,np+1);
for k = 1:np
    cum_len(k+1) = cum_len(k) + hypot(spans_x(k+1)-spans_x(k), spans_y(k+1)-spans_y(k));
end
total_path = cum_len(end);

for m = 1:anim.n_markers
    pos_mm = anim.marker_pos(m) * total_path;
    seg    = find(cum_len <= pos_mm, 1, 'last');
    seg    = min(seg, np);
    frac   = (pos_mm - cum_len(seg)) / max(cum_len(seg+1)-cum_len(seg),1);
    mx     = spans_x(seg)*(1-frac) + spans_x(seg+1)*frac;
    my     = spans_y(seg)*(1-frac) + spans_y(seg+1)*frac;
    set(anim.h_markers(m),'XData',mx,'YData',my);
end

% Update hub load arrows and slip status rings
if nargin >= 4 && ~isempty(report)
    for k = 1:np
        F  = report.F_hub(k);
        SF = report.SF(k);
        dir_k = atan2d(report.T_tight(k),  report.T_slack(k));  % approx direction

        % Recompute direction from span geometry
        kp = mod(k-2,np)+1; kn = mod(k,np)+1;
        dx_p = pulleys(kp).x-pulleys(k).x; dy_p = pulleys(kp).y-pulleys(k).y;
        dx_n = pulleys(kn).x-pulleys(k).x; dy_n = pulleys(kn).y-pulleys(k).y;
        Lp = max(hypot(dx_p,dy_p),1); Ln = max(hypot(dx_n,dy_n),1);
        Fx = report.T_tight(k)*dx_p/Lp + report.T_slack(k)*dx_n/Ln;
        Fy = report.T_tight(k)*dy_p/Lp + report.T_slack(k)*dy_n/Ln;
        scale = min(F/4000, 1)*60 + 10;
        if F > 10
            ex = Fx/max(F,1)*scale; ey = Fy/max(F,1)*scale;
            set(anim.h_arrow(k),...
                'XData',pulleys(k).x,'YData',pulleys(k).y,...
                'UData',ex,'VData',ey);
            set(anim.h_atext(k),'Position',[pulleys(k).x+ex+4, pulleys(k).y+ey+4],...
                'String',sprintf('%.0fN',F));
        end

        % Slip ring colour: green=OK, amber=marginal, red=slip
        if SF >= 1.3
            ring_c = [0.20 0.85 0.60];
        elseif SF >= 1.0
            ring_c = [0.96 0.62 0.04];
        else
            ring_c = [0.94 0.27 0.27];
        end
        anim.h_ring(k).EdgeColor = ring_c;
    end

    % Update text overlays
    v_belt_ms = pulleys(1).r/1000 * rpm*2*pi/60;
    anim.h_rpm_txt.String  = sprintf('Engine: %d RPM', round(rpm));
    anim.h_vel_txt.String  = sprintf('Belt v: %.2f m/s', v_belt_ms);
    anim.h_ten_txt.String  = sprintf('T_{static}: %d N', round(tension));
    if isfield(report,'score')
        clr = score_color(report.score);
        anim.h_score_txt.String = sprintf('Design Score: %d / 100', report.score);
        anim.h_score_txt.Color  = clr;
    end
end

drawnow limitrate;
end % FEAD_Animation_Step


function FEAD_Animation_UpdateLayout(anim, pulleys_new)
%FEAD_ANIMATION_UPDATELAYOUT  Reposition all graphic elements after layout edit.
    ax = anim.ax;
    np = numel(pulleys_new);
    theta_fine = anim.theta_fine;

    for k = 1:np
        px = pulleys_new(k).x; py = pulleys_new(k).y;
        r  = pulleys_new(k).r;

        % Update disc
        xd = px + r*cos(theta_fine); yd = py + r*sin(theta_fine);
        anim.h_disc(k).XData = xd; anim.h_disc(k).YData = yd;

        % Ring
        xr = px+(r+5)*cos(theta_fine); yr = py+(r+5)*sin(theta_fine);
        anim.h_ring(k).XData = xr; anim.h_ring(k).YData = yr;

        % Hub
        xh = px+r*0.18*cos(theta_fine); yh = py+r*0.18*sin(theta_fine);
        anim.h_hub(k).XData = xh; anim.h_hub(k).YData = yh;

        % Label
        anim.h_label(k).Position(1:2) = [px, py+r+18];

        % Spans
        kn = mod(k,np)+1;
        anim.h_span(k).XData = [px pulleys_new(kn).x];
        anim.h_span(k).YData = [py pulleys_new(kn).y];
    end

    anim.pulleys = pulleys_new;
    drawnow limitrate;
end


function c = score_color(score)
    if score >= 80
        c = [0.20 0.85 0.60];   % green
    elseif score >= 60
        c = [0.96 0.62 0.04];   % amber
    else
        c = [0.94 0.27 0.27];   % red
    end
end
