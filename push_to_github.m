%% push_to_github.m  –  Push FEAD model + results to GitHub
%  Saves layout JSON, validation Markdown, commits all .m files and pushes.
%  Token read from environment variable GH_TOKEN (never hardcoded).
%
%  Setup:  setenv('GH_TOKEN','your_token_here')  % run once per session
%  Usage:  push_to_github(pulleys, belt, report)
% ─────────────────────────────────────────────────────────────────────────────
function push_to_github(pulleys, belt, report)

% Read token from environment (never store in code)
GH_TOKEN = getenv('GH_TOKEN');
if isempty(GH_TOKEN)
    GH_TOKEN = input('GitHub personal access token: ','s');
    setenv('GH_TOKEN', GH_TOKEN);
end

REPO_DIR = 'C:\Users\RICARDO\fead-matlab';
REMOTE   = sprintf('https://%s@github.com/ricardomessi/fead-matlab.git', GH_TOKEN);
pnames   = {'CRK','FAN','IDR','ALT','AC','TEN'};

fprintf('\n=== Pushing FEAD Model to GitHub ===\n');

%% 1. Save layout as JSON ─────────────────────────────────────────────────────
layout.timestamp = datestr(now,'yyyy-mm-dd HH:MM:SS');
layout.belt = struct('name',belt.name,'ribs',belt.ribs,...
    'length_mm',belt.length_mm,'mu',belt.mu,'EA',belt.EA,...
    'static_tension',belt.static_tension,'T_max',belt.T_max);
for k = 1:numel(pulleys)
    layout.pulleys.(pnames{k}) = struct(...
        'x',pulleys(k).x,'y',pulleys(k).y,'r',pulleys(k).r,'sr',pulleys(k).sr);
end
if nargin >= 3 && ~isempty(report)
    layout.validation.score     = report.score;
    layout.validation.n_pass    = report.n_pass;
    layout.validation.n_total   = report.n_total;
    layout.validation.F_hub     = report.F_hub;
    layout.validation.SF        = report.SF;
    layout.validation.belt_life = report.belt_life_km;
end

json_str  = jsonencode(layout,'PrettyPrint',true);
json_file = fullfile(REPO_DIR,'latest_layout.json');
fid = fopen(json_file,'w'); fprintf(fid,'%s',json_str); fclose(fid);
fprintf('  Saved: latest_layout.json\n');

%% 2. Save validation Markdown ────────────────────────────────────────────────
if nargin >= 3 && ~isempty(report)
    md_file = fullfile(REPO_DIR,'VALIDATION_REPORT.md');
    fid = fopen(md_file,'w');
    fprintf(fid,'# FEAD Validation Report\n\n**Generated:** %s\n\n',datestr(now));
    fprintf(fid,'## Design Score: %d / 100\n\n%d of %d checks passed.\n\n',...
        report.score,report.n_pass,report.n_total);
    fprintf(fid,'## Belt: %s\n\n',belt.name);
    fprintf(fid,'| Property | Value |\n|---|---|\n');
    fprintf(fid,'| Length | %.1f mm |\n| Static tension | %.0f N |\n| Ribs | %d |\n',...
        belt.length_mm,belt.static_tension,belt.ribs);
    fprintf(fid,'\n## Hub Loads & Belt Life\n\n');
    fprintf(fid,'| Pulley | F_hub (N) | Slip SF | Belt Life (km) |\n|---|---|---|---|\n');
    for k = 1:numel(pnames)
        fprintf(fid,'| %s | %.0f | %.2f | %.0f |\n',...
            pnames{k},report.F_hub(k),report.SF(k),report.belt_life_km(k));
    end
    fprintf(fid,'\n## Suggestions\n\n');
    for i = 1:numel(report.suggestions)
        fprintf(fid,'- %s\n',report.suggestions{i});
    end
    fclose(fid);
    fprintf('  Saved: VALIDATION_REPORT.md\n');
end

%% 3. Git commit + push ───────────────────────────────────────────────────────
score_val = 0;
if nargin >= 3 && isfield(report,'score'), score_val = report.score; end
commit_msg = sprintf('FEAD update Score=%d/100  %s',score_val,datestr(now,'yyyy-mm-dd HH:MM'));

[~] = system(sprintf('git -C "%s" remote set-url origin "%s"',REPO_DIR,REMOTE));
[~] = system(sprintf('git -C "%s" add -A',REPO_DIR));
[s3,r3] = system(sprintf('git -C "%s" commit -m "%s"',REPO_DIR,commit_msg));
[s4,r4] = system(sprintf('git -C "%s" push origin main',REPO_DIR));

if s4 == 0 || contains(r4,'main -> main')
    fprintf('\n  GitHub: https://github.com/ricardomessi/fead-matlab\n');
    fprintf('  Commit: %s\n',strtrim(r3));
    fprintf('✅ Push successful.\n');
else
    fprintf('⚠  Push output: %s\n',strtrim(r4));
end
fprintf('=== Done ===\n\n');
end
