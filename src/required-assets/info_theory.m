%% info_theory_topspin.m
% ME5314 E2 - Information-theoretic analysis of the three recorded topspin strokes
%
% Reads  : Top_spin_SegmentPosition.csv  (Xsens MVN, 60 Hz)
%          looks first in ../motion-data/ (relative to this script), then in the script folder
% Writes : figures_info_theory/*.png  (300 dpi)  and  figures_info_theory/results_info_theory.txt
% Needs  : MATLAB R2016b or newer, base MATLAB only (betainv from Statistics Toolbox is optional)
%
% Pipeline (same as topspin_affordance_scene.py):
%   Xsens (x,y,z) -> Vizard (-y, z, x);  origin shift on the initial pelvis (X,Z only)
%   paddle proxy = left wrist + 0.10 m along forearm->wrist;  speed = |dp|*60
%   ball(d) = P_ref + d*U_OUT  (P_ref = proxy at max speed, U_OUT = horizontal pelvis->proxy)

clear; clc; close all;

%% 1. Parameters ---------------------------------------------------------
FS = 60;                      % Hz
RC = 0.075 + 0.020;           % contact threshold (m): paddle radius + ball radius
CONDS = [0.00 0.08 0.15 0.30];% ball offsets d (m)  -> C1..C4
FAST = 4.0;                   % m/s, "fast" proxy-speed event
HW = 15;                      % half window (frames) around each stroke peak (+-0.25 s)
PADDLE_OFFSET = 0.10;         % m beyond the wrist
SPEED_EDGES = 0:1:8;          % m/s, 1 m/s bins (8 bins) for entropy and KL
PSEUDO = 0.5;                 % pseudo-count per bin for KL (Jeffreys smoothing)
NSTROKE = 3;                  % number of recorded strokes
H_OBS = 0.85; F_OBS = 0.20;   % ILLUSTRATIVE observer hit / false-alarm rates (NOT data)

scriptDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(scriptDir, '..', 'motion-data');
fpos = fullfile(dataDir, 'Top_spin_SegmentPosition.csv');
if ~exist(fpos, 'file')
    dataDir = scriptDir;
    fpos = fullfile(dataDir, 'Top_spin_SegmentPosition.csv');
end
outDir = fullfile(scriptDir, 'figures_info_theory');
if ~exist(outDir, 'dir'), mkdir(outDir); end
diary off;
if exist(fullfile(outDir, 'results_info_theory.txt'), 'file')
    delete(fullfile(outDir, 'results_info_theory.txt'));
end
diary(fullfile(outDir, 'results_info_theory.txt'));

savefig_png = @(fig, name) print(fig, fullfile(outDir, [name '.png']), '-dpng', '-r300');
rn = @(x) sqrt(sum(x.^2, 2));                       % row-wise norm
Hbits = @(p) -sum(p(p > 0) .* log2(p(p > 0)));      % Shannon entropy in bits
Hnats = @(p) -sum(p(p > 0) .* log(p(p > 0)));       % Shannon entropy in nats

%% 2. Load data and build the paddle proxy ------------------------------
T = readtable(fpos, 'VariableNamingRule', 'preserve');
n = height(T);
vars = T.Properties.VariableNames;

% --- Diagnóstico: imprime las primeras cabeceras reales ---
fprintf('Loaded %d rows, %d columns. First column names:\n', n, numel(vars));
for i = 1:min(numel(vars), 20)
    fprintf('  [%2d] "%s"\n', i, vars{i});
end

% Normaliza: minúsculas, quita unidades entre [] o (), deja solo alfanuméricos
normKey = cellfun(@(s) regexprep(lower(s), '\[[^\]]*\]|\([^)]*\)', ''), vars, 'UniformOutput', false);
normKey = cellfun(@(s) regexprep(s, '[^a-z0-9]', ''), normKey, 'UniformOutput', false);

findCol = @(seg, ax) findColIdx(normKey, seg, ax);

% getp(seg) devuelve [-y, z, x]  (Xsens -> Vizard)
getp = @(seg) [-T{:, findCol(seg, 'y')}, ...
                T{:, findCol(seg, 'z')}, ...
                T{:, findCol(seg, 'x')}];

% --- Construcción del proxy (Xsens -> Vizard) ---
pelvis = getp('Pelvis');
hand   = getp('Left Hand');
fore   = getp('Left Forearm');

shift  = repmat([-pelvis(1,1) 0 -pelvis(1,3)], n, 1);
pelvis = pelvis + shift;
hand   = hand   + shift;
fore   = fore   + shift;

proxy  = hand + PADDLE_OFFSET * (hand - fore) ./ repmat(rn(hand - fore), 1, 3);
sp     = [0; rn(diff(proxy)) * FS];      % proxy speed (m/s)
t      = (0:n-1)' / FS;

fprintf('Frames: %d | duration %.3f s | rate %d Hz\n', n, (n-1)/FS, FS);

%% 3. Detect the three strokes (greedy peak picking, no toolbox) --------
s = sp; pk = zeros(1, NSTROKE);
for k = 1:NSTROKE
    [~, i] = max(s);
    pk(k) = i;
    s(max(1, i-30):min(n, i+30)) = 0;
end
pk = sort(pk);
win = cell(1, NSTROKE);
for k = 1:NSTROKE
    win{k} = (max(1, pk(k)-HW):min(n, pk(k)+HW))';
end
[~, iref] = max(sp);                                % reference frame (as in the scene)
Pref = proxy(iref, :);
out = Pref - pelvis(iref, :); out(2) = 0;
U = out / norm(out);
balls = zeros(numel(CONDS), 3);
for k = 1:numel(CONDS), balls(k, :) = Pref + CONDS(k) * U; end

fprintf('\n== STROKES (frame numbers are 0-based, as in the scene) ==\n');
fprintf('Stroke | frame | t (s) | peak speed (m/s) | impact point x y z (m)\n');
for k = 1:NSTROKE
    p = proxy(pk(k), :);
    fprintf('  %d    | %4d  | %.3f | %.2f             | %.3f %.3f %.3f\n', ...
        k, pk(k)-1, (pk(k)-1)/FS, sp(pk(k)), p(1), p(2), p(3));
end
fprintf('Reference frame (max speed): %d (t = %.3f s, %.2f m/s)\n', iref-1, (iref-1)/FS, sp(iref));
imp = proxy(pk, :);
dd = [norm(imp(1,:)-imp(2,:)) norm(imp(1,:)-imp(3,:)) norm(imp(2,:)-imp(3,:))];
fprintf('Inter-stroke impact distances (1-2, 1-3, 2-3): %.3f %.3f %.3f m\n', dd);
mu = mean(imp, 1);
fprintf('RMS radial spread of impact points about their mean: %.3f m (R_c = %.3f m)\n', ...
    sqrt(mean(sum((imp - repmat(mu, NSTROKE, 1)).^2, 2))), RC);

%% 4. Ball-racket distances and per-stroke criterion --------------------
dist = zeros(n, numel(CONDS));
for k = 1:numel(CONDS)
    dist(:, k) = rn(proxy - repmat(balls(k, :), n, 1));
end
dmin = zeros(numel(CONDS), NSTROKE);
for k = 1:numel(CONDS)
    for j = 1:NSTROKE
        dmin(k, j) = min(dist(win{j}, k));
    end
end
icpt = dmin <= RC;                                   % 4 x 3 logical
fprintf('\n== d_min (m) per condition (rows C1..C4) and stroke (cols 1..3); * = intercept ==\n');
for k = 1:numel(CONDS)
    fprintf('C%d (d = %.2f m): ', k, CONDS(k));
    for j = 1:NSTROKE
        if icpt(k, j), mark = '*'; else, mark = ' '; end
        fprintf('%.3f%s  ', dmin(k, j), mark);
    end
    fprintf('\n');
end

%% 5. Shannon entropy of the speed occupancy distribution ---------------
cAll = histcounts(sp, SPEED_EDGES);
cS = zeros(NSTROKE, numel(SPEED_EDGES) - 1);
for j = 1:NSTROKE, cS(j, :) = histcounts(sp(win{j}), SPEED_EDGES); end
pAll = cAll / sum(cAll);
Hb = zeros(1, NSTROKE + 1); Hn = Hb;
Hb(1) = Hbits(pAll); Hn(1) = Hnats(pAll);
for j = 1:NSTROKE
    pj = cS(j, :) / sum(cS(j, :));
    Hb(j+1) = Hbits(pj); Hn(j+1) = Hnats(pj);
end
fprintf('\n== ENTROPY of proxy-speed occupancy (1 m/s bins, max = %.2f bits) ==\n', log2(numel(SPEED_EDGES)-1));
names = {'whole trial', 'stroke 1', 'stroke 2', 'stroke 3'};
for j = 1:numel(names)
    fprintf('%-12s H = %.3f bits = %.3f nats\n', names{j}, Hb(j), Hn(j));
end
fprintf('1 nat = %.4f bits ; 1 bit = %.4f nats\n', 1/log(2), log(2));

%% 6. Kullback-Leibler divergence between strokes (bits) ----------------
KL = zeros(NSTROKE);
for a = 1:NSTROKE
    for b = 1:NSTROKE
        p = (cS(a, :) + PSEUDO) / sum(cS(a, :) + PSEUDO);
        q = (cS(b, :) + PSEUDO) / sum(cS(b, :) + PSEUDO);
        KL(a, b) = sum(p .* log2(p ./ q));
    end
end
fprintf('\n== KL divergence D(row || col) in bits (pseudo-count %.1f) ==\n', PSEUDO);
disp(KL);

%% 7. Conditional problems with two events -------------------------------
% 7.1 frames: A = fast (speed >= FAST), B = contact (dist <= RC), per condition
fast = sp >= FAST;
fprintf('\n== P1: frames, A = fast (>= %.1f m/s), B = contact ==  (fast frames: %d of %d)\n', FAST, sum(fast), n);
for k = 1:numel(CONDS)
    con = dist(:, k) <= RC;
    M = [sum(fast & con), sum(fast & ~con); sum(~fast & con), sum(~fast & ~con)];
    pj = M / sum(M(:)); pa = sum(pj, 2); pb = sum(pj, 1);
    pab = pa * pb;
    mi = sum(pj(pj > 0) .* log2(pj(pj > 0) ./ pab(pj > 0)));
    pBgA = M(1,1) / sum(M(1,:));
    if sum(M(:,1)) > 0, pAgB = M(1,1) / sum(M(:,1)); else, pAgB = NaN; end
    if pb(1) > 0, sur = -log2(pb(1)); else, sur = Inf; end
    fprintf('C%d: [fast&con %d, fast&no %d; slow&con %d, slow&no %d] P(B)=%.4f P(B|A)=%.4f P(A|B)=%.4f I(A;B)=%.4f bits  surprisal(B)=%.2f bits\n', ...
        k, M(1,1), M(1,2), M(2,1), M(2,2), pb(1), pBgA, pAgB, mi, sur);
end

% 7.2 stroke trials: A = near (d <= 0.08) vs far (d >= 0.15), B = intercept
near = CONDS <= 0.08;
nearInt = sum(sum(icpt(near, :)));  nearNo = sum(sum(~icpt(near, :)));
farInt = sum(sum(icpt(~near, :)));  farNo = sum(sum(~icpt(~near, :)));
M = [nearInt nearNo; farInt farNo];
pj = M / sum(M(:)); pa = sum(pj, 2); pb = sum(pj, 1); pab = pa * pb;
mi = sum(pj(pj > 0) .* log2(pj(pj > 0) ./ pab(pj > 0)));
fprintf('\n== P2: 12 stroke-trials, A = near/far, B = intercept ==\n');
fprintf('[near&int %d, near&no %d; far&int %d, far&no %d]\n', M(1,1), M(1,2), M(2,1), M(2,2));
fprintf('P(int)=%.3f  P(int|near)=%.3f  P(int|far)=%.3f  P(near|int)=%.3f\n', pb(1), M(1,1)/sum(M(1,:)), M(2,1)/sum(M(2,:)), M(1,1)/sum(M(:,1)));
fprintf('H(int)=%.3f bits  I(near;int)=%.3f bits\n', Hbits(pb), mi);

% 7.3 Bayes: Beta-Binomial posterior of P(intercept) per condition (uniform prior)
fprintf('\n== P3: Beta(1+s, 1+n-s) posterior of P(intercept) per condition (n = %d strokes) ==\n', NSTROKE);
postA = zeros(1, numel(CONDS)); postB = postA; postMean = postA; ci = nan(numel(CONDS), 2);
for k = 1:numel(CONDS)
    sk = sum(icpt(k, :));
    postA(k) = 1 + sk; postB(k) = 1 + NSTROKE - sk;
    postMean(k) = postA(k) / (postA(k) + postB(k));
    try
        ci(k, :) = [betainv(0.025, postA(k), postB(k)), betainv(0.975, postA(k), postB(k))];
    catch
        % Statistics Toolbox not available: credible interval left as NaN
    end
    fprintf('C%d: s=%d/%d  Beta(%d,%d)  mean=%.3f  95%% CI [%.3f, %.3f]  H(mean)=%.3f bits\n', ...
        k, sk, NSTROKE, postA(k), postB(k), postMean(k), ci(k,1), ci(k,2), Hbits([postMean(k) 1-postMean(k)]));
end

% 7.4 information gained by two extra strokes: KL( p3 || p1 ) per condition
fprintf('\n== P4: information gain from strokes 1 and 3 (KL of Bernoulli predictive, bits / nats) ==\n');
for k = 1:numel(CONDS)
    s1 = icpt(k, 2);                                  % reference stroke only (stroke 2)
    q = (s1 + 1) / 3;                                 % predictive after 1 stroke
    p = (sum(icpt(k, :)) + 1) / 5;                    % predictive after 3 strokes
    klb = p * log2(p / q) + (1 - p) * log2((1 - p) / (1 - q));
    fprintf('C%d: q=%.3f  p=%.3f  KL(p||q)=%.4f bits = %.4f nats\n', k, q, p, klb, klb * log(2));
end

% 7.5 ILLUSTRATIVE ideal-observer example (assumed rates, NOT participant data)
prior = 0.5;                                          % P(feasible): C1,C2 feasible, C3,C4 not
pj = [prior * H_OBS, prior * (1 - H_OBS); (1 - prior) * F_OBS, (1 - prior) * (1 - F_OBS)];
pyes = pj(1,1) + pj(2,1);
pa = sum(pj, 2); pb = sum(pj, 1); pab = pa * pb;
mi = sum(pj(pj > 0) .* log2(pj(pj > 0) ./ pab(pj > 0)));
zf = @(p) sqrt(2) * erfinv(2 * p - 1);
fprintf('\n== P5: ILLUSTRATIVE observer, hit = %.2f, false alarm = %.2f, prior = %.2f ==\n', H_OBS, F_OBS, prior);
fprintf('P(yes)=%.3f  P(feasible|yes)=%.3f  P(feasible|no)=%.3f  I(F;J)=%.3f bits  d''=%.2f\n', ...
    pyes, pj(1,1)/pyes, pj(1,2)/(1 - pyes), mi, zf(H_OBS) - zf(F_OBS));

diary off;

%% 8. Figures --------------------------------------------------------------
cols = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];

% Fig 1: speed time series with the three stroke windows
f1 = figure('Color', 'w', 'Position', [100 100 900 380]); hold on;
ymax = max(sp) * 1.1;
for j = 1:NSTROKE
    patch([t(win{j}(1)) t(win{j}(end)) t(win{j}(end)) t(win{j}(1))], [0 0 ymax ymax], cols(j,:), ...
        'FaceAlpha', 0.15, 'EdgeColor', 'none');
end
plot(t, sp, 'k-', 'LineWidth', 1.2);
plot(t(pk), sp(pk), 'rv', 'MarkerFaceColor', 'r');
plot([0 t(end)], [FAST FAST], 'k--');
for j = 1:NSTROKE
    text(t(pk(j)), sp(pk(j)) + 0.4, sprintf('S%d', j), 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end
xlabel('Time (s)'); ylabel('Paddle-proxy speed (m/s)'); ylim([0 ymax]);
title('Paddle-proxy speed and the three topspin strokes (windows = peak \pm 0.25 s)');
grid on; box on;
savefig_png(f1, 'Fig1_speed_timeseries');

% Fig 2: speed-occupancy distributions
ctr = (SPEED_EDGES(1:end-1) + SPEED_EDGES(2:end)) / 2;
f2 = figure('Color', 'w', 'Position', [100 100 900 600]);
subplot(2, 2, 1); bar(ctr, pAll, 1, 'FaceColor', [0.4 0.4 0.4]);
title(sprintf('Whole trial: H = %.2f bits = %.2f nats', Hb(1), Hn(1)));
for j = 1:NSTROKE
    subplot(2, 2, j + 1); bar(ctr, cS(j, :) / sum(cS(j, :)), 1, 'FaceColor', cols(j, :));
    title(sprintf('Stroke %d: H = %.2f bits = %.2f nats', j, Hb(j+1), Hn(j+1)));
end
for q = 1:4
    subplot(2, 2, q); xlabel('Proxy speed (m/s)'); ylabel('Probability'); ylim([0 0.5]); grid on;
end
savefig_png(f2, 'Fig2_speed_histograms');

% Fig 3: top view of the proxy path, impact points, balls and R_c circles
f3 = figure('Color', 'w', 'Position', [100 100 700 650]); hold on;
plot(proxy(:, 1), proxy(:, 3), '-', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
for j = 1:NSTROKE
    plot(proxy(win{j}, 1), proxy(win{j}, 3), '-', 'Color', cols(j, :), 'LineWidth', 2);
    plot(proxy(pk(j), 1), proxy(pk(j), 3), 'o', 'MarkerFaceColor', cols(j, :), 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
    text(proxy(pk(j), 1), proxy(pk(j), 3) - 0.03, sprintf('S%d', j), 'HorizontalAlignment', 'center');
end
for k = 1:numel(CONDS)
    rectangle('Position', [balls(k,1) - RC, balls(k,3) - RC, 2*RC, 2*RC], 'Curvature', [1 1], 'EdgeColor', 'r', 'LineStyle', '--');
    plot(balls(k, 1), balls(k, 3), 's', 'MarkerFaceColor', [1 0.8 0], 'MarkerEdgeColor', 'k');
    text(balls(k, 1) + 0.01, balls(k, 3) + 0.03, sprintf('C%d', k), 'Color', 'r', 'FontWeight', 'bold');
end
axis equal; grid on; box on;
xlabel('X lateral right (m)'); ylabel('Z forward (m)');
title('Top view: strokes, impact points and balls C1-C4 (dashed: R_c = 0.095 m)');
savefig_png(f3, 'Fig3_topview_impacts');

% Fig 4: ball-racket distance versus time for each condition
f4 = figure('Color', 'w', 'Position', [100 100 950 650]);
for k = 1:numel(CONDS)
    subplot(2, 2, k); hold on;
    for j = 1:NSTROKE
        patch([t(win{j}(1)) t(win{j}(end)) t(win{j}(end)) t(win{j}(1))], [0 0 1.5 1.5], cols(j,:), 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    end
    plot(t, dist(:, k), 'k-', 'LineWidth', 1.1);
    plot([0 t(end)], [RC RC], 'r--');
    title(sprintf('C%d: d = %.2f m', k, CONDS(k)));
    xlabel('Time (s)'); ylabel('Ball-proxy distance (m)'); ylim([0 1.5]); grid on; box on;
end
savefig_png(f4, 'Fig4_distance_vs_time');

% Fig 5: KL divergence matrix
f5 = figure('Color', 'w', 'Position', [100 100 520 450]);
imagesc(KL); colormap(flipud(gray)); colorbar;
set(gca, 'XTick', 1:NSTROKE, 'YTick', 1:NSTROKE);
for a = 1:NSTROKE
    for b = 1:NSTROKE
        text(b, a, sprintf('%.3f', KL(a, b)), 'HorizontalAlignment', 'center', 'Color', 'r', 'FontWeight', 'bold');
    end
end
xlabel('Q (stroke)'); ylabel('P (stroke)');
title('D_{KL}(P || Q) of speed occupancy (bits)');
savefig_png(f5, 'Fig5_KL_matrix');

% Fig 6: Beta posteriors of P(intercept)
f6 = figure('Color', 'w', 'Position', [100 100 700 420]); hold on;
xs = linspace(0.001, 0.999, 500);
ccol = [0 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19; 0.5 0.5 0.5];
for k = 1:numel(CONDS)
    a = postA(k); b = postB(k);
    pdfk = xs .^ (a - 1) .* (1 - xs) .^ (b - 1) * gamma(a + b) / (gamma(a) * gamma(b));
    plot(xs, pdfk, 'LineWidth', 2, 'Color', ccol(k, :), 'DisplayName', sprintf('C%d (d=%.2f m): Beta(%d,%d)', k, CONDS(k), a, b));
end
xlabel('P(intercept)'); ylabel('Posterior density'); legend('Location', 'north'); grid on; box on;
title('Posterior of P(intercept) after the three strokes (uniform prior)');
savefig_png(f6, 'Fig6_beta_posteriors');

% Fig 7: entropy in bits and nats
f7 = figure('Color', 'w', 'Position', [100 100 650 400]);
bar([Hb' Hn']);
set(gca, 'XTickLabel', names); ylabel('Entropy'); legend('bits', 'nats', 'Location', 'southeast'); grid on;
title('Entropy of proxy-speed occupancy (1 m/s bins)');
savefig_png(f7, 'Fig7_entropy_bits_nats');

fprintf('\nDone. Figures and results saved in: %s\n', outDir);


function idx = findColIdx(normKey, seg, ax)
% Busca el índice de la columna para un segmento y eje, sin distinguir mayúsculas,
% ignorando unidades entre [] o () y separadores. Ej.: "Pelvis_y", "Pelvis Y", "Pelvis y (m)"
% se normalizan todos a "pelvisy".
segN = lower(regexprep(seg, '[^A-Za-z0-9]', ''));
key  = [segN lower(ax)];
idx  = find(strcmp(normKey, key), 1);
if isempty(idx)
    % Coincidencia laxa: contiene el segmento, termina en el eje y NO es cuaternión (q...)
    idx = find(contains(normKey, segN) & ...
        endsWith(normKey, lower(ax)) & ...
        ~contains(normKey, 'q'), 1);
end
if isempty(idx)
    error('No encontré columna para segmento "%s", eje "%s". Claves disponibles: %s', ...
        seg, ax, strjoin(normKey, ', '));
end
end