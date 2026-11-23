clear; clc; close all;

FS = 60; RC = 0.095; CONDS = [0 0.08 0.15 0.30]; FAST = 4.0; HW = 15; PAD = 0.10;
SPEED_EDGES = 0:1:8; PSEUDO = 0.5; NSTROKE = 3; H_OBS = 0.85; F_OBS = 0.20;

scriptDir = fileparts(mfilename('fullpath'));
fpos = fullfile(scriptDir, '..', 'motion-data', 'Top_spin_SegmentPosition.csv');
if ~exist(fpos, 'file')
    fpos = fullfile(scriptDir, 'Top_spin_SegmentPosition.csv');
end
outDir = fullfile(scriptDir, 'figures_info_theory');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
fTxt = fullfile(outDir, 'results_info_theory.txt');
if exist(fTxt, 'file')
    delete(fTxt);
end
diary off;
diary(fTxt);

T = readtable(fpos, 'VariableNamingRule', 'preserve');
n = height(T);
fprintf('Loaded %d rows x %d cols\n', n, width(T));

vars = T.Properties.VariableNames;
normKey = cell(1, numel(vars));
for i = 1:numel(vars)
    s = lower(vars{i});
    s = regexprep(s, '\[[^\]]*\]|\([^)]*\)', '');
    s = regexprep(s, '[^a-z0-9]', '');
    normKey{i} = s;
end

pelvis = getpos(T, normKey, 'Pelvis');
hand = getpos(T, normKey, 'Left Hand');
fore = getpos(T, normKey, 'Left Forearm');

shift = repmat([-pelvis(1,1), 0, -pelvis(1,3)], n, 1);
pelvis = pelvis + shift;
hand = hand + shift;
fore = fore + shift;

arm = hand - fore;
proxy = hand + PAD * arm ./ repmat(sqrt(sum(arm.^2, 2)), 1, 3);
sp = [0; sqrt(sum(diff(proxy).^2, 2)) * FS];
t = (0:n-1)' / FS;
fprintf('Duration %.3f s | rate %d Hz\n', (n-1)/FS, FS);

s = sp;
pk = zeros(1, NSTROKE);
for k = 1:NSTROKE
    [m, i] = max(s);
    pk(k) = i;
    s(max(1, i-30):min(n, i+30)) = 0;
end
pk = sort(pk);

win = cell(1, NSTROKE);
for k = 1:NSTROKE
    win{k} = (max(1, pk(k)-HW):min(n, pk(k)+HW))';
end

[mref, iref] = max(sp);
Pref = proxy(iref, :);
outv = Pref - pelvis(iref, :);
outv(2) = 0;
U = outv / norm(outv);
balls = Pref + CONDS' * U;

fprintf('\nSTROKES:\n');
for k = 1:NSTROKE
    p = proxy(pk(k), :);
    fprintf('  %d | %4d | %.3f | %.2f | %.3f %.3f %.3f\n', ...
        k, pk(k)-1, (pk(k)-1)/FS, sp(pk(k)), p(1), p(2), p(3));
end

imp = proxy(pk, :);
mu = mean(imp, 1);
spread = sqrt(mean(sum((imp - repmat(mu, NSTROKE, 1)).^2, 2)));
fprintf('RMS spread: %.3f m\n', spread);

dist = zeros(n, numel(CONDS));
for k = 1:numel(CONDS)
    dist(:,k) = sqrt(sum((proxy - repmat(balls(k,:), n, 1)).^2, 2));
end

dmin = zeros(numel(CONDS), NSTROKE);
for k = 1:numel(CONDS)
    for j = 1:NSTROKE
        dmin(k,j) = min(dist(win{j}, k));
    end
end
icpt = dmin <= RC;

fprintf('\nd_min:\n');
for k = 1:numel(CONDS)
    fprintf('C%d:', k);
    for j = 1:NSTROKE
        if icpt(k,j)
            fprintf(' %.3f*', dmin(k,j));
        else
            fprintf(' %.3f ', dmin(k,j));
        end
    end
    fprintf('\n');
end

nBins = numel(SPEED_EDGES) - 1;
cAll = histcounts(sp, SPEED_EDGES);
cS = zeros(NSTROKE, nBins);
for j = 1:NSTROKE
    cS(j,:) = histcounts(sp(win{j}), SPEED_EDGES);
end
pAll = cAll / sum(cAll);
P = [pAll; cS ./ sum(cS, 2)];

Hb = zeros(1, NSTROKE+1);
Hn = zeros(1, NSTROKE+1);
for i = 1:(NSTROKE+1)
    p = P(i,:);
    Hb(i) = -sum(p .* log2(max(p, realmin)));
    Hn(i) = -sum(p .* log(max(p, realmin)));
end

names = {'whole', 'stroke 1', 'stroke 2', 'stroke 3'};
fprintf('\nENTROPY:\n');
for i = 1:numel(names)
    fprintf('%s: H = %.3f bits = %.3f nats\n', names{i}, Hb(i), Hn(i));
end

Ps = (cS + PSEUDO) ./ sum(cS + PSEUDO, 2);
KL = zeros(NSTROKE);
for a = 1:NSTROKE
    for b = 1:NSTROKE
        KL(a,b) = sum(Ps(a,:) .* log2(Ps(a,:) ./ Ps(b,:)));
    end
end
fprintf('\nKL:\n');
disp(KL);

fast = sp >= FAST;
fprintf('\nP1:\n');
for k = 1:numel(CONDS)
    con = dist(:,k) <= RC;
    M = [sum(fast & con), sum(fast & ~con); sum(~fast & con), sum(~fast & ~con)];
    pj = M / sum(M(:));
    pb = sum(pj, 1);
    mi = mutualInfoBits(M);
    pBgA = M(1,1) / sum(M(1,:));
    pAgB = M(1,1) / sum(M(:,1));
    fprintf('C%d: P(B)=%.4f P(B|A)=%.4f P(A|B)=%.4f I=%.4f\n', ...
        k, pb(1), pBgA, pAgB, mi);
end

near = CONDS <= 0.08;
M = [nnz(icpt(near,:)),  nnz(~icpt(near,:));
     nnz(icpt(~near,:)), nnz(~icpt(~near,:))];
pj = M / sum(M(:));
pb = sum(pj, 1);
mi = mutualInfoBits(M);
H_int = -sum(pb .* log2(max(pb, realmin)));
fprintf('\nP2:\n');
fprintf('P(int)=%.3f P(int|near)=%.3f P(int|far)=%.3f H=%.3f I=%.3f\n', ...
    pb(1), M(1,1)/sum(M(1,:)), M(2,1)/sum(M(2,:)), H_int, mi);

postA = 1 + sum(icpt, 2)';
postB = 1 + NSTROKE - sum(icpt, 2)';
postMean = postA ./ (postA + postB);
fprintf('\nP3:\n');
for k = 1:numel(CONDS)
    fprintf('C%d: mean=%.3f\n', k, postMean(k));
end

fprintf('\nP4:\n');
for k = 1:numel(CONDS)
    q = (icpt(k,2) + 1) / 3;
    p = (sum(icpt(k,:)) + 1) / 5;
    klb = p*log2(p/q) + (1-p)*log2((1-p)/(1-q));
    fprintf('C%d: q=%.3f p=%.3f KL=%.4f bits\n', k, q, p, klb);
end

prior = 0.5;
pj = [prior*H_OBS, prior*(1-H_OBS); (1-prior)*F_OBS, (1-prior)*(1-F_OBS)];
pyes = pj(1,1) + pj(2,1);
mi = mutualInfoBits(pj);
zf = sqrt(2) * erfinv(2*H_OBS - 1) - sqrt(2) * erfinv(2*F_OBS - 1);
fprintf('\nP5:\n');
fprintf('P(yes)=%.3f P(feasible|yes)=%.3f I=%.3f d''=%.2f\n', ...
    pyes, pj(1,1)/pyes, mi, zf);

diary off;

cols = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];

f1 = figure('Color','w','Position',[100 100 900 380]); hold on;
ymax = max(sp) * 1.1;
for j = 1:NSTROKE
    patch([t(win{j}(1)) t(win{j}(end)) t(win{j}(end)) t(win{j}(1))], ...
          [0 0 ymax ymax], cols(j,:), 'FaceAlpha', 0.15, 'EdgeColor', 'none');
end
plot(t, sp, 'k-', 'LineWidth', 1.2);
plot(t(pk), sp(pk), 'rv', 'MarkerFaceColor', 'r');
plot([0 t(end)], [FAST FAST], 'k--');
xlabel('Time (s)'); ylabel('Speed (m/s)'); ylim([0 ymax]); grid on; box on;
print(f1, fullfile(outDir, 'Fig1_speed_timeseries.png'), '-dpng', '-r300');

ctr = (SPEED_EDGES(1:end-1) + SPEED_EDGES(2:end)) / 2;
f2 = figure('Color','w','Position',[100 100 900 600]);
subplot(2,2,1); bar(ctr, pAll, 1, 'FaceColor', [0.4 0.4 0.4]);
title(sprintf('Whole: H = %.2f bits', Hb(1)));
for j = 1:NSTROKE
    subplot(2,2,j+1); bar(ctr, P(j+1,:), 1, 'FaceColor', cols(j,:));
    title(sprintf('Stroke %d: H = %.2f bits', j, Hb(j+1)));
end
for q = 1:4
    subplot(2,2,q); xlabel('Speed (m/s)'); ylabel('P'); ylim([0 0.5]); grid on;
end
print(f2, fullfile(outDir, 'Fig2_speed_histograms.png'), '-dpng', '-r300');

f3 = figure('Color','w','Position',[100 100 700 650]); hold on;
plot(proxy(:,1), proxy(:,3), '-', 'Color', [0.75 0.75 0.75]);
for j = 1:NSTROKE
    plot(proxy(win{j},1), proxy(win{j},3), '-', 'Color', cols(j,:), 'LineWidth', 2);
    plot(proxy(pk(j),1), proxy(pk(j),3), 'o', 'MarkerFaceColor', cols(j,:), 'MarkerEdgeColor', 'k');
end
for k = 1:numel(CONDS)
    rectangle('Position', [balls(k,1)-RC, balls(k,3)-RC, 2*RC, 2*RC], ...
              'Curvature', [1 1], 'EdgeColor', 'r', 'LineStyle', '--');
    plot(balls(k,1), balls(k,3), 's', 'MarkerFaceColor', [1 0.8 0], 'MarkerEdgeColor', 'k');
end
axis equal; grid on; box on; xlabel('X (m)'); ylabel('Z (m)');
print(f3, fullfile(outDir, 'Fig3_topview_impacts.png'), '-dpng', '-r300');

f4 = figure('Color','w','Position',[100 100 950 650]);
for k = 1:numel(CONDS)
    subplot(2,2,k); hold on;
    for j = 1:NSTROKE
        patch([t(win{j}(1)) t(win{j}(end)) t(win{j}(end)) t(win{j}(1))], ...
              [0 0 1.5 1.5], cols(j,:), 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    end
    plot(t, dist(:,k), 'k-');
    plot([0 t(end)], [RC RC], 'r--');
    title(sprintf('C%d', k));
    xlabel('Time (s)'); ylabel('Distance (m)'); ylim([0 1.5]); grid on; box on;
end
print(f4, fullfile(outDir, 'Fig4_distance_vs_time.png'), '-dpng', '-r300');

f5 = figure('Color','w','Position',[100 100 520 450]);
imagesc(KL); colormap(flipud(gray)); colorbar;
set(gca, 'XTick', 1:NSTROKE, 'YTick', 1:NSTROKE);
for a = 1:NSTROKE
    for b = 1:NSTROKE
        text(b, a, sprintf('%.3f', KL(a,b)), 'HorizontalAlignment', 'center', 'Color', 'r', 'FontWeight', 'bold');
    end
end
xlabel('Q'); ylabel('P'); title('KL divergence');
print(f5, fullfile(outDir, 'Fig5_KL_matrix.png'), '-dpng', '-r300');

f6 = figure('Color','w','Position',[100 100 700 420]); hold on;
xs = linspace(0.001, 0.999, 500);
ccol = [0 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19; 0.5 0.5 0.5];
for k = 1:numel(CONDS)
    a = postA(k); b = postB(k);
    y = xs.^(a-1) .* (1-xs).^(b-1) * gamma(a+b) / (gamma(a)*gamma(b));
    plot(xs, y, 'LineWidth', 2, 'Color', ccol(k,:));
end
xlabel('P(intercept)'); ylabel('Density'); grid on; box on; title('Beta posteriors');
print(f6, fullfile(outDir, 'Fig6_beta_posteriors.png'), '-dpng', '-r300');

f7 = figure('Color','w','Position',[100 100 650 400]);
bar([Hb(:) Hn(:)]);
set(gca, 'XTickLabel', names); ylabel('Entropy'); grid on;
print(f7, fullfile(outDir, 'Fig7_entropy_bits_nats.png'), '-dpng', '-r300');

fprintf('\nDone. Saved in: %s\n', outDir);

function p = getpos(T, normKey, seg)
    ix = findColIdx(normKey, seg, 'x');
    iy = findColIdx(normKey, seg, 'y');
    iz = findColIdx(normKey, seg, 'z');
    p = [-T{:,iy}, T{:,iz}, T{:,ix}];
end

function idx = findColIdx(normKey, seg, ax)
    segN = lower(regexprep(seg, '[^A-Za-z0-9]', ''));
    idx = find(strcmp(normKey, [segN lower(ax)]), 1);
    if isempty(idx)
        idx = find(contains(normKey, segN) & endsWith(normKey, lower(ax)) & ~contains(normKey, 'q'), 1);
    end
    if isempty(idx)
        error('No column for seg="%s" axis="%s".', seg, ax);
    end
end

function mi = mutualInfoBits(M)
    pj = M / sum(M(:));
    pa = sum(pj, 2);
    pb = sum(pj, 1);
    pab = pa * pb;
    mi = 0;
    for i = 1:size(pj,1)
        for j = 1:size(pj,2)
            if pj(i,j) > 0
                mi = mi + pj(i,j) * log2(pj(i,j) / pab(i,j));
            end
        end
    end
end