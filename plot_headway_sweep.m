% plot_headway_sweep.m  -- figure from headway_sweep.csv (no need to rerun the sweep)
fid = fopen('headway_sweep.csv'); fgetl(fid);
C = textscan(fid, '%s %f %f', 'Delimiter', ','); fclose(fid);
names = C{1}; H = [C{2} C{3}];
hmax = 1.6; Hp = H; Hp(isnan(Hp)) = hmax + 0.15;      % "no safe h found" drawn past the grid
figure('Color', 'w', 'Position', [100 100 760 440]);
b = barh(Hp); grid on; box on;
set(gca, 'YTick', 1:numel(names), 'YTickLabel', names, 'YDir', 'reverse');
xlim([0 hmax + 0.35]);
hold on; plot([hmax hmax], [0.4 numel(names)+0.6], 'k--', 'HandleVisibility', 'off');
for m = 1:numel(names)
  for L = 1:2
    if isnan(H(m,L))
      text(hmax + 0.17, m + (L-1.5)*0.3, '>1.6', 'FontSize', 8, 'HorizontalAlignment', 'left');
    end
  end
end
xlabel('Minimum safe time headway h [s]  (gap \geq 1 m, \gamma_a \leq 1.01, 10 runs)');
legend({'no V2X loss', 'Gilbert-Elliott V2X loss'}, 'Location', 'northeast');
title('Sensing scheme vs minimum safe headway (stop-and-go, empirical)');
print(gcf, 'headway_sweep.png', '-dpng', '-r150');
