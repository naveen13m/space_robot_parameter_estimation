clear all; close all; clc;

statevar_1 = load('statevar_2a.dat');
statevar_2 = load('statevar_2b.dat');

[num_rows, num_cols] = size(statevar_1);

figure();
text = {'Base x-position', 'Base y-position', 'Base z-position', 'Base z-euler angle', ...
    'Base x-euler angle', 'Base y-euler angle', 'Joint-1 position', 'Joint-2 position', ...
    'Base x-vel', 'Base y-vel', 'Base z-vel', 'Base z-euler rate', ...
    'Base x-euler rate', 'Base y-euler rate', 'Joint-1 speed', 'Joint-2 speed'};
for curr_col = 1 : num_cols - 1
    subplot(4,4,curr_col);
    plot(1:1:num_rows, statevar_1(:, curr_col), 1:1:num_rows, statevar_2(:, curr_col));
    grid on;
    title(text{curr_col});
end
hL = legend('Set-2a', 'Set-2b');
set(hL,'Position', [0.05 0.5 0.05 0.05], 'Units', 'normalized');
set(gcf,'units','pixels','position',[0, 0, 1366, 768]);

tor_1 = load('tor_2a.dat');
tor_2 = load('tor_2b.dat');

[num_rows, num_cols] = size(tor_1);

figure();
text = {'Joint-1 torque', 'Joint-2 toruqe'};
for curr_col = 1 : num_cols
    subplot(1,2,curr_col);
    plot(1:1:num_rows, tor_1(:, curr_col), 1:1:num_rows, tor_2(:, curr_col));
    grid on;
    title(text{curr_col});
end
hL = legend('Set-2a', 'Set-2b');
set(hL,'Position', [0.05 0.5 0.05 0.05], 'Units', 'normalized');
set(gcf,'units','pixels','position',[0, 0, 1366, 768]);