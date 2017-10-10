% Verifies if the coupled parameters are the minimal parameters by
% constructing the dynamics model with two sets of decoupled parameters
% which give the same coupled parameters

clear all; close all; clc;

statevar_1 = load('statevar_1.dat');
statevar_2 = load('statevar_2.dat');

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
hL = legend('Set-1', 'Set-2a');
set(hL,'Position', [0.05 0.5 0.05 0.05], 'Units', 'normalized');
set(gcf,'units','pixels','position',[0, 0, 1366, 768]);