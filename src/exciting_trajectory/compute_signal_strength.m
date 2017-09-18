function inverse_signal_strength = compute_signal_strength(signal)
    [num_instants, num_signals] = size(signal);
%     inverse_signal_strength = 0;
%     for curr_signal_index = 1 : num_signals
%         curr_signal = signal(:, curr_signal_index);
%         curr_signal(curr_signal == 0) = [];
%         inverse_signal_strength = inverse_signal_strength + ...
%                             num_instants / sum(curr_signal.^2)
%     end
    inverse_signal_strength = (num_instants * num_signals) / sum(sum(signal.^2, 1), 2);
end

