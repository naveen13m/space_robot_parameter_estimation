function inverse_signal_strength = compute_signal_strength(signal)
    [num_instants, num_signals] = size(signal);
    inverse_signal_strength = (num_instants * num_signals) / sum(sum(signal.^2, 1), 2);
end

