function inverse_signal_strength = compute_signal_strength(signal)
    [num_instants, num_signals] = size(signal);
    inverse_signal_strength = 0;
    for curr_signal = 1 : num_signals
        for curr_instant = 1 : num_instants
            if signal(curr_instant, curr_signal) > 10e-4
                inverse_signal_strength = inverse_signal_strength + ...
                    (1 / abs(signal(curr_instant, curr_signal)));
            end
        end
    end
    inverse_signal_strength = inverse_signal_strength / (num_instants * num_signals);
end

