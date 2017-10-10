function statevar = filter_kin_data(statevar)
%     disp('Inside function filter_kin_data');
    df = designfilt('lowpassfir','FilterOrder',10,'CutoffFrequency',0.05);
    D = mean(grpdelay(df));
    for i=1:size(statevar,2)
        mod_data = statevar(:,i);
        filt_data = filter(df,[mod_data; zeros(D,1)]);
        filt_data = filt_data(D+1:end);
        if i ~= 1 || i ~= 2 || i~=3
            filt_data(1:30,1)=mod_data(1:30,1);
            filt_data((size(statevar,1)-30):(size(statevar,1)),1)=mod_data((size(statevar,1)-30):(size(statevar,1)),1);
        end
        statevar(:,i) = filt_data;
    end
end

