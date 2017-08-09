function s = nd_sym(x,a)
    a = a(:).';
    format = repmat('%d_',[1 numel(a)]);
    x = [x format(1:end-1)];

    s = cellfun(@createCharArrayElement,num2cell(1:prod(a)),'UniformOutput',false);
    s = sym(reshape(s,a));

    function s = createCharArrayElement(k)
        [v{1:numel(a)}] = ind2sub(a,k);
        s = sprintf(x,v{:});
    end
end