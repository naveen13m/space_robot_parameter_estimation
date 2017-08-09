classdef precondition
    methods
        function [mat, varargout] = scaling(obj, mat, varargin)
            num_rows = size(mat, 1);
            scaling_factor = zeros(num_rows, 1);
            for curr_row_index = 1 : num_rows
                    scaling_factor(curr_row_index) = max(abs(mat(curr_row_index, :)));
                    mat(curr_row_index, :) = mat(curr_row_index, :) / ...
                                        scaling_factor(curr_row_index);
            end
            if nargin == 3
                varargout = mat2cell(rdivide(cell2mat(varargin), scaling_factor), num_rows, 1);
            end
            end
        end
end


