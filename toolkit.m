function res = toolkit(name, text, headInfo)
% a tool for compiling or other use.
% input
% name: type of function, i.e. array, matrix or others
% text: the absolute data
% headInfo: the infomation that need to be display before data

% output
% res: return result
res = '';
if strcmp(name, 'array')
    len = length(text);
    for i = 1:len
        str = sprintf(' %d ', text(i));
        res = strcat(res, str);
    end
elseif strcmp(name, 'matrix')
    [m, n] = size(text);
    for i = 1:m
        for j = 1:n
            res = sprintf('%s %d ', res, text(i, j));
        end
        res = sprintf('%s \n ', res);
    end
end
res = sprintf('%s \n %s', headInfo, res);
disp(res);
end