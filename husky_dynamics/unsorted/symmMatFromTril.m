function mat = symmMatFromTril(vec)
    % vec - lower triangular entries, specified column-wise
    
    p = [1 1 -2*length(vec)];
    r = roots(p);
    dim = floor(max(r));
    mat = zeros(dim,dim);
    mat(tril(ones(dim,dim)) == 1) = vec;
    mat = mat+triu(mat',1);
end