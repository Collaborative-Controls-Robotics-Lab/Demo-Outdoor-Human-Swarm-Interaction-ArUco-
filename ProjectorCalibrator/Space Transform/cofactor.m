function X = cofactor(H)

[n,m] = size(H);
X = H;
for i = 1:n
    for j = 1:m
        ith = 1:n;
        jth = 1:m;
        ith(i) = [];
        jth(j) = [];
%         keyboard
        X(i,j) = (-1)^(i+j)*det(H(ith,jth));
    end
end
end