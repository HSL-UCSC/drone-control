%% This function takes a vector and converts it into a skew symmetric matrix

function f = so3_hat(w)

[row,col] = size(w);

if (row~=3 && col~=1)
    display('dimension error! so3_hat only takes col vectors consisting of 3 elements');
    return;
end

f=[0, -w(3), w(2);
   w(3), 0, -w(1);
   -w(2), w(1), 0];
end
