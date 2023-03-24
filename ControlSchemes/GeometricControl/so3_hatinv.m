%% This function takes a 3 by 3 skew symmetric matrix and converts it into a vector.
function v = so3_hatinv(xi)
[row,col] = size(xi);

if (row~=3 && col~=3)
    display('dimension error! so3_hatinv only takes skew symmetric 3 by 3 matrices');
    return;
end

% if (abs(xi(1,1))> (0.00001)  && abs(xi(2,2))> (0.00001) && abs (xi(3,3))> (0.00001))
%     display('Martix in not skew! Diagonal entries are not "zero"!!');
%     return;
% end

v = [xi(3,2); xi(1,3); xi(2,1)];
end