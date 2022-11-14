% Define skew symmetric matrix
function skewSym=skewSym(vec)
    skewSym = [0 -vec(3) vec(2);
               vec(3) 0 -vec(1);
               -vec(2) vec(1) 0];
end