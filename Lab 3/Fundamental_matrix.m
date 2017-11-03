function step8 = Fundamental_matrix(v1,v2)
pn = size(v1,2);
U = zeros(pn,8);
for i = 1 : pn
    x = v1(1,i); %x_idash
    x_ = v2(1,i); % x_i
    y = v1(2,i); %y_idash
    y_ = v2(2,i); %y_i
    U(i,:) = [x*x_, y*x_, x_, x*y_, y*y_, y_, x, y];
end
F_v = -U\ones(pn,1);
step8 = reshape([F_v;1],[3,3])';
end