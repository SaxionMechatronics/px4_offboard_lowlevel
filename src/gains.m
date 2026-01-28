%% 4th order: h1
A4 = [zeros(3,1), eye(3); zeros(1,4)];
B4 = [zeros(3,1);1];

K1 = place(A4,B4,[-1 -2 -1.3 -2.4]);

%% 3rd order: h3
A3 = [zeros(2,1), eye(2); zeros(1,3)];
B3 = [zeros(2,1);1];

K3 = place(A3,B3,[-1 -2 -1.3]);
%% 2nd order: h2,h4
A2 = [zeros(1,1), eye(1); zeros(1,2)];
B2 = [zeros(1,1);1];

K2 = place(A2,B2,[-1 -2]);
K4 = place(A2,B2,[-1 -2]);

