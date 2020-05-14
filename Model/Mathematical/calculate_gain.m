%author Akshay S Rao

%program to convert continuous time system to discrete time system

% A matrix
A = [0.00000    0.00000    1.00000    0.00000;
    0.00000    0.00000    0.00000    1.00000;
    0.00000   60.72967   -7.83691    7.83691;
    0.00000   64.59751    4.42974   -4.42974];

% B matrix
    B = [0.00000;
    0.00000;
   10.17473;
   -5.75117];

%A2 matrix
 A2 =[ 0.00000    1.00000;
    0.00000  -12.16150];

 %B2 matrix

 B2 =[0.00000;
  -5.70172];

%Q matrix
diagonal = [5000 5e8  50 1e4 ];
diagonal2 = [10000 100];

Q= diag(diagonal);
Q2 = diag(diagonal2);
R=1;

%sampling time
Ts=5*10^(-3);

% c matrix
C = zeros(2,4);
c2 = zeros(2,2);

% d matrix
D = 0;
d2 = 0;

%continuous time system
sysc = ss(A,B,C,D);
sysyawc = ss(A2, B2, c2, d2);

%discrete time system
sysyawd = c2d(sysyawc, Ts);
sysd = c2d(sysc, Ts); 

%returns required gain matrix
[K,e,a]=dlqr(sysd.A,sysd.B,Q,R)
[K1,e1,a1]=dlqr(sysyawd.A,sysyawd.B,Q2,R)