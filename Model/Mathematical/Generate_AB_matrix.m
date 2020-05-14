1;
pkg load control

## author Akshay S Rao

## Function : Medbot_dynamics()
## ----------------------------------------------------
## Input:   y - State Vector. 
##          u - Input to the system
##
## Output:  dy -  Derivative of State Vector. 
## Purpose: Calculates the value of the vector dy according to the equations which 
##          govern this system.
function dy = Medbot_dynamics(y, u)
 
  #Kb=0.005305;
  #Kt=0.005305;
  Kb=0.3821;
  Kt=0.495;
  n = 30;
  fm=0.0022;
  Rm = 2.17;
 #Kb=0.468;
  #Kt=0.317;
  #n = 30;
  #fm=0.0022;
  #Rm = 6.69;
  Jw = 10^(-5);
  g = 9.81;
  m = 0.048;
  M = 1.0;
  R = 0.0325;
  D=0.09;
  Jm = (m*R^2)/2;
  L = 0.030;
  Jpsi = M*L^2/3;#0.0030197143;
  W = 0.18;
  fw = 0.0;
  alpha= (n*Kt)/Rm;
  beta = ((n*Kt*Kb)/Rm)+fm;
  Jphi = M*(W^2+D^2)/12;
  
  
  Fpsi=-alpha*(u(1)+u(2))+2*beta*y(4)-2*beta*y(5);
  Ftheta = alpha*(u(1)+u(2))-2*(beta+fw)*y(4)+2*beta*y(5);
  A = ((2*m+M)*R^2)+(2*Jw)+(2*(n^2)*Jm)+2*(beta+fw);
  B1 = M*L*R*cos(y(2))-2*(n^2)*Jw;
  C = (M*(L^2))+Jpsi+(2*(n^2)*Jm);
  D = (.5*m*(W^2))+Jphi+(.5*((W/R)^2)*(Jw+(n^2)*Jm))+(M*(L*sin(y(2)))^2);
  
  
  
  dy(1,1) = y(4);
  dy(2,1) = y(5);
  dy(3,1) = y(6);
  dy(4,1) = ((Ftheta+M*L*R*((y(5))^2)*sin(y(2)))*C)-B1*(fw+M*g*L*sin(y(2))+M*(L^2)*(y(6)^2)*sin(y(2))*cos(y(2)))/(A*C-B1^2);
  dy(5,1) = (A*(Fpsi+M*g*L*sin(y(2))+M*(L^2)*(y(6)^2)*sin(y(2))*cos(y(2)))-B1*(Ftheta+M*L*R*((y(5))^2)*sin(y(2))))/(A*C-B1^2);
  dy(6,1) = ((W/(2*R))*alpha*(u(2)-u(1))-((W^2)/(2*R^2))*(beta+fw)-2*M*(L^2)*sin(y(2))*cos(y(2))*y(5)*y(6))/D;
endfunction

## Function : sim_Medbot()
## ----------------------------------------------------
## Input:  y0 - Initial condition 
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of MedBot system without 
##          any external input (u)
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0

function [t,y] = sim_Medbot(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6, y0)
  tspan = 0:0.1:10;                ## Initialize time step
  u = [0;0];                           ## No input
  [t,y] = ode45(@(t,y)Medbot_dynamics(y,u),tspan,y0);
endfunction

## Function : Medbot_AB_matrix()
## ----------------------------------------------------
##
## Output:  A - A matrix of system
##          B - B matrix of system
##          
## Purpose: Declare the A and B matrices in this function.

function [A,B] = Medbot_AB_matrix(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6,I,J,K)##x11,x12,x13,x21,x22,x23,x31,B4,B5,B6)##E,F,G,H,I,J,K)##)
    #Kb=0.005305;
  #Kt=0.005305;
  Kb=0.3821;
  Kt=0.495;
  n = 30;
  fm=0.0022;
  Rm = 6.17;
 #Kb=0.468;
  #Kt=0.317;
  #n = 30;
  #fm=0.0022;
  #Rm = 6.69;
  Jw = 10^(-5);
  g = 9.81;
  m = 0.048;
  M = 1.0;
  R = 0.0325;
  D=0.09;
  Jm = (m*R^2)/2;
  L = 0.030;
  Jpsi = M*L^2/3;#0.0030197143;
  W = 0.18;
  fw = 0.0;
  a= (n*Kt)/Rm;
  b = ((n*Kt*Kb)/Rm)+fm;
  Jphi = M*(W^2+D^2)/12;

##  A = [Z eye([2,2]); inv(E)*G  -inv(E)*F];
##  B = [0; 0; a*(M*L*L + Jpsi + M*L*R); -a*(2*m*R*R + M*R*L + M*R*R)];
  A2 = [0 1; 0 (-J/I)];
  B2 = [0; (-K/I)];
  
  A = [0 0   0 1  0   0 ; 
  0 0 0 0 1 0;
       0 0   0 0   0  1 ; 
       0 x11 0 x12 x13 0 ; 
       0 x21 0 x22 x23 0;
       0 0 0 0 0 x31];
  B = [0 ;
       0; 
       0;
       B4 ;
       B5 ;
       -B6];
   disp(A);
   disp(B);
##   disp('\n');
  disp(A2);
  disp(B2);
   
endfunction

## Function : pole_place_Medbot()
## ----------------------------------------------------
## Input:  
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of MedBot system with 
##          external input using the pole_placement controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using Pole Placement Technique.

function [t,y] = pole_place_Medbot(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6, y_setpoint, y0)
  [A,B] = Medbot_AB_matrix(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6);  ## Initialize A and B matrix 
  eigs = [-20+3i;-20-3i;-25;-30;-2+2i;-2-2i];                    ## Initialise desired eigenvalues
  K =   [   -0.0582   -1.9122   -0.0010   -0.0073   -0.0538   -0.0015;
   -0.0582   -1.9122    0.0010   -0.0073   -0.0538    0.0015]                ## Calculate K matrix for desired eigenvalues
  K = K*1000
  tspan = 0:0.1:10;                   ## Initialise time step 
  [t,y] = ode45(@(t,y)Medbot_dynamics(y, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : lqr_Medbot()
## ----------------------------------------------------
## Input:  
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of Medbot system with 
##          external input using the LQR controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using LQR

function [t,y] = lqr_Medbot(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6, y_setpoint, y0)

  [A,B] = Medbot_AB_matrix(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6);  ## Initialize A and B matrix 
  Q = [1 0 0 0 0 0; 
       0 60000 0 0 0 0; 
       0 0 1 0 0 0; 
       0 0 0 60000 0 0; 
       0 0 0 0 1 0; 
       0 0 0 0 0 1];                  ## Initialise desired eigenvalues
  R = [1 0; 0 1];    
  #Q=[1,0,0,0,0,0;
     #0,1,0,0,0,0;
     #0,0,1,0,0,0;
     #0,0,0,1,0,0;
     #0,0,0,0,1,0;
     #0,0,0,0,0,1]
     #R=[1,0;
        #0,1]
      
  K=lqr(A,B,Q,R);  ## Calculate K matrix for desired eigenvalues
  disp(eig(A-B*K));
  tspan = 0:0.1:10;                   ## Initialise time step 
  [t,y] = ode45(@(t,y)Medbot_dynamics(y, -K*(y-y_setpoint)),tspan,y0);
  disp(K);
endfunction

## Function : Medbot_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions.
function Medbot_main()
  
  #Kb=0.005305;
  #Kt=0.005305;
  Kb=0.3821;
  Kt=0.150;
  n = 30;
  fm=0.0022;
  Rm = 6.17;
 #Kb=0.468;
  #Kt=0.317;
  #n = 30;
  #fm=0.0022;
  #Rm = 6.69;
  Jw = 10^(-5);
  g = 9.81;
  m = 0.048;
  M = 1.0;
  R = 0.0325;
  D=0.09;
  Jm = (m*R^2)/2;
  L = 0.050;
  Jpsi = M*L^2/3;#0.0030197143;
  W = 0.18;
  fw = 0.0;
  alpha= (n*Kt)/Rm;
  beta = ((n*Kt*Kb)/Rm)+fm;
  Jphi = M*(W^2+D^2)/12;
  a= (n*Kt)/Rm;
  b = ((n*Kt*Kb)/Rm)+fm;
  E = [((2*m + m) *R*R + 2*Jw +(2*n*n*Jm)) (M*L*R-2*n*n*Jm); (M*L*R - 2*n*n*Jm) (M*L*L + Jpsi + 2*n*n*Jm)];
  F = 2*[b -b; -b b];
  G = [0 0;0 -M*g*L];
  H = [a a;-a -a];
  I = 0.5*m*W*W + Jphi +((W*W)*0.5*(Jw+(n*n*Jm))/(R*R)) ;
  J = ((W*W)*0.5*b)/(R*R);
  K = W*a*0.5/R;
  Z = zeros([2,2]);
  

##  
##  disp(inv(E));
 E11 = ((2*m+M)*(R^2))+(2*Jw)+(2*(n^2)*Jm);
  E12 = (M*L*R)-(2*(n^2)*Jm);
  E21 = (M*L*R)-(2*(n^2)*Jm);
  E22 = (M*(L^2))+Jpsi+(2*(n^2)*Jm);
## 
  x11 = -(g*M*L*E12)/((E11*E22)-(E12*E21));
  x12 = -2*((((beta+fw)*E22)+(beta*E12))/((E11*E22)-(E12*E21)));
  x13 = ((2*beta)*(E22+E12))/((E11*E22)-(E12*E21));
  x21 = (g*M*L*E11)/((E11*E22)-(E12*E21));
  x22 = 2*((((beta+fw)*E12)+(beta*E11))/((E11*E22)-(E12*E21)));
  x23 = -((2*beta)*(E11+E12))/((E11*E22)-(E12*E21));
  x31 = ((W^2)*(beta+fw))/((2*R^2)*((.5*m*(W^2))+Jphi+(.5*((W/R)^2)*(Jw+(n^2)*Jm))));
  B4 = alpha*(E22 + E12)/((E11*E22)-(E12*E21));
  B5 = -alpha*(E11 + E12)/((E11*E22)-(E12*E21));
  B6 = (W*alpha)/((2*R)*((.5*m*(W^2))+Jphi+(.5*((W/R)^2)*(Jw+(n^2)*Jm))));


  y0 = [1;0;0;0;0;0];
  y_setpoint = [0;0;1;0;0;0];
 [A,B] = Medbot_AB_matrix(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6,I,J,K);##x11,x12,x13,x21,x22,x23,x31,B4,B5,B6); ##E,F,G,H,I,J,K);
  #[t,y] = sim_Medbot(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6, y0);      ## Test system with no input
  #[t,y] = pole_place_Medbot(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6, y_setpoint, y0); ## Test system with Pole Placement controller
  #[t,y] = lqr_Medbot(x11,x12,x13,x21,x22,x23,x31,B4,B5,B6, y_setpoint, y0);  ## Test system with LQR controller

  endfunction
