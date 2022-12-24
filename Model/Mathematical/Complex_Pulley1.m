1;
pkg load control


##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra 2019-20)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Biped Patrol
##*  Filename: Complex_Pulley.m
##*  Version: 1.0.0  
##*  Date: November 3, 2019
##*
##*  Team ID :
##*  Team Leader Name:
##*  Team Member Name
##*
##*  
##*  Author: e-Yantra Project, Department of Computer Science
##*  and Engineering, Indian Institute of Technology Bombay.
##*  
##*  Software released under Creative Commons CC BY-NC-SA
##*
##*  For legal information refer to:
##*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
##*     
##*
##*  This software is made available on an “AS IS WHERE IS BASIS”. 
##*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
##*  any and all claim(s) that emanate from the use of the Software or 
##*  breach of the terms of this agreement.
##*  
##*  e-Yantra - An MHRD project under National Mission on Education using 
##*  ICT(NMEICT)
##*
##**************************************************************************


## Function : draw_complex_pulley()
## ----------------------------------------------------
## Input:   y - State Vector. In case of complex pulley system, the state variables
##              are position x of mass m1 wrt pulley A (along vertical), position y
##              of mass m2 wrt pulley B (along vertical), velocity x_dot (of mass m1)
##              wrt pulley A and velocity y_dot (of mass m2) wrt pulley B 
##
## Purpose: Takes the state vector as input. It draws the complex pulley system in 
##          a 2D plot.
function draw_complex_pulley(y)
  ml = 0.2;
  mb = 0.1;
  L_A = 1.3;
  L_B = 1.1;
  
  pd_A = 0.4;
  py_A = 1;
  pd_B = 0.3; 
  
  x1 = y(1);
  x2 = L_A - y(1);
  y1 = y(3);
  y2 = L_B - y(3);
  
  pulley_A_pos = {0, py_A};
  pulley_B_pos = {(-pd_A/2), py_A-x2};
  m1_pos = {(pd_A/2), py_A-x1};
  m2_pos = {((-pd_A+pd_B)/2), py_A-x2-y1};
  m3_pos = {((-pd_A-pd_B)/2), py_A-x2-y2};
  x1_string = {(pd_A/2), py_A, (pd_A/2), (py_A-x1)};
  x2_string = {(-pd_A/2), py_A, (-pd_A/2), (py_A-x2)};
  y1_string = {((-pd_A+pd_B)/2), py_A-x2, ((-pd_A+pd_B)/2), py_A-x2-y1};
  y2_string = {((-pd_A-pd_B)/2), py_A-x2, ((-pd_A-pd_B)/2), py_A-x2-y2};
  
  hold on;
  clf;
  axis equal;
  rectangle('Position',[pulley_A_pos{1}-(pd_A/2),pulley_A_pos{2}-(pd_A/2),pd_A, pd_A],'Curvature',1,'FaceColor',[1 0 0]);## Pulley A
  rectangle('Position',[pulley_B_pos{1}-(pd_B/2),pulley_B_pos{2}-(pd_B/2),pd_B, pd_B],'Curvature',1,'FaceColor',[1 0 0]);## Pulley B
  rectangle('Position',[m1_pos{1}-(ml/2),m1_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 0 1]);## m1 mass
  rectangle('Position',[m2_pos{1}-(ml/2),m2_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 1 0]);## m2 mass
  rectangle('Position',[m3_pos{1}-(ml/2),m3_pos{2}-(mb/2),ml, mb],'Curvature',0.1,'FaceColor',[0 1 1]);## m1 mass
  line ([x1_string{1} x1_string{3}], [x1_string{2} x1_string{4}], "linestyle", "-", "color", "k");
  line ([x2_string{1} x2_string{3}], [x2_string{2} x2_string{4}], "linestyle", "-", "color", "k");
  line ([y2_string{1} y2_string{3}], [y2_string{2} y2_string{4}], "linestyle", "-", "color", "k");
  line ([y1_string{1} y1_string{3}], [y1_string{2} y1_string{4}], "linestyle", "-", "color", "k");
  xlim([-1.5 1.5]);
  ylim([-1.5 1.5]);
  drawnow
  hold off
  
endfunction

## Function : complex_pulley_dynamics()
## ----------------------------------------------------
## Input:   y - State Vector. In case of complex pulley system, the state variables
##              are position x of mass m1 wrt pulley A (along vertical), position y
##              of mass m2 wrt pulley B (along vertical), velocity x_dot (of mass m1)
##              wrt pulley A and velocity y_dot (of mass m2) wrt pulley B 
##          m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          m3 - Mass of 3rd block
##          g  - Acceleration due to gravity
##          rA - radius of pulley A
##          rB - radius of pulley B
##          u  - Input to the system. In this case there are two inputs acting on the 
##               system. Torque acting on pulley A is u(1) and torque acting on pulley
##               B is u(2).
##
## Output:  dy -  Derivative of State Vector.
##
## Purpose: Calculates the value of the vector dy according to the equations which 
##          govern this system.
function dy = complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB, u)
  
  dy(1,1) = y(2);
  dy(2,1) = (1/(m1*m2+m1*m3+4*m2*m3))*(((m1*m2+m1*m3-4*m2*m3)*g)+((u(1)/rA)*(m2+m3))-((u(2)/rB)*(m3-m2)));
  dy(3,1) = y(4);
  dy(4,1) = (1/(m1*m2+m1*m3+4*m2*m3))*(((2*m1*m2-2*m1*m3)*g)+((u(1)/rA)*(m2-m3))+((u(2)/rB)*(m1+m2+m3)));
endfunction

## Function : sim_complex_pulley()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          m3 - Mass of 3rd block
##          g  - Acceleration due to gravity
##          rA - radius of pulley A
##          rB - radius of pulley B
##          y0 - Initial Condition of system
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of complex pulley without 
##          any external input (u)
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0
function [t,y] = sim_complex_pulley(m1, m2, m3, g, rA, rB, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = [0; 0];                             ## No Input
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB,u),tspan,y0); 
  endfunction

## Function : complex_pulley_AB_matrix()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          m3 - Mass of 3rd block
##          g  - Acceleration due to gravity
##          rA - radius of pulley A
##          rB - radius of pulley B
##
## Output:  A - A matrix of system
##          B - B matrix of system
##          
## Purpose: Declare the A and B matrices in this function.
function [A,B] = complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB)
  A = [0 1 0 0;
       0 0 0 0;
       0 0 0 1;
       0 0 0 0];
  B =(1/(m1*m2 + m1*m3 + 4*m2*m3)) *([0, 0; (m2+m3)/rA, (m2-m3)/rB; 0, 0; (m2-m3)/rA, (m1+m2+m3)/rB]);
endfunction

## Function : pole_place_complex_pulley()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          m3 - Mass of 3rd block
##          g  - Acceleration due to gravity
##          rA - radius of pulley A
##          rB - radius of pulley B
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of complex pulley with 
##          external input using the pole_placement controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using Pole Placement Technique.
function [t,y] = pole_place_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
    [A,B]=complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB);
     eigs = [-1.5 + 1.50i;
             -1.5 - 1.5i;
             -2 + 1.5i;
             -2 - 1.5i];
     K=place(A,B,eigs);
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB,-K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : lqr_complex_pulley()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          m3 - Mass of 3rd block
##          g  - Acceleration due to gravity
##          rA - radius of pulley A
##          rB - radius of pulley B
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of complex pulley with 
##          external input using the LQR controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using LQR Controller.
function [t,y] = lqr_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
   [A,B]=complex_pulley_AB_matrix(m1, m2, m3, g, rA, rB);
    Q=[1500 0 0 0;
       0 10 0 0;
       0 0 1000 0;
       0 0 0 100];
       
    R=[1 0;0 1];
    [K,m,n]= lqr(A,B,Q,R);
   disp(n); 
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)complex_pulley_dynamics(y, m1, m2, m3, g, rA, rB,-K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : complex_pulley_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions and observing the behavior of the system. Constant
##          parameters like mass of block, radius of pulley etc are declared here.
function complex_pulley_main()
  m1 = 23.95;
  m2 = 12;
  m3 = 12.03;
  g = 9.8;
  rA = 0.2;
  rB = 0.2;
  y_setpoint = [0.5 ; 0; 0.8; 0];
  y0 = [0.9 ; 0; 0.2; 0];
  
  #[t,y] = sim_complex_pulley(m1, m2, m3, g, rA, rB, y0)
  [t,y] = pole_place_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
  #[t,y] = lqr_complex_pulley(m1, m2, m3, g, rA, rB, y_setpoint, y0)
  
  for k = 1:length(t)
    draw_complex_pulley(y(k, :));
  endfor
  
endfunction
