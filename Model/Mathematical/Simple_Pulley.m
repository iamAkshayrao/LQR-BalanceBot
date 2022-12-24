1;
pkg load control


##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra 2019-20)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Biped Patrol
##*  Filename: Simple_Pulley.m
##*  Version: 1.0.0  
##*  Date: November 3, 2019
##*
##*  Team ID :
##*  Team Leader Name:
##*  Team Member Name
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


## Function : draw_pulley()
## ----------------------------------------------------
## Input:   y - State Vector. In case of simple pulley system, the state variables
##              are position x of mass m1 wrt pulley (along vertical) and the 
##              velocity x_dot of mass m1 wrt pulley (along vertical)
##
## Purpose: Takes the state vector as input. It draws the simple pulley system in 
##          a 2D plot.
function draw_pulley(y)
  
  pd = 0.4;                   ## Pulley Diameter
  p_y = 0.5;                 ## Pulley position wrt y
  L = 1;                      ## Length of string
  ml = 0.2;                  ## Mass Length
  mb = 0.1;                  ## Mass Breadth
  x1 = y(1);
  x2 = L-y(1);
  hold on;
  clf;
  axis equal;
  rectangle('Position',[0-(pd/2),p_y-(pd/2),pd,pd],'Curvature',1,'FaceColor',[1 0 0]);
  rectangle('Position',[-(pd/2)-(ml/2),p_y-x1-(mb/2),ml,mb],'Curvature',0.1,'FaceColor',[0 0 1]);
  rectangle('Position',[(pd/2)-(ml/2),p_y-x2-(mb/2),ml,mb],'Curvature',0.1,'FaceColor',[0 1 0]);
  line ([0-(pd/2) 0-(pd/2)], [p_y p_y-x1], "linestyle", "-", "color", "k");
  line ([(pd/2) (pd/2)], [p_y p_y-x2], "linestyle", "-", "color", "k");
  line ([0 0], [1 p_y], "linestyle", "-", "color", "k");
  xlim([-1 1])
  ylim([-1 1])
  drawnow
  hold off
  
endfunction

## Function : pulley_dynamics()
## ----------------------------------------------------
## Input:   y - State Vector. In case of simple pulley system, the state variables
##              are position x of mass m1 wrt pulley (along vertical) and the 
##              velocity x_dot of mass m1 wrt pulley (along vertical)
##          m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          g  - Acceleration due to gravity
##          r  - radius of pulley 
##          u - Input to the system
##
## Output:  dy -  Derivative of State Vector. In case of simple pulley system they 
##                will be the velocity x_dot and acceleration x_dot_dot
## Purpose: Calculates the value of the vector dy according to the equations which 
##          govern this system.
function dy = pulley_dynamics(y, m1, m2, g, r, u)
  
  dy(1,1) = y(2,1);
  dy(2,1) = ((m1-m2)*g+u/r)/(m1+m2);;  
endfunction

## Function : sim_pulley()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          g  - Acceleration due to gravity
##          r  - radius of pulley
##          y0 - Initial Condition
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of simple pulley without 
##          any external input (u)
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0
function [t,y] = sim_pulley(m1, m2, g, r, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = 0;                             ## No Input
  [t,y] =  ode45(@(t,y)pulley_dynamics(y, m1, m2, g, r, u),tspan,y0);;  
endfunction

## Function : pulley_AB_matrix()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          g  - Acceleration due to gravity
##          r  - radius of pulley
##
## Output:  A - A matrix of system
##          B - B matrix of system
##          
## Purpose: Declare the A and B matrices in this function.
function [A,B] = pulley_AB_matrix(m1, m2, g, r)
  A = [0 1;0 0];
  B = [0 ; 1/r*(m1+m2)];
endfunction

## Function : pole_place_pulley()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          g  - Acceleration due to gravity
##          r  - radius of pulley
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of simple pulley with 
##          external input using the pole_placement controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using Pole Placement Technique.
function [t,y] = pole_place_pulley(m1, m2, g, r, y_setpoint, y0)
  [A,B]=pulley_AB_matrix(m1, m2, g, r);
  eigs = [-7499;-4.4];
  K=place(A,B,eigs);
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)pulley_dynamics(y, m1, m2, g, r,-K*(y-y_setpoint)),tspan,y0); ;  
endfunction

## Function : lqr_pulley()
## ----------------------------------------------------
## Input:   m1 - Mass of 1st block
##          m2 - Mass of 2nd block
##          g  - Acceleration due to gravity
##          r  - radius of pulley
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of simple pulley with 
##          external input using the LQR controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using LQR
function [t,y] = lqr_pulley(m1, m2, g, r, y_setpoint, y0)
  [A,B]=pulley_AB_matrix(m1, m2, g, r);
  Q=[200000 0;0 10000];
  R=1;
  [K,lambda,l]= lqr(A,B,Q,R);
  disp(K);
  disp(l);
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)pulley_dynamics(y, m1, m2, g, r,-K*(y-y_setpoint)),tspan,y0) ;  
endfunction

## Function : simple_pulley_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions. Constant parameters like masses m1 and m2, 
##          gravity g and radius of pulley are defined here.
function simple_pulley_main()
  m1 = 6;
  m2 = 9;
  g = 9.8;
  r = 0.2;
  y0 = [0.4 ; 0];                   ## Initial condtion
  y_setpoint = [0.8; 0];              ## Set Point
  
  #[t,y] = sim_pulley(m1, m2, g, r, y0);
  [t,y] = pole_place_pulley(m1, m2, g, r, y_setpoint, y0);
 #[t,y] = lqr_pulley(m1, m2, g, r, y_setpoint, y0);
  disp(y);
  for k = 1:length(t)
    draw_pulley(y(k, :));  
  endfor
endfunction
