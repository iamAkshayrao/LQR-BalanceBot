1;
pkg load control


##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra 2019-20)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Biped Patrol
##*  Filename: Simple_Pendulum.m
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
##*  This software is made available on an �AS IS WHERE IS BASIS�. 
##*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
##*  any and all claim(s) that emanate from the use of the Software or 
##*  breach of the terms of this agreement.
##*  
##*  e-Yantra - An MHRD project under National Mission on Education using 
##*  ICT(NMEICT)
##*
##**************************************************************************


## Function : draw_pendulum()
## ----------------------------------------------------
## Input:   y - State Vector. In case of pendulum, the state vector consists of 
##          two state variables, theta (angle the pendulum makes with vertical)
##          and theta_dot(angular velocity or rate of change of theta wrt time).
##          L - Length of Pendulum
##
## Purpose: Takes the state vector and length of pendulum as input. It draws the 
##          pendulum in a 2D plot.
function draw_pendulum(y, L)
  theta = y(1);          ## Store the first variable of state vector in theta
  
  x = L*sin(theta);      ## x-coordinate of pendulum bob
  y = 1-L*cos(theta);  ## y coordinate of pendulum bob
  d = 0.1;               ## diameter of pendulum bob
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x-(d/2),y-(d/2),d,d],'Curvature',1,'FaceColor',[1 0 0]);
  line ([0 x], [1 y], "linestyle", "-", "color", "k");
  xlim([-1 1])
  ylim([-0.5 2])
  drawnow
  hold off
endfunction

## Function : pendulum_dynamics()
## ----------------------------------------------------
## Input:   y - State Vector. In case of pendulum, the state vector consists of 
##          two state variables, theta (angle the pendulum makes with vertical)
##          and theta_dot(angular velocity or rate of change of theta wrt time).
##          m - Mass of Pendulum Bob
##          g - Acceleration due to gravity
##          L - Length of Pendulum
##          u - Input to the system
##
## Output:  dy -  Derivative of State Vector. In case of pendulum they will be
##          angular velocity and angular acceleration(theta_dot and theta_dot_dot)
##
## Purpose: Calculates the value of the vector dy according to the equations which 
##          govern this system.
function dy = pendulum_dynamics(y, m, L, g, u)
  sin_theta = sin(y(1));
  cos_theta = cos(y(1));
  
  dy(1,1) = y(2);                                  
  dy(2,1) = (-g/L)*sin_theta + (1.0/(m*L^2))*u;    
endfunction

## Function : sim_pendulum()
## ----------------------------------------------------
## Input:   m - Mass of Pendulum Bob
##          g - Acceleration due to gravity
##          L - Length of Pendulum
##          y0 - Initial condition of system.
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of simple pendulum without 
##          any external input (u)
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0
function [t,y] = sim_pendulum(m, g, L, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = 0;                             ## No Input
  [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, u),tspan,y0); ## Solving the differential equation  
endfunction

## Function : pendulum_AB_matrix()
## ----------------------------------------------------
## Input:   m - Mass of Pendulum Bob
##          g - Acceleration due to gravity
##          L - Length of Pendulum
##
## Output:  A - A matrix of system
##          B - B matrix of system
##          
## Purpose: Declare the A and B matrices in this function.
function [A,B] = pendulum_AB_matrix(m, g, L)
  A = [0 1; g/L 0];
  B = [0 ; 1.0/(m*L^2)];
endfunction

## Function : pole_place_pendulum()
## ----------------------------------------------------
## Input:   m - Mass of Pendulum Bob
##          g - Acceleration due to gravity
##          L - Length of Pendulum
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of simple pendulum with 
##          external input using the pole_placement controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using Pole Placement Technique.
function [t,y] = pole_place_pendulum(m, g, L, y_setpoint, y0)
  [A,B] = pendulum_AB_matrix(m, g, L);                            ## Initialize A and B matrix
  eigs  = [-2 -1];                             ## Initialise desired eigenvalues
  K     = place(A, B, eigs);                           ## Calculate K matrix for desired eigenvalues
  
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : lqr_pendulum()
## ----------------------------------------------------
## Input:   m - Mass of Pendulum Bob
##          g - Acceleration due to gravity
##          L - Length of Pendulum
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of simple pendulum with 
##          external input using the LQR controller.
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using LQR technique.
function [t,y] = lqr_pendulum(m, g, L, y_setpoint, y0)
  [A,B] = pendulum_AB_matrix(m, g, L);               ## Initialize A and B matrix
  Q     = [10 0 ;0 1];                   ## Initialise Q matrix
  R     = 1;                   ## Initialise R 
  
  K     = lqr(A, B, Q, R) ;                   ## Calculate K matrix from A,B,Q,R matrices
  
  tspan = 0:0.1:10;                  ## Initialise time step 
  [t,y] = ode45(@(t,y)pendulum_dynamics(y, m, L, g, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : simple_pendulum_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions. Constant parameters like mass m, gravity g
##          length of pendulum L are defined here.
function simple_pendulum_main()
  m = 1;             
  g = 9.8;
  L = 0.5;
  y_setpoint = [pi; 0];                ## Set Point 
  y0 = [pi/6 ; 0];                   ## Initial condtion
  
  [t,y] = sim_pendulum(m,g,L, y0);        ## Test Simple Pendulum
  #[t,y] = pole_place_pendulum(m,g,L, y_setpoint, y0) ## Test Simple Pendulum with Pole Placement Controller
  #[t,y] = lqr_pendulum(m,g,L, y_setpoint, y0);        ## Test Simple Pendulum with LQR Controller

  for k = 1:length(t)
    draw_pendulum(y(k, :), L);  
  endfor
endfunction