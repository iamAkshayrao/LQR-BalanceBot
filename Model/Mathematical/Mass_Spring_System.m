1;
pkg load control


##**************************************************************************
##*                OCTAVE PROGRAMMING (e-Yantra 2019-20)
##*                ====================================
##*  This software is intended to teach Octave Programming and Mathematical
##*  Modeling concepts
##*  Theme: Biped Patrol
##*  Filename: Mass_Spring_System.m
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


## Function : draw_mass_spring()
## ----------------------------------------------------
## Input:   y - State Vector. In case of mass spring system, the state variables
##              are the position x and the velocity x_dot
##
## Purpose: Takes the state vector as input. It draws the mass spring system in 
##          a 2D plot.
function draw_mass_spring(y)    
  l = 0.3; ## Length of rectangle
  b = 0.2; ## Breadth of rectangle
  
  x_pos = y(1);
  y_pos = 0;
  
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x_pos-l/2,y_pos,l,b],'Curvature',0,'FaceColor',[0 0 1]);
  line ([0 0], [0 0.6], "linestyle", "-", "color", "k");
  line ([-0.5 x_pos], [(y_pos+b)/2 (y_pos+b)/2], "linestyle", "--", "color", "k");
  text(-0.05, 0.65, "Eqbm Pt")
  xlim([-0.5 1])
  ylim([0 1])
  drawnow
  hold off;
endfunction

## Function : mass_spring_dynamics()
## ----------------------------------------------------
## Input:   y - State Vector. In case of mass spring system, the state variables
##              are the position x and the velocity x_dot
##          m - Mass of Block
##          k - Spring Constant
##          u - Input to the system
##
## Output:  dy -  Derivative of State Vector. In case of mass spring system they 
##                will be the velocity x_dot and acceleration x_dot_dot
## Purpose: Calculates the value of the vector dy according to the equations which 
##          govern this system.
function dy = mass_spring_dynamics(y, m, k, u)
  
  dy(1,1) = y(2);
  dy(2,1) = (u/m) - (k/m)*y(1);
endfunction

## Function : sim_mass_spring()
## ----------------------------------------------------
## Input:   m - Mass of Block
##          k - Spring Constant
##          y0 - Initial condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of mass spring system without 
##          any external input (u)
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0
function [t,y] = sim_mass_spring(m, k, y0)
  tspan = 0:0.1:10;                ## Initialize time step
  u = 0;                           ## No input
  [t,y] = ode45(@(t,y)mass_spring_dynamics(y, m, k, u),tspan,y0);;
endfunction

## Function : mass_spring_AB_matrix()
## ----------------------------------------------------
## Input:   m - Mass of Block
##          k - Spring Constant
##
## Output:  A - A matrix of system
##          B - B matrix of system
##          
## Purpose: Declare the A and B matrices in this function.
function [A,B] = mass_spring_AB_matrix(m, k)
  A = [0 1;(-k/m) 0];
  B = [0; (1.0/m)];
endfunction

## Function : pole_place_mass_spring()
## ----------------------------------------------------
## Input:   m - Mass of Block
##          k - Spring Constant
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of mass spring system with 
##          external input using the pole_placement controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using Pole Placement Technique.
function [t,y] = pole_place_mass_spring(m, k, y_setpoint, y0)
  [A,B] = mass_spring_AB_matrix(m, k);  ## Initialize A and B matrix 
  eigs  = [-20 -1];                    ## Initialise desired eigenvalues
  K     = place(A, B, eigs);                ## Calculate K matrix for desired eigenvalues
  
  tspan = 0:0.1:10;                   ## Initialise time step 
  [t,y] = ode45(@(t,y)mass_spring_dynamics(y, m, k, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : lqr_mass_spring()
## ----------------------------------------------------
## Input:   m - Mass of Block
##          k - Spring Constant
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of mass spring system with 
##          external input using the LQR controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using LQR
function [t,y] = lqr_mass_spring(m, k, y_setpoint, y0)
  [A,B] = mass_spring_AB_matrix(m, k); ;  ## Initialize A and B matrix 
  Q = [10000 0;0 100] ;                  ## Initialise desired eigenvalues
  R = 1;               ## Calculate K matrix for desired eigenvalues
  
  K = lqr(A,B,Q,R);
  
  tspan = 0:0.1:10;                   ## Initialise time step 
  [t,y] = ode45(@(t,y)mass_spring_dynamics(y, m, k, -K*(y-y_setpoint)),tspan,y0) ;
endfunction

## Function : mass_spring_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions. Constant parameters like mass m, spring
##          constant k are defined here.
function mass_spring_main()
  m = 0.2;
  k = 0.8;
  y0 = [0.7; 0];
  y_setpoint = [-0.2; 0];
  
  #[t,y] = sim_mass_spring(m,k, y0);      ## Test mass spring system with no input
 #[t,y] = pole_place_mass_spring(m, k, y_setpoint, y0); ## Test system with Pole Placement controller
  [t,y] = lqr_mass_spring(m, k, y_setpoint, y0);  ## Test system with LQR controller
  disp(y);
  for k = 1:length(t)
    draw_mass_spring(y(k, :));  
  endfor
endfunction
