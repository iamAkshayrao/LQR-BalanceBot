## Team Id      :	9
## Author List  :	Akshay S Rao, Aliasgar AV		    
## Filename     : read_write.m
## Theme        :	Bipedal Patrol

clear all;
global A = csvread('csv_matter.csv');  #do not change this line

################################################
#######Declare your global variables here#######
################################################
global B     =  zeros(8000,2);
global f_cut = 5;
global ax_lp = 0.0;
global ay_lp = 0.0;         # initialising previous accel's output holder
global az_lp = 0.0;
global gx_hp = 0.0;
global gy_hp = 0.0;
global gz_hp = 0.0;         # initialising previous gyro's output holder
global gx_n_minus_one = 0.0;
global gy_n_minus_one = 0.0; 
global gz_n_minus_one = 0.0;# initialising previous gyro's input value
global pitch  = 0.0;
global roll   = 0.0;

  
function [ax, ay, az] = read_accel(axl,axh,ayl,ayh,azl,azh)  
  global f_cut;
  #################################################
  ####### Write a code here to combine the ########
  #### HIGH and LOW values from ACCELEROMETER #####
  #################################################
    
  scaled_acc_values =[0.0 0.0 0.0]; #holds scaled ax, ay, az values
  
   ########## Combine values of x ,y, z##################
   
  unscaled_ax_value = bin2dec(strcat(dec2bin(axh, 8), dec2bin(axl, 8)));
  unscaled_ay_value = bin2dec(strcat(dec2bin(ayh, 8), dec2bin(ayl, 8)));
  unscaled_az_value = bin2dec(strcat(dec2bin(azh, 8), dec2bin(azl, 8)));
  
  unscaled_values   =[unscaled_ax_value unscaled_ay_value unscaled_az_value];
  
  #scaling values
  for i=1:3     
    if (unscaled_values(i) > 32767)   #check negative value
        scaled_acc_values(i) = (unscaled_values(i) - 65536)/16384.0;
    else
        scaled_acc_values(i) = unscaled_values(i)/16384.0;
    endif
  endfor
  
  ####################################################
  # Call function lowpassfilter(ax,ay,az,f_cut) here #
  ####################################################
  [ax, ay, az] = lowpassfilter(scaled_acc_values(1),scaled_acc_values(2),...
                               scaled_acc_values(3),f_cut);

endfunction

function [gx, gy, gz] = read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  global f_cut;
  #################################################
  ####### Write a code here to combine the ########
  ###### HIGH and LOW values from GYROSCOPE #######
  #################################################
  scaled_gyro_values =[0.0 0.0 0.0]; #holds scaled gx, gy, gz values
  
  ########## Combine values of x ##################
  unscaled_gx_value = bin2dec(strcat(dec2bin(gxh, 8), dec2bin(gxl, 8)));
  unscaled_gy_value = bin2dec(strcat(dec2bin(gyh, 8), dec2bin(gyl, 8)));
  unscaled_gz_value = bin2dec(strcat(dec2bin(gzh, 8), dec2bin(gzl, 8)));
  
  unscaled_values   =[unscaled_gx_value unscaled_gy_value unscaled_gz_value];
  #scaling values
  for i=1:3
     if (unscaled_values(i) > 32767) #check negative value
        scaled_gyro_values(i) = (unscaled_values(i) - 65536)/131.0;
     else
        scaled_gyro_values(i) = unscaled_values(i)/131.0;
     endif
  endfor

  #####################################################
  # Call function highpassfilter(ax,ay,az,f_cut) here #
  #####################################################;
  [gx, gy, gz] = highpassfilter(scaled_gyro_values(1), scaled_gyro_values(2),...
                                scaled_gyro_values(3),f_cut);
                                    
  
endfunction



function [ax_lp, ay_lp, az_lp] = lowpassfilter(ax,ay,az,f_cut)
  dT    = 0.01;                 #time in seconds
  Tau   = 1.0/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);         #do not change this line
  
  global ax_lp;global ay_lp;global az_lp ; 
  
  ################################################
  ##############Write your code here##############
  ################################################
  ax_lp = (1-alpha)*ax + (alpha*ax_lp);
  ay_lp = (1-alpha)*ay + (alpha*ay_lp);
  az_lp = (1-alpha)*az + (alpha*az_lp);

  
  
endfunction



function [gx_hp, gy_hp, gz_hp] = highpassfilter(gx,gy,gz,f_cut)
  dT    = 0.01;                        #time in seconds
  Tau   = 1.0/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  global gx_n_minus_one;global gy_n_minus_one;global gz_n_minus_one;
  global gx_hp; global gy_hp; global gz_hp; 
  ################################################
  ##############Write your code here##############
  ################################################
  gx_hp = (1-alpha)*gx_hp + (1-alpha)*(gx - gx_n_minus_one);
  gy_hp = (1-alpha)*gy_hp + (1-alpha)*(gy - gy_n_minus_one);
  gz_hp = (1-alpha)*gz_hp + (1-alpha)*(gz - gz_n_minus_one);
  
  gx_n_minus_one = gx;
  gy_n_minus_one = gy;
  gz_n_minus_one = gz;
  
  #y[n] = (1-alpha).y[n-1] + (1-alpha)(x[n]-x[n-1])
endfunction

function [pitch] = comp_filter_pitch(ax,ay,az,gx,gy,gz)
  alpha = 0.03;
  dt    = 0.01;
  global pitch ;
  
  ##############################################
  ####### Write a code here to calculate  ######
  ####### PITCH using complementry filter ######
  ##############################################
  
  
  
  pitch      = pitch - (gx*dt);                  #gyro's pitch angle
  acc_angle  = atan2(ay,sqrt(az*az)) *(180.0/pi);#pitch angle from accel...
  pitch      = (1- alpha)* pitch + (alpha)*(acc_angle);
  
  #angle = (1- alpha)(agyro_ngle + gyro * dt) + (alpha)(acc)

endfunction 

function [roll] = comp_filter_roll(ax,ay,az,gx,gy,gz)
  alpha = 0.03;
  dt    = 0.01;
  global roll;
 
  ##############################################
  ####### Write a code here to calculate #######
  ####### ROLL using complementry filter #######
  ##############################################
  roll       = roll - (gy*dt) ;                     # gyro's roll angle
  acc_angle  = atan2((ax),sqrt(az*az))*(180.0/pi);  #calculating accel... roll angle
  roll       = ((1- alpha)* roll + (alpha)*(acc_angle )) ;
  
endfunction 

function execute_code
  global B ;
  global A ;                              #do not change this line

  for n = 1:rows(A)                      #do not change this line
    
    ###############################################
    ####### Write a code here to calculate  #######
    ####### PITCH using complementry filter #######
    ###############################################
    
    [ax ay az]   = read_accel(A(n,2), A(n,1), A(n,4), A(n,3), A(n,6), A(n,5));
    [gx gy gz]   = read_gyro (A(n,8), A(n,7), A(n,10), A(n,9), A(n,12),A(n,11));
    B(n,1)       = comp_filter_pitch(ax, ay, az, gx, gy, gz);
    B(n,2)       = comp_filter_roll (ax, ay, az, gx, gy, gz);
    
  endfor
  csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
