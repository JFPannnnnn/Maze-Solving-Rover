% Copyright (c) 2020, Ian G. Bennett
% All rights reserved.
% Development funded by the University of Toronto, Department of Mechanical
% and Industrial Engineering.
% Distributed under GNU AGPLv3 license.

clear
close all
clc

sim = 1;

if sim
% Initialize tcp server to read and respond to algorithm commands
[s_cmd, s_rply] = tcp_setup();
fopen(s_cmd);
%fopen(s_rply);
else
    %connect to rover Bluetooth
end

% Robot Sensor Measurements
u = [0,0,0,0,0];  % Ultrasonic measurements
u_past = [0,0,0,0,0];
u_pastpast = [0,0,0,0,0];
pos = [0,0,0];  % Position (x,y,rotation)
speed = 1;
rot_stuck = 90;
stepcount = 0;
%pls_go_straight_up(s_cmd,s_rply);

while 1
    
    % Take Measurements
    for ct = 1:5
        cmdstring = [strcat('u',num2str(ct)) newline];
        u(ct) = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
    
    ir = tcpclient_write(['i1' newline], s_cmd, s_rply);
    
    odom = tcpclient_write(['o3' newline], s_cmd, s_rply);
    
    gyro = tcpclient_write(['g1' newline], s_cmd, s_rply);
    
    comp = tcpclient_write(['c1' newline], s_cmd, s_rply);
    
    % Display Values
    disp('Ultrasonic')
    disp(u)
    disp('IR Sensor')
    disp(ir)
    disp('Odometer')
    disp(odom)
    disp('Gyroscope')
    disp(gyro)
    disp('Compass')
    disp(comp)
    
    % Pick random direction, left or right, to try travelling in
    direct_i = randperm(2,2)*2;
    direct = -direct_i*90+270;
    if u(5)>35 && u(5)<60 && u(3)>12 && ((u(1)>12 && u(1) < 48))    
        the_remainder(s_cmd,s_rply);
    elseif u(2) > 40 && u(2) < 60 && u(3)>12 && ((u(1)>12 && u(1)< 24) || (u(1) > 36 && u(1) < 48))          
        the_remainder(s_cmd,s_rply);
    elseif u(2)>7 && u(1)>12 && u(3)>20 && (7<u(5)&&u(5)<26 )
        cmdstring = [strcat('d1-',num2str(3)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
         pls_makesameu4u5(s_cmd,s_rply);
         pls_go_straight_cross(s_cmd,s_rply);
         cmdstring = [strcat('d1-',num2str(3)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
         pls_makesameu4u5(s_cmd,s_rply);
         cmdstring = [strcat('d1-',num2str(2)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        
    elseif u(2)>13 && u(1)>12 && u(3)<=20 && (7<u(5)&&u(5)<26 )  
         cmdstring = [strcat('d1-',num2str(3)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
         pls_makesameu4u5(s_cmd,s_rply);
         pls_go_straight_cross(s_cmd,s_rply);
         cmdstring = [strcat('d1-',num2str(3)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
         pls_makesameu4u5(s_cmd,s_rply);
         cmdstring = [strcat('d1-',num2str(2)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    elseif u(5)>12 && u(1)>12 && u(3)>20 && (7<u(2)&&u(2)<50)  
         cmdstring = [strcat('d1-',num2str(3)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
         pls_makesameu4u5(s_cmd,s_rply);
         pls_go_straight_cross(s_cmd,s_rply);
         cmdstring = [strcat('d1-',num2str(3)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
         pls_makesameu4u5(s_cmd,s_rply);
         cmdstring = [strcat('d1-',num2str(2)) newline];            
         reply = tcpclient_write(cmdstring, s_cmd, s_rply);

    end
    for ct = 1:2
        if  (u(1) > 4) && (u(2) > 1.5) && (u(4) > 1.5)&& (u(5) > 1.5)
            
            % If the way ahead is clear, go forward
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            if stepcount >=2
                adjustmentu24(u,u_past,speed,s_cmd, s_rply);               
            end
            break
            
        elseif u(direct_i(ct)) > 4
            
            % If not, pick a random direction that is clear and go that way
            cmdstring = [strcat('r1-',num2str(direct(ct))) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_go_straight_up(s_cmd,s_rply);
            
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            break
        elseif ct == 2
            if u(2)>=u(5)
            % If no directions clear, turn to the left
                cmdstring = [strcat('r1-',num2str(rot_stuck)) newline];    % Rotate if stuck
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                pls_go_straight_up(s_cmd,s_rply);
            elseif u(5)>u(2)
            % If no directions clear, turn to the left
                cmdstring = [strcat('r1--',num2str(rot_stuck)) newline];    % Rotate if stuck
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                pls_go_straight_up(s_cmd,s_rply);
            end 
            
        end
       
    end
    if abs(u_past(1)-u(1))<0.5 && abs(u_past(3)-u(3))<0.5
        cmdstring = [strcat('d1--',num2str(2*speed)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        pls_go_straight_up(s_cmd,s_rply);
    end
    stepcount = stepcount+1;
    u_past = u;
    
end
    
  
    
    
    