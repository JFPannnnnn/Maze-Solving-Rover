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
u = [0,0,0,0,0,0];  % Ultrasonic measurements
u_past = [0,0,0,0,0,0];
pos = [0,0,0];  % Position (x,y,rotation)
speed = 1;
rot_stuck = 90;
stepcount = 0;
% pls_go_straight_up(s_cmd,s_rply);
pls_makesameu4u5(s_cmd,s_rply);

while 1
    
    % Take Measurements
    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    %     ir = tcpclient_write(['i1' newline], s_cmd, s_rply);
    %
    %     odom = tcpclient_write(['o3' newline], s_cmd, s_rply);
    %
    %     gyro = tcpclient_write(['g1' newline], s_cmd, s_rply);
    %
    %     comp = tcpclient_write(['c1' newline], s_cmd, s_rply);
    
    % Display Values
    disp('Ultrasonic')
    disp(u)
    %     disp('IR Sensor')
    %     disp(ir)
    %     disp('Odometer')
    %     disp(odom)
    %     disp('Gyroscope')
    %     disp(gyro)
    %     disp('Compass')
    %     disp(comp)
    
    % Pick random direction, left or right, to try travelling in
    if length(u) == 6
        direct_i = randperm(2,2)*2;
        direct = -direct_i*90+270;

        if u(5)>35 && u(5)<60 && u(3)>12 && ((u(1)>20 && u(1) < 55))
            the_remainder(s_cmd,s_rply);
        elseif u(2) > 40 && u(2) < 60 && u(3)>20 && ((u(1)>12 && u(1)< 24) || (u(1) > 36 && u(1) < 48))
            the_remainder(s_cmd,s_rply);
        elseif u(2)>7 && u(1)>12 && u(3)>20 && (7<u(5)&&u(5)<26 )
            cmdstring = [strcat('d1--',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_makesameu4u5(s_cmd,s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);




        elseif u(2)>13 && u(1)>12 && u(3)<=20 && (7<u(5)&&u(5)<30)
            cmdstring = [strcat('d1--',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_makesameu4u5(s_cmd,s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);


        elseif u(5)>12 && u(1)>12 && u(3)>20 && (7<u(2)&&u(2)<50)
            cmdstring = [strcat('d1--',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_makesameu4u5(s_cmd,s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);



        end
        if abs(u_past(1)-u(1))<0.5 && abs(u_past(3)-u(3))<0.5
            if u(3) > 3
                cmdstring = [strcat('d1--',num2str(2*speed)) newline];             % Build command string to move bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                pls_go_straight_up(s_cmd,s_rply);
            else
                pls_go_straight_up(s_cmd,s_rply);
            end
        end

        for ct = 1:2
            if  (u(1) > 2.8) && (u(2) > 1.3) && (u(4) > 1.3)&& (u(5) > 1.3)

                % If the way ahead is clear, go forward
                if stepcount >=2
                    adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                end
                cmdstring = [strcat('d1-',num2str(speed*1.5)) newline];             % Build command string to move bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                break

            elseif u(direct_i(ct)) > 6

                % If not, pick a random direction that is clear and go that way
                cmdstring = [strcat('r1-',num2str(direct(ct))) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                %             pls_go_straight_up(s_cmd,s_rply);
                break
            elseif ct == 2
                if u(2)>=u(5)
                    % If no directions clear, turn to the left
                    cmdstring = [strcat('r1-',num2str(rot_stuck)) newline];    % Rotate if stuck
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    %                 pls_go_straight_up(s_cmd,s_rply);
                elseif u(5)>u(2)
                    % If no directions clear, turn to the left
                    cmdstring = [strcat('r1--',num2str(rot_stuck)) newline];    % Rotate if stuck
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    %                 pls_go_straight_up(s_cmd,s_rply);
                end

            end

        end
    %     if abs(u_past(1)-u(1))<0.5 && abs(u_past(3)-u(3))<0.5
    %         if u(3) > 3
    %             cmdstring = [strcat('d1--',num2str(2*speed)) newline];             % Build command string to move bot
    %             reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    %             pls_go_straight_up(s_cmd,s_rply);
    %         else
    %             pls_go_straight_up(s_cmd,s_rply);
    %         end
    %     end
        %     if u(1)<2.5 && u(2)<4  && u(4)<4 && u(5)<4
        %         cmdstring = [strcat('r1-',num2str(180)) newline];    % Rotate if stuck
        %         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        %     end
        stepcount = stepcount+1;
        u_past = u;
    else 
        
    end
    
    
end

%%
function[] = adjustmentu24(u,u_past,speed,s_cmd, s_rply)

u_matrix_twotimes = read_two_times(s_cmd,s_rply);

u2 = average(u_matrix_twotimes(:,2));
u4 = average(u_matrix_twotimes(:,4));
u5 = average(u_matrix_twotimes(:,5));

if u(2) > 12 && u(5) > 12
else
    if abs(u(2)-u_past(2))/u_past(2) < 5 || abs(u(5)-u_past(5))/u_past(5) < 5
        if (u2<u4 && u2>3.5) || (u2>u4 && (u4<2)||u5<2)
            cmdstring = [strcat('r1-',num2str(20)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(-13)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        elseif (u2>u4 && u4>3.5) || (u2<u4 && u2<2)
            cmdstring = [strcat('r1-',num2str(-20)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(13)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        end
        toler = 0.2;
        u2diff = straight (u(4),u_past(2),toler);
        u4diff = straight (u(4),u_past(4),toler);
        if (u2diff==1) || (u4diff==1)
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            if (u(2)-u_past(2))<0
                cmdstring = [strcat('r1-',num2str(-5)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                
            else
                cmdstring = [strcat('r1-',num2str(5)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            end
        end
        
        
    end
end


end



%%
function ave = average(x)
ave = sum(x(:))/numel(x);
end



%%
function[] = pls_go_straight_up(s_cmd,s_rply)
u = 0;  % Ultrasonic measurements
u_past = u;
count =0;
error = 1;
incre = 10;
tol = 0.03;
num = 1;
angle = 0;
u_matrix_fivetimes = 0;

u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
ua = average(u_matrix_fivetimes(:,1));
ub = average(u_matrix_fivetimes(:,2));
uc = average(u_matrix_fivetimes(:,3));
ud = average(u_matrix_fivetimes(:,4));
ue = average(u_matrix_fivetimes(:,5));

all = [ua ub uc ud ue];
[val,num] = min(all);

while all(num) <= 1.7
    all(num) = 10000;
    [val,num] = min(all);
end

while error
    if (count == 0)
%         u1 = read_five_times(s_cmd,s_rply,num)
        u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
        u1 = average(u_matrix_fivetimes(:,num));
        cmdstring = [strcat('r1-',num2str(5)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply)
%         u2  = read_five_times(s_cmd,s_rply,num)
        u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
        u2 = average(u_matrix_fivetimes(:,num));
        u = u2;
        if straight(u1,u2,tol)
            error = 0;
        else
            if u2>u1
                incre = -incre;
                inv = 1;
            end
            
        end
        cmdstring = [strcat('r1-',num2str(-5)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        
    else
        cmdstring = ['ua' newline];
        u = tcpclient_write(cmdstring, s_cmd, s_rply);
        u = u(num);
        %             u = read_five_times(s_cmd,s_rply,num)
        if ~straight(u(1),u_past(1),tol)
            %                 if u<=u_past
            %                     if inv
            %                     incre = incre + 0.2;
            %                     else
            %                         incre = incre - 0.2;
            %                     end
            %                 end
            if (angle<=720 &&angle>0) || (angle<=0 && (angle>=-720))
                cmdstring = [strcat('r1-',num2str(incre)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                angle = angle+incre;
            else
%                 um = read_five_times(s_cmd,s_rply,1);
%                 un = read_five_times(s_cmd,s_rply,3);
                u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
                um = average(u_matrix_fivetimes(:,1));
                un = average(u_matrix_fivetimes(:,3));
                if um >= un
                    cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                else
                    cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                end
                angle = 0;
            end
        else
%             u1 = read_five_times(s_cmd,s_rply,num)
%             cmdstring = [strcat('r1-',num2str(-1)) newline];
%             reply = tcpclient_write(cmdstring, s_cmd, s_rply)
%             u2  = read_five_times(s_cmd,s_rply,num)
            u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
            u1 = average(u_matrix_fivetimes(:,num));
            cmdstring = [strcat('r1-',num2str(-1)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply)
            u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
            u2 = average(u_matrix_fivetimes(:,num));
            
            if straight(u1,u2,tol)
                error = 0;
            end
            cmdstring = [strcat('r1-',num2str(2)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            
        end
        
    end
    u_past = u
    count = count+1
    
end
end


%%
function[] = pls_makesameu4u5(s_cmd,s_rply)
u = [0,0,0,0,0,0];  % Ultrasonic measurements
rot_stuck = 90;
stepcount = 0;

a = 1;

for i = 1:4
    
%     for m = 4:5
%         cmdstring = [strcat('u',num2str(m)) newline];
%         u(m) = tcpclient_write(cmdstring, s_cmd, s_rply);
%     end

    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    if (abs(u(4) - u(5)) <= 0.15)
        u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
        u4 = average(u_matrix_fivetimes(:,4));
        u5 = average(u_matrix_fivetimes(:,5));
        if abs(u4 - u5) <= 0.15
            a = 0;
            break;
        end
        
    elseif (abs(u(4) - u(5)) > 0.15) && (abs(u(4) - u(5)) <= 7)
        %abs(u(4) - u(5))>1
        % If there is a large discrepancy between ultra 4 and 5, then
        % adjust until they produce similiar readings
        % adjust its direction
        if u(4)>u(5)
            beta = 5; % The spacing between u4 and u5 is ~2 inches
            cmdstring = [strcat('r1-',num2str(beta)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        else
            beta = -5; % The spacing between u4 and u5 is ~2 inches
            cmdstring = [strcat('r1-',num2str(beta)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            
        end
    else
        a = 0;
    end
end
end




%%
function[u_matrix_twotimes] = read_two_times(s_cmd,s_rply)
u_two = [0 0 0 0 0 0];
u_matrix_twotimes = zeros(2,6);
for i = 1:2
    cmdstring = ['ua' newline];
    u_two = tcpclient_write(cmdstring, s_cmd, s_rply);
    if length(u_two) == 6
        for j = 1:6
            u_matrix_twotimes(i,j) = u_two(j);
        end
    else 
        break
    end
end
end 
%%
function[u_matrix_fivetimes] = read_five_times(s_cmd,s_rply)
u_five = [0 0 0 0 0 0];
u_matrix_fivetimes = zeros(5,6);
for i = 1:5
    cmdstring = ['ua' newline];
    u_five = tcpclient_write(cmdstring, s_cmd, s_rply);
    if length(u_five) == 6
        for j = 1:6
            u_matrix_fivetimes(i,j) = u_five(j);
        end
    else
        break
    end

end
end 
%%
function[] = the_remainder(s_cmd,s_rply)
u = [0 0 0 0 0 0];  % Ultrasonic measurements
rot_stuck = 90;
stepcount = 0;

cmdstring = ['ua' newline];
u = tcpclient_write(cmdstring, s_cmd, s_rply);
r = rem((u(1)-1),12);
a = r/15;
u_first = u
for i = 1:10
    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    r = rem((u(1)-1),12);
    if r > 2.8
        r = rem((u(1)-1),12);
        cmdstring = [strcat('d1-',num2str(a)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    else
        break
    end
    
end

array = [u(2),u(4)]
[val,idx] = max(array)

if idx == 1
    % If there is space on the left
    cmdstring = [strcat('r1-',num2str(90)) newline];  % Build command string to rotate bot
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    for i = 1:4
        cmdstring = [strcat('d1-',num2str(0.8)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        if abs(u(2)-u_first(2)) < 5
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            if i>= 2
                toler = 0.2;
                u2diff = straight (u(2),u_first(2),toler);
                u4diff = straight (u(4),u_first(4),toler);
                if (u2diff==1) || (u4diff==1)
                    cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                else
                    if (u(2)-u_first(2))<0
                        cmdstring = [strcat('r1-',num2str(-5)) newline];  % Build command string to rotate bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        
                    else
                        cmdstring = [strcat('r1-',num2str(5)) newline];  % Build command string to rotate bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        
                    end
                end
            end
        end
        u_first = u;
    end
    
elseif idx == 2
    % If there is space on the right
    cmdstring = [strcat('r1-',num2str(-90)) newline];  % Build command string to rotate bot
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    for i = 1:4
        cmdstring = [strcat('d1-',num2str(0.8)) newline];             % Build command string to move bot
        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
        if abs(u(2)-u_first(2)) < 5
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            if i >=2
                toler = 0.2;
                u2diff = straight (u(2),u_first(2),toler);
                u4diff = straight (u(4),u_first(4),toler);
                if (u2diff==1) || (u4diff==1)
                    cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                else
                    if (u(2)-u_first(2))<0
                        cmdstring = [strcat('r1-',num2str(-5)) newline];  % Build command string to rotate bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        
                    else
                        cmdstring = [strcat('r1-',num2str(5)) newline];  % Build command string to rotate bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        cmdstring = [strcat('d1-',num2str(1)) newline];             % Build command string to move bot
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                        
                    end
                end
            end
        end
        u_first = u;
    end
    
end
end

%%
function[a] = straight(u,u_past,tol)

error = abs(u - u_past)/u_past;

    if (average(error) <= tol)
        a = 1;

    else
        a =0;
    end
end






