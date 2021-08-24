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
speed = 1.5;
rot_stuck = 90;
stepcount = 0;
turn = 0;
movement = 0;
location= [0,0];
anglesu = 0;
pls_makesameu4u5(s_cmd,s_rply);

%% localization, initialize the map
%initalization of the world
dim1 = 32; dim2 = 16; 
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; 
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

%make blocks
M = zeros(size(bw));
Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 1;
end
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];

%generate ultrasonic world
ultra = zeros(size(bw));
for sec_row = 1:4:dim2,
    for sec_col = 1:4:dim1,
        segRow = M(sec_row+2, sec_col:sec_col+5);
        segCol = M(sec_row:sec_row+5, sec_col+2);
        val = sum(segRow)+sum(segCol);
        if val == 2 && sum(segRow)~=1,
            val = 5;
        end
        ultra(sec_row:sec_row+3, sec_col:sec_col+3) = val;
    end
end

%create mask for blocks
M = abs(M-1);
M = M(2:end-1, 2:end-1);
figure; imagesc((bw+1).*M); colormap(gray);

%initialize probability
p = ones(dim2,dim1)*(1/n); 

% figure;
%% 

heading = tcpclient_write(['c1' newline], s_cmd, s_rply); % ['c1' newline] is the cmdstring
disp(heading);
if heading >= -45 && heading  <= 45 || heading > 315 && heading <= 360
    heading = 0;
elseif heading > 45 && heading  <= 135
    heading = 90;
elseif heading > 135 && heading  <= 225
    heading = 180;
elseif heading > 225 && heading  <= 315
    heading = 270;   
end

% special blocks
% one side blocked
M_one_side = 0.1.*ones(size(bw));
Blocks = [2, 1; 2, 3; 3, 4; 8, 2;];
for xx = 1:size(Blocks,1)
	x = Blocks(xx,1); y = Blocks(xx,2);
	M_one_side(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 0.6;
end

% three side blocked
M_tree = 0.1.*ones(size(bw));
Blocks = [3,3; 6, 1; 8, 1; 8, 4;];
for xx = 1:size(Blocks,1)
	x = Blocks(xx,1); y = Blocks(xx,2);
	M_tree(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 0.6;
end

% two sides side blocked
M_two_side = 0.1.*ones(size(bw));
Blocks = [1,3; 2, 4; 3, 1; 4, 4;5, 4;5, 2;6, 3;7, 2;8, 3;];
for xx = 1:size(Blocks,1)
	x = Blocks(xx,1); y = Blocks(xx,2);
	M_two_side(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 0.6;
end

% two sides edges blocked
M_two_edge = 0.1.*ones(size(bw));
Blocks = [1,1; 2, 2; 1, 4; 4, 1;4, 2;6, 4;];
for xx = 1:size(Blocks,1)
	x = Blocks(xx,1); y = Blocks(xx,2);
	M_two_edge(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 0.6;
end


% four sides open
M_four_open = 0.1.*ones(size(bw));
Blocks = [6,2;];
for xx = 1:size(Blocks,1),
	x = Blocks(xx,1); y = Blocks(xx,2);
	M_four_open(1+(y-1)*4:(y-1)*4+4, 1+(x-1)*4:(x-1)*4+4) = 0.6;
end
%% main function 
while 1
    
    % Take Measurements
    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    Vec_sensors = [u(1) u(2) u(3) u(5)];
    
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
            [p,heading] = the_remainder(s_cmd,s_rply,ultra, M, p, heading);
        elseif u(2) > 40 && u(2) < 60 && u(3)>20 && ((u(1)>12 && u(1)< 24) || (u(1) > 36 && u(1) < 48))
            [p,heading] = the_remainder(s_cmd,s_rply,ultra, M, p, heading);
        elseif u(2)>7 && u(1)>12 && u(3)>20 && (7<u(5)&&u(5)<26 )
            cmdstring = [strcat('d1--',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_makesameu4u5(s_cmd,s_rply);
            
            for i = 1:6
                cmdstring = [strcat('d1-',num2str(3)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                cmdstring = ['ua' newline];
                u = tcpclient_write(cmdstring, s_cmd, s_rply);
                m_u = determine_mu(u);
                m_m = 'w';
                p = sense_uold(ultra, M, p, m_u);
                imagesc(p);
                [p, heading] = move(p, M, heading, m_m);   
            end 



        elseif u(2)>13 && u(1)>12 && u(3)<=20 && (7<u(5)&&u(5)<30)
            cmdstring = [strcat('d1--',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_makesameu4u5(s_cmd,s_rply);
            
            for i = 1:6
                cmdstring = [strcat('d1-',num2str(3)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                cmdstring = ['ua' newline];
                u = tcpclient_write(cmdstring, s_cmd, s_rply);
                m_u = determine_mu(u);
                m_m = 'w';
                p = sense_uold(ultra, M, p, m_u);
                imagesc(p)
                [p, heading] = move(p, M, heading, m_m); 
            end


       elseif u(5)>12 && u(1)>12 && u(3)>20 && (7<u(2)&&u(2)<50)
            cmdstring = [strcat('d1--',num2str(6)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            pls_makesameu4u5(s_cmd,s_rply);
            
            for i = 1:6
                cmdstring = [strcat('d1-',num2str(3)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                cmdstring = ['ua' newline];
                u = tcpclient_write(cmdstring, s_cmd, s_rply);
                m_u = determine_mu(u);
                m_m = 'w';
                p = sense_uold(ultra, M, p, m_u);
                imagesc(p)
                [p, heading] = move(p, M, heading, m_m); 
            end


        end
        if abs(u_past(1)-u(1))<0.5 && abs(u_past(3)-u(3))<0.5 % if stuck
            if u(3) > 3
                cmdstring = [strcat('d1--',num2str(2*speed)) newline];             % Build command string to move bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                [turn,anglesu] = pls_go_straight_up(s_cmd,s_rply);
                heading = heading + anglesu;
                anglesu = 0;
                m_m = 's';
            else
                [turn,anglesu] = pls_go_straight_up(s_cmd,s_rply);
                heading = heading + anglesu;
                anglesu = 0;
            end
        end

        for ct = 1:2
            if  (u(1) > 3) && (u(2) > 1.1) && (u(4) > 1.1)&& (u(5) > 1.1)

                % If the way ahead is clear, go forward
                if stepcount >=2
                    movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
%                     if movement ~= 0
%                        m_m = 'w';
%                        p = sense_uold(ultra, M, p, m_u);  
%                     end
%                     movement = 0;
                end
                cmdstring = [strcat('d1-',num2str(speed*1.5)) newline];             % Build command string to move bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                if turn == 0
                    m_m = 'w';     
                elseif turn == 1
                    m_m = 'a';
                elseif turn == -1
                    m_m = 'd';
                end
                turn = 0;
                break

            elseif u(direct_i(ct)) > 6

                % If not, pick a random direction that is clear and go that way
                cmdstring = [strcat('r1-',num2str(direct(ct))) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                if direct(ct) == 90
                    turn = 1;   
                else
                    turn = -1;    
                end
                
                %             pls_go_straight_up(s_cmd,s_rply);
                break
            elseif ct == 2
                if u(2)>=u(5)
                    % If no directions clear, turn to the left
                    cmdstring = [strcat('r1-',num2str(rot_stuck)) newline];    % Rotate if stuck
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    turn = 1;
                elseif u(5)>u(2)
                    % If no directions clear, turn to the left
                    cmdstring = [strcat('r1--',num2str(rot_stuck)) newline];    % Rotate if stuck
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    turn = -1;
                end

            end
            
        end
        stepcount = stepcount+1;
        u_past = u;
        m_r = tcpclient_write(['i2' newline], s_cmd, s_rply); % read the i2 value
        %% determine m_u 
        m_u = determine_mu(u);
        if stepcount <2
            p = sense_u(ultra, M, p, m_u,Vec_sensors, M_one_side,M_tree,M_two_edge,M_four_open,M_two_side);
        else
            p = sense_uold(ultra, M, p, m_u);
        end
        imagesc(p);
%     title(['step: ' num2str(k)]);
        [p, heading] = move(p, M, heading, m_m);
        pause(0.1);
        

        
    else 
        
    end
    value = max(p,[],'all');
    if value >= 0.09
        [a,b] = Indexing(p); 
        fprintf(strcat('Localization Complete,Location is measured in # of blocks from origin, location is:',num2str([b,a])),newline); 
    end
    
end

%%
function[moveu24] = adjustmentu24(u,u_past,speed,s_cmd, s_rply)

u_matrix_twotimes = read_two_times(s_cmd,s_rply);

u2 = average(u_matrix_twotimes(:,2));
u4 = average(u_matrix_twotimes(:,4));
u5 = average(u_matrix_twotimes(:,5));
moveu24 = 0;
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
            moveu24 = speed;
        elseif (u2>u4 && u4>3.5) || (u2<u4 && u2<2)
            cmdstring = [strcat('r1-',num2str(-20)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            cmdstring = [strcat('r1-',num2str(13)) newline];  % Build command string to rotate bot
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            moveu24 = speed;
        end
        toler = 0.2;
        u2diff = straight (u(4),u_past(2),toler);
        u4diff = straight (u(4),u_past(4),toler);
        if (u2diff==1) || (u4diff==1)
            
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
function[turn,angle] = pls_go_straight_up(s_cmd,s_rply)
u = 0;  % Ultrasonic measurements
u_past = u;
count =0;
error = 1;
incre = 10;
tol = 0.03;
num = 1;
angle = 0;
u_matrix_fivetimes = 0;
turn=0;

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
        u_matrix_fivetimes = read_five_times(s_cmd,s_rply);
        u1 = average(u_matrix_fivetimes(:,num));
        cmdstring = [strcat('r1-',num2str(5)) newline];
        reply = tcpclient_write(cmdstring, s_cmd, s_rply)
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
        if ~straight(u(1),u_past(1),tol)
            if (angle<=720 &&angle>0) || (angle<=0 && (angle>=-720))
                cmdstring = [strcat('r1-',num2str(incre)) newline];
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                angle = angle+incre;
            else
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
    if wrapTo180(angle) > 90
        turn = 1;
        angle = wrapTo180(angle);
    elseif wrapTo180(angle) < -90
        turn = -1;
        angle = wrapTo180(angle);
    end
   
   
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
function[p,heading] = the_remainder(s_cmd,s_rply,world, mask, p, heading)
u = [0 0 0 0 0 0];  % Ultrasonic measurements
rot_stuck = 90;
stepcount = 0;

cmdstring = ['ua' newline];
u = tcpclient_write(cmdstring, s_cmd, s_rply);
r = rem((u(1)-1),12);
a = r/15;
u_first = u;
SenVal = 0;
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
SenVal = determine_mu(u);
p = sense_uold(world, mask, p, SenVal);
[p, heading] = move(p, mask, heading, 'w');

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
    SenVal = determine_mu(u);
    p = sense_uold(world, mask, p, SenVal);
    [p, heading] = move(p, mask, heading, 'a');
    
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
    SenVal = determine_mu(u);
    p = sense_uold(world, mask, p, SenVal); 
    [p, heading] = move(p, mask, heading, 'd');
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

%%
function [pnew, heading] = move(p, mask, heading, Move)
   %models movement error
   K = [0.1 0.1 0.1; 0.1 0.8 0.1; 0.1 0.1 0.1];
   
   %2D convolution
   pnew = conv2(p, K,'same');

   disp(['heading: ' num2str(heading)])
   
   %direction  
   if Move == 'a'
        heading = mod(heading + 90,360);
   elseif Move == 'd'
		heading = mod(heading - 90,360);
   elseif Move == 's'
		heading = mod(heading + 180,360);
   end
   
   col_move = cosd(heading);
   row_move = sind(heading);
   
   if col_move > 0
	   pnew = [zeros(size(pnew,1),1) pnew(:,1:end-1)];
   elseif col_move < 0
	   pnew = [pnew(:,2:end) zeros(size(pnew,1),1)];
   end
   
   if row_move < 0
	   pnew = [zeros(1,size(pnew,2)); pnew(1:end-1,:)];
   elseif row_move > 0
	   pnew = [pnew(2:end,:); zeros(1,size(pnew,2))];
   end

   %masking matrix
   pnew = pnew.*mask;

   %normalization
   pnew = pnew/sum(sum(pnew));
   disp(['move: ' Move ' heading: ' num2str(heading)])
  
end

%% 
function pnew = sense_uold(world, mask, p, SenVal)
  %models sensor reading
  pHit = 0.6; %default = 0.9
  pMiss = 0.2; %default = 0.1
  
  mult = pMiss*ones(size(world));
  mult(world == SenVal) = pHit;
  
  %2D multiplication step
  pnew = p.*mult;
  
  %masking matrix
  pnew = pnew.*mask;
   
  %normalization
  pnew = pnew./sum(sum(pnew));

end
%%
function pnew = sense_u(world, mask, p, SenVal,Vec_sensors, M_one_side,M_tree,M_two_edge,M_four_open,M_two_side)
  %models sensor reading
  pHit = 0.8; %default = 0.9
  pMiss = 0.2; %default = 0.1
  
  mult = pMiss*ones(size(world));
  mult(world == SenVal) = pHit;
  
  %2D multiplication step
  pnew = p.*mult;
  
  
  
  n = 0;
  for i = 1:length(Vec_sensors)
      if Vec_sensors(i) - 9.5 < 0
        n = n + 1;
      else 
      end 
  end
  
  if n == 1
      pnew = pnew.* M_one_side;
  elseif n == 3
      pnew = pnew.* M_tree;
  elseif n == 2
      if Vec_sensors(2)<9 && Vec_sensors(4)<9
          pnew = pnew.* M_two_side;
      elseif (Vec_sensors(2)<9 || Vec_sensors(4)<9) && (Vec_sensors(1)<9 || Vec_sensors(3)<9)
          pnew = pnew.* M_two_edge;     
      end 
  elseif n == 0
      pnew = pnew.* M_four_open;
      
%   else 
%       pnew = pnew;
  end
  %masking matrix
  pnew = pnew.*mask;
  %normalization
  pnew = pnew./sum(sum(pnew));

end
%%
function [m_u]= determine_mu(u)
    if (u(1)<12&&u(2)>4&&u(3)>7&&u(5)>4)||(u(2)<4&&u(1)>12&&u(3)>7&&u(5)>4)||(u(3)<4&&u(1)>12&&u(2)>4&&u(5)>4)||(u(5)<4&&u(1)>12&&u(3)>7&&u(2)>4)
                m_u = 1;
    elseif (u(1)<12&&u(2)<4&&u(3)>4&&u(5)>4)||(u(1)>4&&u(2)<4&&u(3)<12&&u(5)>4)||(u(1)>4&&u(2)>4&&u(3)<12&&u(5)<4)||(u(1)<12&&u(2)>4&&u(3)>4&&u(5)<4)
        m_u = 2;
    elseif (u(1)<12&&u(2)<4&&u(3)<12&&u(5)>4)||(u(1)>4&&u(2)<4&&u(3)<4&&u(5)<4)||(u(1)<4&&u(2)>4&&u(3)<4&&u(5)<4)||(u(1)<4&&u(2)<4&&u(3)>4&&u(5)<4)
        m_u = 3;
    elseif (u(1)>12&&u(2)>12&&u(3)>12&&u(5)>12)
        m_u = 0;
    else
        m_u = 5;
    end
end 

%%
function[a,b] = Indexing(p)

for i = 1:16
    for j = 1:32
        if p(i,j) == max(p,[],'all')
            a = 5-round(i/4);
            b = round(j/4);
        end
    end
end
    
end