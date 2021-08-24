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
pathfound = 0;
pls_makesameu4u5(s_cmd,s_rply);
correction = 0;
prompt = 'What is the drop-off? ';
maze_type = input(prompt) ; % drop off zone 1 bottom left, 2 bottom right, 3 cross road, 4 top right
%% localization, initialize the map
%initalization of the world
dim1 = 32; dim2 = 16;
locationindex = reshape(1:dim1*dim2,dim1,dim2)';
n = numel(locationindex);
rand('twister',5489);
bw = reshape(randi([0 1],n,1),dim2,dim1); %0 = black, 1 = white

%initalization of the world floodfill  % 12 means the wall
map_to_pickupdestination = [20 20 20 20 20 20 20 20 20 20;20 0 0 1 2 20 6 20 8 20;20 0 0 20 3 4 5 6 7 20;20 1 20 5 20 20 6 20 8 20;20 2 3 4 5 6 7 20 9 20;20 20 20 20 20 20 20 20 20 20];

if maze_type == 1 % Dropping zone bottom left corner
    map_to_dropoffdestination = [20 20 20 20 20 20 20 20 20 20;20 6 7 8 9 20 7 20 9 20;20 5 6 20 8 7 6 7 8 20;20 4 20 0 20 20 5 20 10 20;20 3 2 1 2 3 4 20 11 20;20 20 20 20 20 20 20 20 20 20];

elseif maze_type == 2  % Dropping zone bottom right  corner
    map_to_dropoffdestination = [20 20 20 20 20 20 20 20 20 20;20 10 9 8 7 20 5 20 3 20;20 11 10 20 6 5 4 3 2 20;20 12 20 10 20 20 5 20 1 20;20 11 10 9 8 7 6 20 0 20;20 20 20 20 20 20 20 20 20 20];

elseif maze_type ==3  % Dropping zone cross road
    map_to_dropoffdestination = [20 20 20 20 20 20 20 20 20 20;20 7 6 5 4 20 0 20 4 20;20 8 7 20 3 2 1 2 3 20;20 9 20 7 20 20 2 20 4 20;20 8 7 6 5 4 3 20 5 20;20 20 20 20 20 20 20 20 20 20];

elseif maze_type ==4  % Dropping zone top right corner
    map_to_dropoffdestination = [20 20 20 20 20 20 20 20 20 20;20 9 8 7 6 20 4 20 0 20;20 10 9 20 5 4 3 2 1 20;20 11 20 9 20 20 4 20 2 20;20 10 9 8 7 6 5 20 3 20;20 20 20 20 20 20 20 20 20 20];
end

dim1_big = 8; dim2_big = 4;
locationindex_floodfill = reshape(1:dim1_big*dim2_big,dim1_big,dim2_big)';

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
    cmdstring = ['ua-5' newline];
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
    newline;
    disp('Ultrasonic')
    disp(u);
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
        if (pathfound == 1 || pathfound == 3)&& correction == 1
            for i = 1:length(the_shortest_path)
                if the_shortest_path(i) == 1
                    cmdstring = [strcat('r1-',num2str(90)) newline];  
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    m_m = 'a';
                    [p, heading] = move(p, M, heading, m_m);
                    cmdstring = ['ua' newline];
                    u = tcpclient_write(cmdstring, s_cmd, s_rply);
                    for j = 1:4
                        if u(1) < 3
                            break    
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                    end
                elseif the_shortest_path(i) == 2
                    cmdstring = [strcat('r1-',num2str(-90)) newline];  
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    m_m = 'd';
                    [p, heading] = move(p, M, heading, m_m);
                    cmdstring = ['ua' newline];
                    u = tcpclient_write(cmdstring, s_cmd, s_rply);
                    for j = 1:4
                        if u(1) < 3
                            break    
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                    end
                elseif the_shortest_path(i) == 3
                    cmdstring = [strcat('r1-',num2str(180)) newline];  
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    m_m = 's';
                    [p, heading] = move(p, M, heading, m_m);
                    cmdstring = ['ua' newline];
                    u = tcpclient_write(cmdstring, s_cmd, s_rply);
                    for j = 1:4
                        if u(1) < 3
                            break    
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                    end
                    if mod(u(1),12)>3.3 && the_shortest_path (i+1) ~= 4 && u(1)<12
                        movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                        cmdstring = [strcat('d1-',num2str(mod(u(1),12)-3.3-movement*cos(pi/9))) newline];
                        reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    end
                elseif the_shortest_path(i) == 4      
                    for j = 1:4
                        if (u(1) < 3 || u(6) < 4.7) && (pathfound == 1)
                            break;
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                    end
                    if i < length(the_shortest_path)
                        if mod(u(1),12) > 3.3 && the_shortest_path (i+1) ~= 4 && u(1) < u(3) && u(1) < 15
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(mod(u(1),12)-3.3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                        elseif mod(u(3),12) > 3.3 && the_shortest_path (i+1) ~= 4 && u(1) > u(3) && u(3) < 15
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(-(mod(u(3),12)-3.3-movement*cos(pi/9)))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            
                        end
                        
%                     else
%                         movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
%                         cmdstring = [strcat('d1-',num2str(mod(u(1),12)-3-movement)) newline];
%                         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
%                         m_u = determine_mu(u);
%                         m_m = 'w';
%                         p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
%                         imagesc(p);
%                         [p, heading] = move(p, M, heading, m_m);
                    end
                    
                    
                end
                imagesc(p);
            %     title(['step: ' num2str(k)]);
                pause(0.1);
            end
            %% for testing 
            if pathfound == 3
                cmdstring = [strcat('d1-',num2str(-1.5)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                cmdstring = [strcat('g1-',num2str(180)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                pause(3);
                cmdstring = [strcat('d1-',num2str(-4)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                fprintf('Arrived Dropping Zone\n Push Enter to Continue');
                pause
            end
            %%
            pls_makesameu4u5(s_cmd,s_rply);
            pathfound = 2;
            fprintf('Arrving Loading Zone\n');   
        else
            if u(5)>35 && u(5)<60 && u(3)>12 && ((u(1)>20 && u(1) < 55))
                [p,heading] = the_remainder(s_cmd,s_rply,ultra, M, p, heading,Vec_sensors,M_four_open);
            elseif u(2) > 40 && u(2) < 60 && u(3)>20 && ((u(1)>12 && u(1)< 24) || (u(1) > 36 && u(1) < 48))
                    [p,heading] = the_remainder(s_cmd,s_rply,ultra, M, p, heading,Vec_sensors,M_four_open);
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
                    p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
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
                    p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
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
                    p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                    imagesc(p)
                    [p, heading] = move(p, M, heading, m_m);
                end
                
                
            end
            if abs(u_past(1)-u(1))<0.5 && abs(u_past(3)-u(3))<0.5 % if stuck
                if u(3) > 3
                    cmdstring = [strcat('d1--',num2str(2*speed)) newline];             % Build command string to move bot
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    pls_makesameu4u5(s_cmd,s_rply);
%                     heading = heading + anglesu;
%                     anglesu = 0;
                    
                else
                    pls_makesameu4u5(s_cmd,s_rply);
%                     heading = heading + anglesu;
%                     anglesu = 0;
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
                p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
            end
            imagesc(p);
            %     title(['step: ' num2str(k)]);
            [p, heading] = move(p, M, heading, m_m);
            pause(0.1);
        end
        value = max(p,[],'all');
        if pathfound < 1 
            if value >= 0.09
                if turn == 1
                    m_m = 'a';
                    [p, heading] = move(p, M, heading, m_m);
                elseif turn == -1
                    m_m = 'd';
                    [p, heading] = move(p, M, heading, m_m);
                end
                turn = 0;
                cmdstring = ['ua' newline];
                u = tcpclient_write(cmdstring, s_cmd, s_rply);
                
                [a,b] = Indexing(p);
                x_vec = [a,b];
                 % add a function to check the rover's position 
                correction = wall_detection(s_cmd,s_rply,map_to_pickupdestination,x_vec,heading);
                if correction ==1
                    the_shortest_path = find_shortestPath(map_to_pickupdestination,x_vec,heading);
                    fprintf(strcat('location is:',num2str([b,a])));
                    pathfound = pathfound + 1; 
                end
            end
        %% to find the block 
        elseif pathfound ==2 
            correction = 0;
            the_shortest_path =[];
            if value >= 0.09
                [a,b] = Indexing(p);
                x_vec = [a,b];
                correction = wall_detection(s_cmd,s_rply,map_to_dropoffdestination,x_vec,heading);
                if correction ==1&& a+1 == 2 && b+1 ==2 %% reach the top left corner
                    if u(5)>10
                        the_shortest_path = [2,2,2];
                    elseif u(2)>10
                        the_shortest_path = [1,1,1];
                    end
                end
            end
            
            gripper_check = 0;
            
            
            
            for i = 1:length(the_shortest_path)
                if the_shortest_path(i) == 1
                    cmdstring = [strcat('r1-',num2str(90)) newline];  
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    m_m = 'a';
                    [p, heading] = move(p, M, heading, m_m);
                    cmdstring = ['ua' newline];
                    u = tcpclient_write(cmdstring, s_cmd, s_rply);
                    for j = 1:4
                        if u(1) < 3
                            break    
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                        if u(6)<u(1) && u(6)<6.5
                            break
                        end
                    end
                elseif the_shortest_path(i) == 2
                    cmdstring = [strcat('r1-',num2str(-90)) newline];  
                    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                    m_m = 'd';
                    [p, heading] = move(p, M, heading, m_m);
                    cmdstring = ['ua' newline];
                    u = tcpclient_write(cmdstring, s_cmd, s_rply);
                    for j = 1:4
                        if u(1) < 3
                            break    
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                        if u(6)<u(1) && u(6)<6.5
                            break
                        end
                    end
               
                elseif the_shortest_path(i) == 4
                    
                    for j = 1:4
                        if u(1) < 3 || u(6) < 4.7
                            break;
                        else
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            cmdstring = ['ua' newline];
                            u_past = u;
                            u = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                            movement = 0;
                        end
                        if u(6)<u(1) && u(6)<6.5
                            break
                        end
                    end
                    if i < length(the_shortest_path)
                        if mod(u(1),12) > 3.3 && the_shortest_path (i+1) ~= 4 && u(1) < u(3) && u(1) < 15
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(mod(u(1),12)-3.3-movement*cos(pi/9))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            m_u = determine_mu(u);
                            m_m = 'w';
                            p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
                            imagesc(p);
                            [p, heading] = move(p, M, heading, m_m);
                        elseif mod(u(3),12) > 3.3 && the_shortest_path (i+1) ~= 4 && u(1) > u(3) && u(3) < 15
                            movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
                            cmdstring = [strcat('d1-',num2str(-(mod(u(3),12)-3.3-movement*cos(pi/9)))) newline];
                            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                            
                        end
%                     else
%                         movement = adjustmentu24(u,u_past,speed,s_cmd, s_rply);
%                         cmdstring = [strcat('d1-',num2str(mod(u(1),12)-3-movement)) newline];
%                         reply = tcpclient_write(cmdstring, s_cmd, s_rply);
%                         m_u = determine_mu(u);
%                         m_m = 'w';
%                         p = sense_uold(ultra, M, p, m_u,Vec_sensors,M_four_open);
%                         imagesc(p);
%                         [p, heading] = move(p, M, heading, m_m);
                    end
                    
                    
                end
                imagesc(p);
            %     title(['step: ' num2str(k)]);
                pause(0.1);
                if u(6)<u(1) && u(6)<6.5
                    break    % out of the loop
                end
            end
            cmdstring = ['ua' newline];
            u = tcpclient_write(cmdstring, s_cmd, s_rply);
            
            if u(6)<u(1) && u(6)< 6.5
                gripper_check = grabbing(heading,s_cmd, s_rply);  % grab the block 
                if gripper_check ==1
                    pathfound = pathfound+1;
                    
                end 
            end
            
            
        end       
            
            
%% go back home            
        if pathfound == 3
            correction = 0;
            if value >= 0.09
            [a,b] = Indexing(p);
            x_vec = [a,b];
            correction = wall_detection(s_cmd,s_rply,map_to_dropoffdestination,x_vec,heading);
                if correction ==1
                    the_shortest_path = find_shortestPath(map_to_dropoffdestination,x_vec,heading);
                    fprintf(strcat('location is:',num2str([b,a])));
                    
                end
            end
        end
        
        
         
        
    end
    
end

%%
function[moveu24] = adjustmentu24(u,u_past,speed,s_cmd, s_rply)

    cmdstring = ['ua-5' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    u2 = u(2);
    u4 = u(4);
    u5 = u(5);
    moveu24 = 0;
    if u(2) > 12 && u(5) > 12
    else
        if abs(u(2)-u_past(2))/u_past(2) < 5 || abs(u(5)-u_past(5))/u_past(5) < 5
            if (u2<u5 && u2>3.5) || (u2>u5 && (u4<2.1)||u5<2.1)
                cmdstring = [strcat('r1-',num2str(20)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                cmdstring = [strcat('d1-',num2str(speed)) newline];             % Build command string to move bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                cmdstring = [strcat('r1-',num2str(-13)) newline];  % Build command string to rotate bot
                reply = tcpclient_write(cmdstring, s_cmd, s_rply);
                moveu24 = speed;
            elseif (u2>u5 && u5>3.5) || (u2<u5 && u2<2.3)
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
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
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
        cmdstring = ['ua-5' newline];
        u = tcpclient_write(cmdstring, s_cmd, s_rply);
        u4 = u(4);
        u5 = u(5);
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
%           angle = angle+5;
        else
            beta = -5; % The spacing between u4 and u5 is ~2 inches
            cmdstring = [strcat('r1-',num2str(beta)) newline];
            reply = tcpclient_write(cmdstring, s_cmd, s_rply);
            %angle = angle - 5;
            
        end
    else
        a = 0;
    end
end
end




%%
% function[u_matrix_twotimes] = read_two_times(s_cmd,s_rply)
% u_two = [0 0 0 0 0 0];
% u_matrix_twotimes = zeros(2,6);
% for i = 1:2
%     cmdstring = ['ua' newline];
%     u_two = tcpclient_write(cmdstring, s_cmd, s_rply);
%     if length(u_two) == 6
%         for j = 1:6
%             u_matrix_twotimes(i,j) = u_two(j);
%         end
%     else
%         break
%     end
% end
% end
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
function[p,heading] = the_remainder(s_cmd,s_rply,ultra, M, p, heading,Vec_sensors, M_four_open)
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
p = sense_uold(ultra, M, p, SenVal,Vec_sensors,M_four_open);
[p, heading] = move(p, M, heading, 'w');

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
    p = sense_uold(ultra, M, p, SenVal,Vec_sensors,M_four_open);
    [p, heading] = move(p, M, heading, 'a');
    
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
    p = sense_uold(ultra, M, p, SenVal,Vec_sensors,M_four_open);
    [p, heading] = move(p, M, heading, 'd');
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
function pnew = sense_uold(world, mask, p, SenVal,Vec_sensors,M_four_open)
%models sensor reading
pHit = 0.6; %default = 0.9
pMiss = 0.2; %default = 0.1

mult = pMiss*ones(size(world));
mult(world == SenVal) = pHit;

%2D multiplication step
pnew = p.*mult;
n = 0;
for i = 1:length(Vec_sensors)
    if Vec_sensors(i) - 11 < 0
        n = n + 1;
    else
    end
end
if n==0
    pnew = pnew.* M_four_open;   
end

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
            a = ceil(i/4);
            b = ceil(j/4);
        end
    end
end

end

%%Proposed function
function[str_m_m] = find_shortestPath(map_to_pickupdestination,vec,heading)
str_m_m = []; % output is [1 2 3 4] to indicate the direction
a = vec(1);  % vec is the location of the rover
b = vec(2);
a_row = a+1; %convert it to row and column
b_col = b+1; % because the find_shortestpath has wall (12) around it, so we need to add one on col and row
i = 0;
delta_heading = 0;
m_m = 0;    % m_m is to set up the value of the rover
m = map_to_pickupdestination(a,b); % output the value is the floodfill number
while m > 0
    
    i = i + 1;
    [value,index] = min([map_to_pickupdestination(a_row + 1,b_col);map_to_pickupdestination(a_row,b_col + 1);map_to_pickupdestination(a_row - 1,b_col);map_to_pickupdestination(a_row,b_col-1);]);
    %     [x_shortest,y_shortest] = the_index(:,index)
    if index == 1  % the small block is on the down
        %go heading = 270 (south) (relative to global coordinates)
        m_m = get_next_heading(270,heading); % to find the direction
        heading = 270;
        str_m_m(i) = m_m;
    elseif index == 2 % the smallest block is on the right
        %go heading = 0 (east) (relative to global coordinates)
        m_m = get_next_heading(0,heading);
        heading = 0;
        str_m_m(i) = m_m;
    elseif index == 3  % the smallest block is on the forward
        %go heading = 90 (north) (relative to global coordinates)
        m_m = get_next_heading(90,heading);
        heading = 90;
        str_m_m(i) = m_m;
    else
        %go heading = 180 (west) (relative to global coordinates)
        m_m = get_next_heading(180,heading);
        heading = 180;
        str_m_m(i) = m_m;
    end
    
    the_index = [a_row + 1,b_col;a_row,b_col + 1;a_row - 1,b_col;a_row,b_col-1;]; % to update the a_row and b_row, for the next move
    next_block = the_index(index,:);
    a_row = next_block(1);
    b_col = next_block(2);
    m = map_to_pickupdestination(a_row,b_col);
    % updat the new row and column
    %also we need to decide if we want it to turn around or to go straight
    %to follow the shortest path
    
end
end

%%
function[str_m_m] = get_next_heading(desired_heading,actual_heading)
delta = (desired_heading - actual_heading); % difference to decide where to turn
str_m_m = 0;
if actual_heading == 0
    if delta > 0 && desired_heading <= 180
        if delta  == 90
            str_m_m = 1; % 1 'a' = turn left, correct me if im wrong
        else
            str_m_m = 3;
        end
    elseif delta > 0 && desired_heading > 180 %270 is the only possibility
        str_m_m = 2; % 2 'd' = turn right, correct me if im wrong
    end
end
if delta > 0
    if delta  == 90
        str_m_m = 1; % 'a' = turn left, correct me if im wrong
    elseif delta == 180
        str_m_m = 3;
    else
        str_m_m = 2;
    end
    
elseif delta < 0
    
    if abs(delta) == 90
        str_m_m = 2; % 'd' = turn right, correct me if im wrong
    elseif abs(delta) == 180
        str_m_m = 3;
    else
        str_m_m = 1;
    end
elseif delta == 0
    str_m_m = 4;  % 4 means go straight
end
end
%% wall detection 
function [correction] = wall_detection(s_cmd,s_rply,map_to_pickupdestination,x_vec,heading)
    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    
    front_wall = 0;
    left_wall = 0;
    right_wall = 0;
    back_wall = 0;
    a = x_vec(1)+1;
    b = x_vec(2)+1;
    
    
    correction = 0;
    
    if u(1)<4 % the front has wall
        front_wall = 20;
    end 
    if u(2)<4
        left_wall = 20;
    end
    if u(3) <4
        back_wall = 20;
    end
    if (u(4)<4) &&(u(5)<4)
        right_wall =20;
    end 
    
    if map_to_pickupdestination(a+1,b) ~=20
        map_to_pickupdestination(a+1,b) =0;
    end
    if map_to_pickupdestination(a-1,b) ~=20
        map_to_pickupdestination(a-1,b) =0;
    end
    if map_to_pickupdestination(a,b+1) ~=20
        map_to_pickupdestination(a,b+1) =0;
    end
    if map_to_pickupdestination(a,b-1) ~=20
        map_to_pickupdestination(a,b-1) =0;
    end
    
    if heading ==0
        if (map_to_pickupdestination(a+1,b) == right_wall && map_to_pickupdestination(a-1,b)==left_wall && map_to_pickupdestination(a,b+1) == front_wall&&map_to_pickupdestination(a,b-1)==back_wall) 
            correction = 1;
            
        end
    elseif heading == 90
        if (map_to_pickupdestination(a+1,b) == back_wall && map_to_pickupdestination(a-1,b)==front_wall && map_to_pickupdestination(a,b+1) == right_wall&&map_to_pickupdestination(a,b-1)==left_wall) 
            correction = 1;
            
        end
    elseif heading == 180
        if (map_to_pickupdestination(a+1,b) == left_wall && map_to_pickupdestination(a-1,b)==right_wall && map_to_pickupdestination(a,b+1) == back_wall&&map_to_pickupdestination(a,b-1)==front_wall) 
            correction = 1;
            
        end
    elseif heading == 270
        if (map_to_pickupdestination(a+1,b) == front_wall && map_to_pickupdestination(a-1,b)==back_wall && map_to_pickupdestination(a,b+1) == left_wall&&map_to_pickupdestination(a,b-1)==right_wall) 
            correction = 1;
        end  
    end
        

end 

%% block finding 

%% block grabbing 
function [gripper_check] = grabbing(heading, s_cmd, s_rply)
    
    gripper_check = 0;
    
    cmdstring = [strcat('d1-',num2str(-4)) newline];  % Build command string to rotate bot
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    cmdstring = [strcat('g1-',num2str(180)) newline];  % Build command string to rotate bot
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    pause(3);
    cmdstring = ['ua' newline];
    u = tcpclient_write(cmdstring, s_cmd, s_rply);
    cmdstring = [strcat('d1-',num2str(u(6)-4.7)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    cmdstring = [strcat('g1-',num2str(40)) newline];
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);
    pause(3);
    %         if u(6)<2
    gripper_check = 1;
    %         end
end