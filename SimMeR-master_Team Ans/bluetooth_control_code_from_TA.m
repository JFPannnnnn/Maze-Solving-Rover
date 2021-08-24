clear
close all
clc

% Initialize tcp server to read and respond to algorithm commands
%{
[s_cmd, s_rply] = tcp_setup();
fopen(s_cmd);
fopen(s_rply);
%}
btInfo = instrhwinfo('Bluetooth','ROB2')
s_cmd = Bluetooth(btInfo.RemoteID, 1);
fopen(s_cmd);
s_rply = s_cmd;

% Robot Sensor Measurements
u = [0,0,0];  % Ultrasonic measurements
pos = [0,0,0];  % Position (x,y,rotation)
speed = 2;
rot_stuck = 90;
stepcount = 0;

while 1
%{
    str = input('enter message: ', 's');
    
    % Check if user wants to exit
    if strcmp(str,'exit')
        disp('Exiting program after closing all connections...')
        fclose(s_cmd);
        delete(instrfindall);
        break;
    end
    
    cmdstring = [str newline]; 
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);

    disp(['reply: ' reply])
%}
    
    for ct = 1:3
        cmdstring = [strcat('u',num2str(ct)) newline];
        u(ct) = str2double(tcpclient_write(cmdstring, s_cmd, s_rply));
    end
    
    disp('Ultrasonic:')
    disp(u)
    
    if (u(1) >= 6) % it is 10 inches 
        cmdstring = [strcat('d1-',num2str(speed)) newline];
    elseif (u(1) < 6) && (u(3) < 6)
        cmdstring = [strcat('r1-',num2str(40)) newline]; %num2str() this is to change the speed
    else
        cmdstring = [strcat('r1-',num2str(-40)) newline];
    end
        
    reply = tcpclient_write(cmdstring, s_cmd, s_rply);

end

% in reality, the rover is not moving in a straight line, because the motor
% is not rotating at same rate. 



% %% original code
% clear
% close all
% clc
% 
% % Initialize tcp server to read and respond to algorithm commands
% %{
% [s_cmd, s_rply] = tcp_setup();
% fopen(s_cmd);
% fopen(s_rply);
% %}
% btInfo = instrhwinfo('Bluetooth','ROB2')
% s_cmd = Bluetooth(btInfo.RemoteID, 1);
% fopen(s_cmd);
% s_rply = s_cmd;
% 
% % Robot Sensor Measurements
% u = [0,0,0];  % Ultrasonic measurements
% pos = [0,0,0];  % Position (x,y,rotation)
% speed = 1;
% rot_stuck = 90;
% stepcount = 0;
% 
% while 1
% %{
%     str = input('enter message: ', 's');
%     
%     % Check if user wants to exit
%     if strcmp(str,'exit')
%         disp('Exiting program after closing all connections...')
%         fclose(s_cmd);
%         delete(instrfindall);
%         break;
%     end
%     
%     cmdstring = [str newline]; 
%     reply = tcpclient_write(cmdstring, s_cmd, s_rply);
% 
%     disp(['reply: ' reply])
% %}
%     
%     for ct = 1:3
%         cmdstring = [strcat('u',num2str(ct)) newline];
%         u(ct) = str2double(tcpclient_write(cmdstring, s_cmd, s_rply));
%     end
%     
%     disp('Ultrasonic:')
%     disp(u)
%     
%     if (u(1) >= 4)
%         cmdstring = [strcat('d1-',num2str(speed)) newline];
%     elseif (u(1) < 4) && (u(3) < 4)
%         cmdstring = [strcat('r1-',num2str(30)) newline];
%     else
%         cmdstring = [strcat('r1-',num2str(-30)) newline];
%     end
%         
%     reply = tcpclient_write(cmdstring, s_cmd, s_rply);
% 
% end