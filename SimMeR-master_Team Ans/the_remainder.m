function[] = the_remainder(s_cmd,s_rply)
u = [0,0,0,0,0];  % Ultrasonic measurements
rot_stuck = 90;
stepcount = 0;

for m = 1:5
    cmdstring = [strcat('u',num2str(m)) newline];
    u(m) = tcpclient_write(cmdstring, s_cmd, s_rply);
end
r = rem((u(1)-1),12);
a = r/15;
u_first = u
for i = 1:10
    for m = 1:5
        cmdstring = [strcat('u',num2str(m)) newline];
        u(m) = tcpclient_write(cmdstring, s_cmd, s_rply);
    end
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
            for m = 1:5
                cmdstring = [strcat('u',num2str(m)) newline];
                u(m) = tcpclient_write(cmdstring, s_cmd, s_rply);
            end
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
            for m = 1:5
                cmdstring = [strcat('u',num2str(m)) newline];
                u(m) = tcpclient_write(cmdstring, s_cmd, s_rply);
            end
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


